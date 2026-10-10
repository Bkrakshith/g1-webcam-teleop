"""OBSBOT Tiny 3 pan / tilt / zoom control over standard UVC (V4L2) controls.

Why V4L2 and not the OBSBOT SDK (libdev.so v1.0.2 in ../obsbot-camera-control/sdk)?
  * The SDK does not know the Tiny 3: its product enum ends at Tail 2S and a probe
    (tools/sdk_probe.cpp in the fork) fails with "getDevUuidR ret -3 / uvc device init failed"
    on this camera (USB 3564:ff02). Its vendor-protocol messages go unanswered.
  * The Tiny 3 exposes the standard UVC camera-terminal controls pan_absolute, tilt_absolute
    (arc-seconds, 1 deg steps), zoom_absolute (0..100) through uvcvideo. They work while
    another fd streams video, need no root, no vendor blob and no C++ shim.

Measured on this unit (2026-09-29, 1280x720 MJPG):
  * +pan turns the camera to its LEFT (scene shifts right); +tilt turns it UP.
  * ~17 px/deg at 1280 px width, zoom 0  ->  f0 ~= 975 px, HFOV ~= 67 deg.
  * zoom_absolute 30 ~= 1.85x magnification (roughly 1 + 0.028*zoom).
  * motion starts ~0.08 s after the command; a 10 deg pan step settles in ~1.1 s.
  * read-back (G_CTRL) returns the last commanded target instantly (uvcvideo cache /
    device reports the target), NOT the live gimbal angle. `GimbalModel` below therefore
    estimates the actual angle from the command history (latency + rate limit) and that
    estimate is what gimbal compensation should use.
  * The Tiny 3's own AI tracking cannot be switched off through UVC or the v1.0.2 SDK.
    Turn it off on the camera (gesture / OBSBOT Center) - see README. Our absolute
    commands override it only momentarily.

Internal convention used by everything above this module:
  yaw_right_deg  = camera turned to its right  (= -pan_absolute_deg on the Tiny 3)
  pitch_up_deg   = camera turned up            (= +tilt_absolute_deg)
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

from .v4l2 import V4L2Device, find_device

ARCSEC_PER_DEG = 3600.0


@dataclass
class PTZLimits:
    yaw_deg: Tuple[float, float] = (-60.0, 60.0)     # soft limits (hardware: +-130)
    pitch_deg: Tuple[float, float] = (-35.0, 30.0)   # soft limits (hardware: +-90)
    zoom: Tuple[float, float] = (0.0, 60.0)          # soft limits (hardware: 0..100)


@dataclass
class GimbalDynamics:
    latency_s: float = 0.08
    max_rate_deg_s: float = 12.0     # observed ~10-12 deg/s for large steps
    zoom_rate_per_s: float = 30.0    # zoom units per second (0->30 took ~1.1 s)


@dataclass
class CameraOptics:
    """Pinhole model as a function of zoom. Numbers measured on the Tiny 3 at 1280 px width."""
    f0_px_at_1280: float = 975.0
    zoom_gain: float = 0.028          # magnification = 1 + zoom_gain * zoom

    def magnification(self, zoom: float) -> float:
        return 1.0 + self.zoom_gain * max(0.0, zoom)

    def focal_px(self, width_px: int, zoom: float) -> float:
        return self.f0_px_at_1280 * (width_px / 1280.0) * self.magnification(zoom)

    def hfov_deg(self, width_px: int, zoom: float) -> float:
        return math.degrees(2.0 * math.atan(0.5 * width_px / self.focal_px(width_px, zoom)))


class _Axis:
    """Rate-limited first-order follower with a pure delay, driven by commands."""

    def __init__(self, value: float, rate: float, latency: float, t0: float):
        self.value = value
        self.rate = rate
        self.latency = latency
        self.pending = []           # (t_effective, target)
        self.target = value
        self.t = t0

    def command(self, target: float, t: float):
        self.pending.append((t + self.latency, target))

    def advance(self, t: float) -> float:
        while self.t < t:
            # next event boundary
            t_next = t
            if self.pending and self.pending[0][0] <= t:
                t_next = max(self.t, self.pending[0][0])
            dt = t_next - self.t
            step = self.rate * dt
            err = self.target - self.value
            self.value += max(-step, min(step, err))
            self.t = t_next
            while self.pending and self.pending[0][0] <= self.t:
                self.target = self.pending.pop(0)[1]
            if t_next == t:
                break
        return self.value


class GimbalModel:
    def __init__(self, yaw=0.0, pitch=0.0, zoom=0.0, dyn: GimbalDynamics = GimbalDynamics(),
                 t0: Optional[float] = None):
        self.dyn = dyn
        t0 = time.monotonic() if t0 is None else t0
        self.yaw = _Axis(yaw, dyn.max_rate_deg_s, dyn.latency_s, t0)
        self.pitch = _Axis(pitch, dyn.max_rate_deg_s, dyn.latency_s, t0)
        self.zoom = _Axis(zoom, dyn.zoom_rate_per_s, dyn.latency_s, t0)
        self._lock = threading.Lock()

    def command(self, yaw=None, pitch=None, zoom=None, t=None):
        t = time.monotonic() if t is None else t
        with self._lock:
            for ax, v in ((self.yaw, yaw), (self.pitch, pitch), (self.zoom, zoom)):
                if v is not None:
                    ax.advance(t)
                    ax.command(v, t)

    def estimate(self, t=None) -> Tuple[float, float, float]:
        t = time.monotonic() if t is None else t
        with self._lock:
            return self.yaw.advance(t), self.pitch.advance(t), self.zoom.advance(t)


@dataclass
class PTZState:
    yaw_cmd: float = 0.0
    pitch_cmd: float = 0.0
    zoom_cmd: float = 0.0
    yaw_est: float = 0.0
    pitch_est: float = 0.0
    zoom_est: float = 0.0
    readback: Optional[Tuple[float, float, float]] = None
    hardware: bool = False
    notes: list = field(default_factory=list)


class ObsbotPTZ:
    """Absolute + relative PTZ in the internal (yaw right / pitch up) convention.

    Commands are rate limited to one hardware write per `min_cmd_interval_s` per axis and
    quantised to the 1 deg hardware step (zoom: 1 unit), so a controller can call
    `set_absolute` at frame rate without flooding USB.
    """

    PAN_SIGN = -1.0    # yaw_right = PAN_SIGN * pan_absolute   (Tiny 3: +pan = left)
    TILT_SIGN = +1.0   # pitch_up  = TILT_SIGN * tilt_absolute

    def __init__(self, device: Optional[str] = None, limits: PTZLimits = PTZLimits(),
                 dynamics: GimbalDynamics = GimbalDynamics(), optics: CameraOptics = CameraOptics(),
                 min_cmd_interval_s: float = 0.05):
        self.limits = limits
        self.optics = optics
        self.min_cmd_interval_s = min_cmd_interval_s
        path = device or find_device()
        if path is None:
            raise RuntimeError("no OBSBOT / UVC PTZ camera found")
        self.dev = V4L2Device(path)
        for key in ("pan_absolute", "tilt_absolute", "zoom_absolute"):
            if not self.dev.has(key):
                raise RuntimeError(f"{path} has no {key} control")
        y, p, z = self.readback()
        self.model = GimbalModel(y, p, z, dynamics)
        self._cmd = [y, p, z]
        self._last_write = [0.0, 0.0, 0.0]
        self._lock = threading.Lock()

    # ---- hardware conversion ---------------------------------------------------------
    def readback(self) -> Tuple[float, float, float]:
        """Last commanded (device-reported) angles in the internal convention."""
        pan = self.dev.get("pan_absolute") / ARCSEC_PER_DEG
        tilt = self.dev.get("tilt_absolute") / ARCSEC_PER_DEG
        zoom = float(self.dev.get("zoom_absolute"))
        return self.PAN_SIGN * pan, self.TILT_SIGN * tilt, zoom

    def _write(self, axis: int, value: float, t: float, force: bool = False):
        lo, hi = (self.limits.yaw_deg, self.limits.pitch_deg, self.limits.zoom)[axis]
        value = max(lo, min(hi, value))
        q = float(round(value))            # hardware resolution: 1 deg / 1 zoom unit
        if not force and q == self._cmd[axis]:
            return
        if not force and t - self._last_write[axis] < self.min_cmd_interval_s:
            return
        if axis == 0:
            self.dev.set("pan_absolute", int(round(self.PAN_SIGN * q * ARCSEC_PER_DEG)))
            self.model.command(yaw=q, t=t)
        elif axis == 1:
            self.dev.set("tilt_absolute", int(round(self.TILT_SIGN * q * ARCSEC_PER_DEG)))
            self.model.command(pitch=q, t=t)
        else:
            self.dev.set("zoom_absolute", int(q))
            self.model.command(zoom=q, t=t)
        self._cmd[axis] = q
        self._last_write[axis] = t

    # ---- public API ------------------------------------------------------------------
    def set_absolute(self, yaw: Optional[float] = None, pitch: Optional[float] = None,
                     zoom: Optional[float] = None, force: bool = False):
        t = time.monotonic()
        with self._lock:
            for i, v in enumerate((yaw, pitch, zoom)):
                if v is not None:
                    self._write(i, v, t, force)

    def move_relative(self, dyaw: float = 0.0, dpitch: float = 0.0, dzoom: float = 0.0):
        with self._lock:
            y, p, z = self._cmd
        self.set_absolute(y + dyaw, p + dpitch, z + dzoom, force=True)

    def center(self, zoom: Optional[float] = 0.0):
        self.set_absolute(0.0, 0.0, zoom, force=True)

    def commanded(self) -> Tuple[float, float, float]:
        with self._lock:
            return tuple(self._cmd)

    def estimate(self, t: Optional[float] = None) -> Tuple[float, float, float]:
        """Best estimate of the actual (moving) gimbal yaw/pitch/zoom at time t."""
        return self.model.estimate(t)

    def disable_ai_tracking(self) -> Tuple[bool, str]:
        return (False, "Tiny 3 AI tracking is not reachable via UVC or OBSBOT SDK v1.0.2; "
                       "switch it off on the camera (gesture / OBSBOT Center).")

    def state(self) -> PTZState:
        y, p, z = self.commanded()
        ye, pe, ze = self.estimate()
        return PTZState(y, p, z, ye, pe, ze, self.readback(), True)

    def close(self):
        self.dev.close()


class SimulatedPTZ:
    """Same interface without hardware (unit tests, recorded clips, cameras without PTZ)."""

    def __init__(self, limits: PTZLimits = PTZLimits(), dynamics: GimbalDynamics = GimbalDynamics(),
                 optics: CameraOptics = CameraOptics(), clock=time.monotonic):
        self.limits = limits
        self.optics = optics
        self.clock = clock
        self.model = GimbalModel(0.0, 0.0, 0.0, dynamics, t0=clock())
        self._cmd = [0.0, 0.0, 0.0]
        self.writes = 0

    def set_absolute(self, yaw=None, pitch=None, zoom=None, force=False):
        t = self.clock()
        for i, v in enumerate((yaw, pitch, zoom)):
            if v is None:
                continue
            lo, hi = (self.limits.yaw_deg, self.limits.pitch_deg, self.limits.zoom)[i]
            q = float(round(max(lo, min(hi, v))))
            if q != self._cmd[i] or force:
                self._cmd[i] = q
                self.writes += 1
                self.model.command(**{("yaw", "pitch", "zoom")[i]: q}, t=t)

    def move_relative(self, dyaw=0.0, dpitch=0.0, dzoom=0.0):
        y, p, z = self._cmd
        self.set_absolute(y + dyaw, p + dpitch, z + dzoom, force=True)

    def center(self, zoom=0.0):
        self.set_absolute(0.0, 0.0, zoom, force=True)

    def commanded(self):
        return tuple(self._cmd)

    def readback(self):
        return tuple(self._cmd)

    def estimate(self, t=None):
        return self.model.estimate(self.clock() if t is None else t)

    def disable_ai_tracking(self):
        return True, "simulated"

    def state(self) -> PTZState:
        y, p, z = self._cmd
        ye, pe, ze = self.estimate()
        return PTZState(y, p, z, ye, pe, ze, (y, p, z), False)

    def close(self):
        pass
