"""Hand-following auto-framing for a PTZ webcam.

The tracker (which owns the capture) calls `AutoFramer.update()` once per processed frame
with the hand and forearm boxes it found. The framer keeps the union box centred with a
rate-limited PI controller on yaw/pitch, zooms out when fingertips approach the frame
edge / zooms in when the hands are small, holds still inside a deadband, and sweeps to
search when the hands have been lost for a while.

All boxes are normalised image coordinates (x0, y0, x1, y1) in [0, 1] of the *current*
camera image. Angles use the internal convention of camera.py (yaw right +, pitch up +).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

Box = Tuple[float, float, float, float]


@dataclass
class FramingConfig:
    enabled: bool = True
    # centring
    target_center: Tuple[float, float] = (0.5, 0.5)
    deadband_enter_deg: float = 3.0      # start correcting when |error| exceeds this
    deadband_exit_deg: float = 1.0       # stop correcting once |error| is below this
    kp: float = 1.6                      # 1/s   (target velocity per deg of error)
    ki: float = 0.25                     # 1/s^2
    integral_limit_deg: float = 6.0
    max_rate_deg_s: float = 8.0          # below the gimbal's ~12 deg/s so the model stays valid
    max_accel_deg_s2: float = 30.0
    # zoom
    size_low: float = 0.40               # union box (max of w, h) below this -> zoom in
    size_high: float = 0.72              # above this -> zoom out
    edge_margin: float = 0.05            # fingertip closer than this to an edge -> zoom out fast
    zoom_in_rate: float = 5.0            # zoom units / s
    zoom_out_rate: float = 12.0
    zoom_out_fast_rate: float = 30.0
    # loss handling
    hold_s: float = 1.0                  # hold still this long after losing the hands
    search_after_s: float = 1.0
    search_yaw_deg: Tuple[float, float] = (-25.0, 25.0)
    search_pitch_deg: float = -5.0
    search_zoom: float = 0.0
    search_rate_deg_s: float = 6.0
    search_max_s: float = 25.0           # then park at the home pose and wait (no endless sweeping)


@dataclass
class FramingOutput:
    state: str                      # "tracking" | "holding" | "searching" | "off"
    yaw_target: float
    pitch_target: float
    zoom_target: float
    union_box: Optional[Box] = None
    error_deg: Tuple[float, float] = (0.0, 0.0)
    fingers_at_edge: bool = False
    notes: List[str] = field(default_factory=list)

    @property
    def contract_state(self) -> str:
        """Contract values are tracking|searching|off; 'holding' is reported as tracking."""
        return {"holding": "tracking"}.get(self.state, self.state)


def union_box(boxes: Sequence[Box]) -> Optional[Box]:
    boxes = [b for b in boxes if b is not None]
    if not boxes:
        return None
    return (min(b[0] for b in boxes), min(b[1] for b in boxes),
            max(b[2] for b in boxes), max(b[3] for b in boxes))


def points_box(points: Sequence[Tuple[float, float]], pad: float = 0.0) -> Optional[Box]:
    pts = list(points)
    if not pts:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)


def pixel_to_angle(u: float, fov_deg: float) -> float:
    """Normalised image coordinate offset from centre -> angle (deg), pinhole model."""
    return math.degrees(math.atan((u - 0.5) * 2.0 * math.tan(math.radians(fov_deg) / 2.0)))


class _PIAxis:
    def __init__(self, cfg: FramingConfig):
        self.cfg = cfg
        self.active = False
        self.integral = 0.0
        self.rate = 0.0

    def reset(self):
        self.active = False
        self.integral = 0.0
        self.rate = 0.0

    def step(self, image_err: float, abs_err: float, dt: float, saturated: bool) -> float:
        """image_err: angle between desired and current optical axis (deadband input).
        abs_err: desired absolute angle - current commanded target (PI input).
        Returns the target velocity (deg/s)."""
        c = self.cfg
        if not self.active and abs(image_err) > c.deadband_enter_deg:
            self.active = True
        elif self.active and abs(image_err) < c.deadband_exit_deg:
            self.active = False
        if self.active:
            if not saturated:
                self.integral = max(-c.integral_limit_deg,
                                    min(c.integral_limit_deg, self.integral + abs_err * dt))
            want = c.kp * abs_err + c.ki * self.integral
        else:
            self.integral *= max(0.0, 1.0 - 2.0 * dt)      # leak while idle
            want = 0.0
        want = max(-c.max_rate_deg_s, min(c.max_rate_deg_s, want))
        dv = c.max_accel_deg_s2 * dt
        self.rate += max(-dv, min(dv, want - self.rate))
        return self.rate


class AutoFramer:
    def __init__(self, ptz, config: FramingConfig = FramingConfig()):
        self.ptz = ptz
        self.cfg = config
        y, p, z = ptz.commanded()
        self.yaw, self.pitch, self.zoom = float(y), float(p), float(z)
        self.ax_yaw = _PIAxis(config)
        self.ax_pitch = _PIAxis(config)
        self.t_last: Optional[float] = None
        self.t_seen: Optional[float] = None
        self.search_dir = 1.0
        self.t_search: Optional[float] = None
        self.parked = False
        self.state = "off" if not config.enabled else "searching"

    @property
    def enabled(self) -> bool:
        return self.cfg.enabled

    def set_enabled(self, on: bool):
        self.cfg.enabled = on
        if not on:
            self.state = "off"
            self.ax_yaw.reset()
            self.ax_pitch.reset()

    def _clamp(self):
        lim = self.ptz.limits
        sat_y = not (lim.yaw_deg[0] < self.yaw < lim.yaw_deg[1])
        sat_p = not (lim.pitch_deg[0] < self.pitch < lim.pitch_deg[1])
        self.yaw = max(lim.yaw_deg[0], min(lim.yaw_deg[1], self.yaw))
        self.pitch = max(lim.pitch_deg[0], min(lim.pitch_deg[1], self.pitch))
        self.zoom = max(lim.zoom[0], min(lim.zoom[1], self.zoom))
        return sat_y, sat_p

    def update(self, t: float, image_size: Tuple[int, int], boxes: Sequence[Box],
               finger_points: Sequence[Tuple[float, float]] = ()) -> FramingOutput:
        cfg = self.cfg
        dt = 0.0 if self.t_last is None else max(0.0, min(0.2, t - self.t_last))
        self.t_last = t
        if not cfg.enabled:
            self.state = "off"
            return FramingOutput("off", self.yaw, self.pitch, self.zoom, union_box(boxes))

        w, h = image_size
        yaw_est, pitch_est, zoom_est = self.ptz.estimate(t)
        hfov = self.ptz.optics.hfov_deg(w, zoom_est)
        vfov = math.degrees(2 * math.atan(math.tan(math.radians(hfov) / 2) * h / w))
        box = union_box(boxes)
        at_edge = any(p[0] < cfg.edge_margin or p[0] > 1 - cfg.edge_margin or
                      p[1] < cfg.edge_margin or p[1] > 1 - cfg.edge_margin for p in finger_points)
        err = (0.0, 0.0)

        if box is not None:
            self.t_seen = t
            self.t_search = None
            self.parked = False
            if self.state == "searching":
                self.ax_yaw.reset()
                self.ax_pitch.reset()
            self.state = "tracking"
            cx = 0.5 * (box[0] + box[2])
            cy = 0.5 * (box[1] + box[3])
            ex = pixel_to_angle(cx - cfg.target_center[0] + 0.5, hfov)       # + = box right -> yaw right
            ey = -pixel_to_angle(cy - cfg.target_center[1] + 0.5, vfov)      # + = box high  -> pitch up
            err = (ex, ey)
            # PI acts on (desired absolute angle - commanded target); the desired absolute angle
            # uses the *estimated* current pose, so gimbal lag does not cause overshoot.
            sat_y, sat_p = self._clamp()
            vy = self.ax_yaw.step(ex, yaw_est + ex - self.yaw, dt, sat_y)
            vp = self.ax_pitch.step(ey, pitch_est + ey - self.pitch, dt, sat_p)
            self.yaw += vy * dt
            self.pitch += vp * dt
            size = max(box[2] - box[0], box[3] - box[1])
            if at_edge:
                self.zoom -= cfg.zoom_out_fast_rate * dt
            elif size > cfg.size_high:
                self.zoom -= cfg.zoom_out_rate * dt
            elif size < cfg.size_low:
                self.zoom += cfg.zoom_in_rate * dt
        else:
            lost_for = float("inf") if self.t_seen is None else t - self.t_seen
            if lost_for < cfg.hold_s:
                self.state = "holding"
                self.ax_yaw.reset()
                self.ax_pitch.reset()
            else:
                self.state = "searching"
                if self.t_search is None:
                    self.t_search = t
                self.parked = t - self.t_search > cfg.search_max_s
                # zoom out first, bring pitch to the search level, then sweep yaw back and forth
                self.zoom -= cfg.zoom_out_fast_rate * dt
                step = cfg.search_rate_deg_s * dt
                self.pitch += max(-step, min(step, cfg.search_pitch_deg - self.pitch))
                if self.parked:            # gave up sweeping: go home and wait for the operator
                    self.yaw += max(-step, min(step, 0.0 - self.yaw))
                elif zoom_est <= cfg.search_zoom + 5:
                    lo, hi = cfg.search_yaw_deg
                    self.yaw += self.search_dir * step
                    if self.yaw >= hi:
                        self.yaw, self.search_dir = hi, -1.0
                    elif self.yaw <= lo:
                        self.yaw, self.search_dir = lo, 1.0
        self._clamp()
        self.ptz.set_absolute(self.yaw, self.pitch, self.zoom)
        out = FramingOutput(self.state, self.yaw, self.pitch, self.zoom, box, err, at_edge)
        if self.state == "searching" and self.parked:
            out.notes.append("parked")
        return out

    def recenter(self):
        self.yaw = self.pitch = 0.0
        self.zoom = 0.0
        self.ax_yaw.reset()
        self.ax_pitch.reset()
        self.ptz.center(0.0)
