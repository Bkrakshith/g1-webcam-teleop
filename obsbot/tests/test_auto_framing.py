"""Auto-framing controller on synthetic boxes, closed loop against the gimbal model, and the
PTZ layer's hardware conversions (with a fake V4L2 device)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from obsbot_ptz import camera as cam_mod  # noqa: E402
from obsbot_ptz.auto_framing import AutoFramer, FramingConfig, pixel_to_angle, union_box  # noqa: E402
from obsbot_ptz.camera import CameraOptics, GimbalDynamics, GimbalModel, PTZLimits, SimulatedPTZ  # noqa: E402

W, H = 1280, 720


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class World:
    """Hands at a fixed direction (yaw/pitch of their centre, deg) with an angular size.
    Renders the normalised box the camera sees given the *actual* gimbal state."""

    def __init__(self, ptz, yaw=10.0, pitch=-6.0, size_deg=(20.0, 14.0)):
        self.ptz, self.yaw, self.pitch, self.size = ptz, yaw, pitch, size_deg

    def box(self, t):
        cy_, cp_, cz_ = self.ptz.estimate(t)
        f = self.ptz.optics.focal_px(W, cz_)

        def px(dyaw, dpitch):
            return (W / 2 + f * math.tan(math.radians(dyaw))) / W, (H / 2 - f * math.tan(math.radians(dpitch))) / H
        x0, y0 = px(self.yaw - self.size[0] / 2 - cy_, self.pitch + self.size[1] / 2 - cp_)
        x1, y1 = px(self.yaw + self.size[0] / 2 - cy_, self.pitch - self.size[1] / 2 - cp_)
        if x1 < 0 or x0 > 1 or y1 < 0 or y0 > 1:
            return None
        return (x0, y0, x1, y1)


def run(framer, world, clock, seconds, dt=1 / 30, lost=False, fingers_fn=None, record=None):
    n = int(seconds / dt)
    out = None
    for _ in range(n):
        clock.t += dt
        b = None if lost else world.box(clock.t)
        fingers = fingers_fn(b) if (fingers_fn and b) else ([(b[0], b[1]), (b[2], b[3])] if b else [])
        out = framer.update(clock.t, (W, H), [b] if b else [], fingers)
        if record is not None:
            record.append((clock.t, out, framer.ptz.estimate(clock.t)))
    return out


def make(cfg=None, limits=PTZLimits()):
    clock = Clock()
    ptz = SimulatedPTZ(limits=limits, clock=clock)
    return clock, ptz, AutoFramer(ptz, cfg or FramingConfig())


def test_union_and_pixel_angle():
    assert union_box([(0.1, 0.2, 0.3, 0.4), (0.2, 0.1, 0.5, 0.3)]) == (0.1, 0.1, 0.5, 0.4)
    assert union_box([]) is None
    assert pixel_to_angle(0.5, 60) == 0
    assert abs(pixel_to_angle(1.0, 60) - 30) < 1e-9


def test_centres_the_hands_and_settles():
    clock, ptz, fr = make()
    world = World(ptz, yaw=12.0, pitch=-7.0)
    rec = []
    run(fr, world, clock, 6.0, record=rec)
    yaw_est, pitch_est, _ = ptz.estimate(clock.t)
    # hands centred to within the hardware resolution (1 deg) + deadband
    assert abs(yaw_est - 12.0) <= 1.5 and abs(pitch_est + 7.0) <= 1.5
    # no big overshoot on the way
    assert max(r[2][0] for r in rec) <= 12.0 + 2.0
    # commanded target velocity respects the rate limit
    ys = np.array([r[1].yaw_target for r in rec])
    assert np.max(np.abs(np.diff(ys))) * 30 <= FramingConfig().max_rate_deg_s + 1e-6
    # and it stops writing once settled (deadband)
    writes = ptz.writes
    run(fr, world, clock, 2.0)
    assert ptz.writes == writes


def test_deadband_ignores_jitter():
    clock, ptz, fr = make()
    world = World(ptz, yaw=0.0, pitch=0.0, size_deg=(34.0, 20.0))   # size inside the zoom band
    rng = np.random.default_rng(0)
    for _ in range(90):
        clock.t += 1 / 30
        world.yaw, world.pitch = rng.normal(scale=0.8, size=2)     # +-0.8 deg jitter
        b = world.box(clock.t)
        fr.update(clock.t, (W, H), [b], [])
    assert ptz.writes == 0


def test_follows_moving_hands():
    clock, ptz, fr = make()
    world = World(ptz, yaw=0.0, pitch=0.0)
    for k in range(240):                                           # hands drift at 4 deg/s
        world.yaw = 4.0 * k / 30
        run(fr, world, clock, 1 / 30)
    yaw_est = ptz.estimate(clock.t)[0]
    assert abs(yaw_est - world.yaw) < 4.0


def test_zoom_in_when_small_and_out_at_edges():
    clock, ptz, fr = make()
    world = World(ptz, yaw=0.0, pitch=0.0, size_deg=(8.0, 6.0))    # small in the frame
    run(fr, world, clock, 4.0)
    z_in = ptz.commanded()[2]
    assert z_in > 5
    # fingers at the image edge -> zoom out fast
    clock2, ptz2, fr2 = make()
    ptz2.set_absolute(zoom=40, force=True)
    fr2.zoom = 40.0
    clock2.t += 5
    world2 = World(ptz2, yaw=0.0, pitch=0.0, size_deg=(12.0, 8.0))
    run(fr2, world2, clock2, 1.0, fingers_fn=lambda b: [(0.01, 0.5)])
    assert ptz2.commanded()[2] < 15


def test_zoom_out_when_box_large():
    clock, ptz, fr = make()
    ptz.set_absolute(zoom=30, force=True)
    fr.zoom = 30.0
    clock.t += 5
    world = World(ptz, yaw=0.0, pitch=0.0, size_deg=(34.0, 20.0))
    run(fr, world, clock, 3.0)
    assert ptz.commanded()[2] < 30


def test_hold_then_search_then_park_within_limits():
    cfg = FramingConfig(hold_s=1.0, search_max_s=6.0)
    clock, ptz, fr = make(cfg)
    world = World(ptz, yaw=5.0, pitch=0.0)
    run(fr, world, clock, 3.0)
    held = ptz.commanded()
    rec = []
    run(fr, world, clock, 0.8, lost=True, record=rec)
    assert all(r[1].state == "holding" for r in rec)
    assert ptz.commanded() == held
    rec = []
    run(fr, world, clock, 5.0, lost=True, record=rec)
    assert rec[-1][1].state == "searching" and rec[-1][1].contract_state == "searching"
    yaws = [r[1].yaw_target for r in rec]
    assert max(yaws) - min(yaws) > 10                              # it sweeps
    lo, hi = cfg.search_yaw_deg
    assert min(yaws) >= lo - 1e-9 and max(yaws) <= hi + 1e-9
    rec = []
    run(fr, world, clock, 12.0, lost=True, record=rec)             # past search_max_s -> parked at home
    assert "parked" in rec[-1][1].notes
    assert abs(ptz.commanded()[0]) <= 0.5
    # hands re-appear -> tracking again
    out = run(fr, world, clock, 3.0)
    assert out.state == "tracking"


def test_hard_limits():
    lim = PTZLimits(yaw_deg=(-20, 20), pitch_deg=(-10, 10), zoom=(0, 30))
    clock, ptz, fr = make(limits=lim)
    world = World(ptz, yaw=30.0, pitch=-17.0, size_deg=(3.0, 3.0))  # visible, but beyond the limits
    rec = []
    run(fr, world, clock, 10.0, record=rec)
    y, p, z = ptz.commanded()
    assert -20 <= y <= 20 and -10 <= p <= 10 and 0 <= z <= 30
    assert y == 20 and p == -10


def test_disabled_framer_does_nothing():
    clock, ptz, fr = make(FramingConfig(enabled=False))
    world = World(ptz, yaw=15.0)
    out = run(fr, world, clock, 2.0)
    assert out.state == "off" and ptz.writes == 0


# ------------------------------------------------------------------- gimbal model
def test_gimbal_model_latency_and_rate():
    dyn = GimbalDynamics(latency_s=0.1, max_rate_deg_s=10.0)
    m = GimbalModel(0, 0, 0, dyn, t0=0.0)
    m.command(yaw=10.0, t=0.0)
    assert m.estimate(0.09)[0] == 0.0                 # still in the latency window
    assert abs(m.estimate(0.6)[0] - 5.0) < 1e-9       # 0.5 s at 10 deg/s
    assert abs(m.estimate(2.0)[0] - 10.0) < 1e-9
    m.command(yaw=-10.0, t=2.0)
    assert abs(m.estimate(2.6)[0] - 5.0) < 1e-9


# ------------------------------------------------------------------- hardware conversion
class FakeV4L2:
    def __init__(self, path):
        from obsbot_ptz.v4l2 import CtrlInfo
        self.values = {"pan_absolute": 0, "tilt_absolute": 0, "zoom_absolute": 0}
        self.controls = {
            "pan_absolute": CtrlInfo(1, "Pan", 1, -468000, 468000, 3600, 0, 0),
            "tilt_absolute": CtrlInfo(2, "Tilt", 1, -324000, 324000, 3600, 0, 0),
            "zoom_absolute": CtrlInfo(3, "Zoom", 1, 0, 100, 1, 0, 0),
        }
        self.log = []

    def has(self, k):
        return k in self.controls

    def get(self, k):
        return self.values[k]

    def set(self, k, v):
        self.values[k] = v
        self.log.append((k, v))
        return v

    def close(self):
        pass


def test_obsbot_ptz_units_signs_limits(monkeypatch):
    monkeypatch.setattr(cam_mod, "V4L2Device", FakeV4L2)
    ptz = cam_mod.ObsbotPTZ("/dev/fake", min_cmd_interval_s=0.0)
    ptz.set_absolute(yaw=10.0, pitch=5.0, zoom=20)
    # Tiny 3: +pan_absolute turns LEFT, so yaw right 10 deg = pan -36000 arcsec; tilt up = +tilt
    assert ptz.dev.values == {"pan_absolute": -36000, "tilt_absolute": 18000, "zoom_absolute": 20}
    assert ptz.readback() == (10.0, 5.0, 20.0)
    ptz.set_absolute(yaw=10.4)                        # quantised to 1 deg -> no new write
    assert len(ptz.dev.log) == 3
    ptz.set_absolute(yaw=500.0, pitch=-500.0, zoom=1000)   # soft limits
    lim = PTZLimits()
    assert ptz.commanded() == (lim.yaw_deg[1], lim.pitch_deg[0], lim.zoom[1])
    ptz.move_relative(dyaw=-5)
    assert ptz.commanded()[0] == lim.yaw_deg[1] - 5
    ptz.center()
    assert ptz.dev.values == {"pan_absolute": 0, "tilt_absolute": 0, "zoom_absolute": 0}


def test_optics_model():
    o = CameraOptics()
    assert abs(o.hfov_deg(1280, 0) - 66.6) < 1.0      # measured ~17 px/deg on the Tiny 3
    assert o.hfov_deg(1280, 50) < o.hfov_deg(1280, 0)
