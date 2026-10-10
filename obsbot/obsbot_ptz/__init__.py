"""OBSBOT Tiny 3 PTZ control (UVC/V4L2) and hand-following auto-framing."""
from .camera import (CameraOptics, GimbalDynamics, GimbalModel, ObsbotPTZ, PTZLimits, PTZState,
                     SimulatedPTZ)
from .auto_framing import AutoFramer, FramingConfig, FramingOutput, points_box, union_box

__all__ = ["ObsbotPTZ", "SimulatedPTZ", "PTZLimits", "PTZState", "GimbalDynamics", "GimbalModel",
           "CameraOptics", "AutoFramer", "FramingConfig", "FramingOutput", "union_box", "points_box",
           "open_ptz"]


def open_ptz(device=None, allow_simulated=True, **kw):
    """Open the real OBSBOT if its PTZ controls are reachable, else a SimulatedPTZ."""
    try:
        return ObsbotPTZ(device, **kw)
    except Exception as e:  # noqa: BLE001
        if not allow_simulated:
            raise
        sim = SimulatedPTZ()
        sim.open_error = str(e)
        return sim
