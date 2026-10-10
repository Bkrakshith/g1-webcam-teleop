"""Minimal V4L2 control access through ioctl (no v4l-utils / apt needed).

Only the control ioctls are used (QUERYCAP, QUERYCTRL, G_CTRL, S_CTRL). These do not
touch streaming, so a second file descriptor on the same /dev/videoN can be used for
control while another fd in the same process (OpenCV) owns the capture stream.
"""
from __future__ import annotations

import ctypes
import errno
import fcntl
import os
import threading
from dataclasses import dataclass
from typing import Dict, Optional

# ---- ioctl number helpers (asm-generic/ioctl.h) -------------------------------------
_IOC_WRITE, _IOC_READ = 1, 2


def _IOC(direction: int, typ: str, nr: int, size: int) -> int:
    return (direction << 30) | (size << 16) | (ord(typ) << 8) | nr


def _IOR(typ, nr, size):
    return _IOC(_IOC_READ, typ, nr, size)


def _IOWR(typ, nr, size):
    return _IOC(_IOC_READ | _IOC_WRITE, typ, nr, size)


class v4l2_capability(ctypes.Structure):
    _fields_ = [("driver", ctypes.c_char * 16), ("card", ctypes.c_char * 32),
                ("bus_info", ctypes.c_char * 32), ("version", ctypes.c_uint32),
                ("capabilities", ctypes.c_uint32), ("device_caps", ctypes.c_uint32),
                ("reserved", ctypes.c_uint32 * 3)]


class v4l2_queryctrl(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32), ("type", ctypes.c_uint32), ("name", ctypes.c_char * 32),
                ("minimum", ctypes.c_int32), ("maximum", ctypes.c_int32), ("step", ctypes.c_int32),
                ("default_value", ctypes.c_int32), ("flags", ctypes.c_uint32),
                ("reserved", ctypes.c_uint32 * 2)]


class v4l2_control(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32), ("value", ctypes.c_int32)]


VIDIOC_QUERYCAP = _IOR("V", 0, ctypes.sizeof(v4l2_capability))
VIDIOC_G_CTRL = _IOWR("V", 27, ctypes.sizeof(v4l2_control))
VIDIOC_S_CTRL = _IOWR("V", 28, ctypes.sizeof(v4l2_control))
VIDIOC_QUERYCTRL = _IOWR("V", 36, ctypes.sizeof(v4l2_queryctrl))
V4L2_CTRL_FLAG_NEXT_CTRL = 0x80000000
V4L2_CTRL_FLAG_DISABLED = 0x0001
V4L2_CTRL_FLAG_INACTIVE = 0x0010

# Control ids (linux/v4l2-controls.h)
V4L2_CID_CAMERA_CLASS_BASE = 0x009A0900
CID = {
    "pan_absolute": V4L2_CID_CAMERA_CLASS_BASE + 8,
    "tilt_absolute": V4L2_CID_CAMERA_CLASS_BASE + 9,
    "focus_absolute": V4L2_CID_CAMERA_CLASS_BASE + 10,
    "focus_auto": V4L2_CID_CAMERA_CLASS_BASE + 12,
    "zoom_absolute": V4L2_CID_CAMERA_CLASS_BASE + 13,
    "zoom_continuous": V4L2_CID_CAMERA_CLASS_BASE + 15,
    "pan_speed": V4L2_CID_CAMERA_CLASS_BASE + 32,
    "tilt_speed": V4L2_CID_CAMERA_CLASS_BASE + 33,
    "exposure_auto": V4L2_CID_CAMERA_CLASS_BASE + 1,
}


@dataclass
class CtrlInfo:
    id: int
    name: str
    type: int
    minimum: int
    maximum: int
    step: int
    default: int
    flags: int


class V4L2Device:
    """Thread-safe control-only handle on a V4L2 node."""

    def __init__(self, path: str = "/dev/video0"):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        self._lock = threading.Lock()
        cap = v4l2_capability()
        fcntl.ioctl(self.fd, VIDIOC_QUERYCAP, cap)
        self.card = cap.card.decode(errors="replace")
        self.driver = cap.driver.decode(errors="replace")
        self.bus_info = cap.bus_info.decode(errors="replace")
        self.controls: Dict[str, CtrlInfo] = self._enumerate()

    def _enumerate(self) -> Dict[str, CtrlInfo]:
        out: Dict[str, CtrlInfo] = {}
        q = v4l2_queryctrl()
        q.id = V4L2_CTRL_FLAG_NEXT_CTRL
        while True:
            try:
                fcntl.ioctl(self.fd, VIDIOC_QUERYCTRL, q)
            except OSError:
                break
            name = q.name.decode(errors="replace")
            out_key = next((k for k, v in CID.items() if v == q.id), name)
            out[out_key] = CtrlInfo(q.id, name, q.type, q.minimum, q.maximum, q.step,
                                    q.default_value, q.flags)
            q.id |= V4L2_CTRL_FLAG_NEXT_CTRL
        return out

    def has(self, key: str) -> bool:
        info = self.controls.get(key)
        return info is not None and not (info.flags & V4L2_CTRL_FLAG_DISABLED)

    def get(self, key: str) -> int:
        c = v4l2_control(id=CID.get(key, self.controls[key].id if key in self.controls else 0))
        with self._lock:
            fcntl.ioctl(self.fd, VIDIOC_G_CTRL, c)
        return c.value

    def set(self, key: str, value: int) -> int:
        info = self.controls.get(key)
        if info is not None:
            value = max(info.minimum, min(info.maximum, int(value)))
            if info.step > 1:
                value = info.minimum + round((value - info.minimum) / info.step) * info.step
        c = v4l2_control(id=CID.get(key, info.id if info else 0), value=int(value))
        with self._lock:
            try:
                fcntl.ioctl(self.fd, VIDIOC_S_CTRL, c)
            except OSError as e:
                if e.errno == errno.EBUSY:  # device busy moving: retry once
                    fcntl.ioctl(self.fd, VIDIOC_S_CTRL, c)
                else:
                    raise
        return int(value)

    def describe(self) -> str:
        lines = [f"{self.path}: {self.card} ({self.driver}, {self.bus_info})"]
        for k, i in self.controls.items():
            try:
                val: Optional[int] = self.get(k)
            except OSError:
                val = None
            lines.append(f"  {k:20s} [{i.minimum}, {i.maximum}] step={i.step} default={i.default} "
                         f"flags={i.flags:#x} value={val}")
        return "\n".join(lines)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


def find_device(vendor_product: str = "3564:", name_hint: str = "OBSBOT") -> Optional[str]:
    """Return the /dev/videoN capture node of the first matching camera (by USB id or card name)."""
    base = "/sys/class/video4linux"
    if not os.path.isdir(base):
        return None
    for node in sorted(os.listdir(base), key=lambda s: int("".join(ch for ch in s if ch.isdigit()) or 0)):
        path = f"/dev/{node}"
        try:
            name = open(f"{base}/{node}/name").read().strip()
        except OSError:
            continue
        usb_ok = False
        try:
            dev = os.path.realpath(f"{base}/{node}/device")
            usb = os.path.dirname(dev)
            vid = open(f"{usb}/idVendor").read().strip()
            pid = open(f"{usb}/idProduct").read().strip()
            usb_ok = f"{vid}:{pid}".startswith(vendor_product)
        except OSError:
            pass
        if not (usb_ok or name_hint.lower() in name.lower()):
            continue
        try:
            d = V4L2Device(path)
            is_capture = d.has("pan_absolute") or d.has("zoom_absolute")
            d.close()
        except OSError:
            continue
        if is_capture:
            return path
    return None
