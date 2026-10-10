#!/usr/bin/env python3
"""OBSBOT hardware check: controls, small PTZ moves with image-based verification, return to
centre, capture FPS. Frames stay in memory (nothing is written to disk).

  python g1-webcam-teleop/obsbot/hw_check.py [--device /dev/video0] [--seconds 5]
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obsbot_ptz import ObsbotPTZ  # noqa: E402
from obsbot_ptz.v4l2 import find_device  # noqa: E402


def median_shift(orb, bf, a, b):
    ka, da = orb.detectAndCompute(a, None)
    kb, db = orb.detectAndCompute(b, None)
    if da is None or db is None:
        return None
    m = sorted(bf.match(da, db), key=lambda x: x.distance)[:300]
    if len(m) < 20:
        return None
    v = np.array([np.subtract(kb[x.trainIdx].pt, ka[x.queryIdx].pt) for x in m])
    return np.median(v, axis=0).tolist()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default=None)
    ap.add_argument("--seconds", type=float, default=5.0)
    a = ap.parse_args()
    dev = a.device or find_device()
    report = {"device": dev}
    ptz = ObsbotPTZ(dev, min_cmd_interval_s=0.0)
    report["card"] = ptz.dev.card
    report["controls"] = {k: [c.minimum, c.maximum, c.step] for k, c in ptz.dev.controls.items()
                          if k in ("pan_absolute", "tilt_absolute", "zoom_absolute", "pan_speed", "tilt_speed")}
    report["readback_start (yaw_right, pitch_up, zoom)"] = ptz.readback()
    report["ai_tracking"] = ptz.disable_ai_tracking()[1]

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def frame():
        for _ in range(4):
            cap.read()
        ok, f = cap.read()
        return cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) if ok else None

    orb = cv2.ORB_create(3000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    ptz.center(0)
    time.sleep(2.0)
    base = frame()
    moves = {}
    for name, kw in (("yaw_right+5", dict(dyaw=5)), ("pitch_up+4", dict(dpitch=4))):
        ptz.move_relative(**kw)
        time.sleep(1.5)
        moves[name] = {"readback": ptz.readback(), "image_shift_px(dx,dy)": median_shift(orb, bf, base, frame())}
        ptz.center(0)
        time.sleep(1.5)
    ptz.set_absolute(zoom=15, force=True)
    time.sleep(1.5)
    moves["zoom 15"] = {"readback": ptz.readback()}
    ptz.center(0)
    time.sleep(1.5)
    moves["back_to_centre"] = {"readback": ptz.readback(), "image_shift_px(dx,dy)": median_shift(orb, bf, base, frame())}
    report["moves"] = moves

    n, t0 = 0, time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        ok, _ = cap.read()
        n += ok
    report["capture_fps_1280x720_mjpg"] = round(n / a.seconds, 1)
    cap.release()
    report["readback_end"] = ptz.readback()
    ptz.close()
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
