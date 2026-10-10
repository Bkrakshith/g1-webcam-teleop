# g1-webcam-teleop: webcam teleoperation of the G1 arms + BrainCo Revo2 hands

Desk-webcam teleoperation for the hospital G1. `xr_teleoperate/` is our fork of
unitreerobotics/xr_teleoperate, checked out inside this repo. The IK URDF comes from `g1-brainco-description`
(found through `$HG1_ROOT`) and the ROS 2 backend talks to the mode manager in `hospital-sim-ros2`. The master
repo `hospital-sim` checks both out next to this one and has `env.sh`, which sets `HG1_ROOT` and sources ROS 2
Humble, the shared `.venv` and the `ros2_ws` overlay. Commands in this README run from the master checkout;
the folder table is relative to this repo.

No VR headset: the operator sits at a desk facing an **OBSBOT Tiny 3** webcam. MediaPipe tracks
shoulders, elbows, wrists and 21 hand keypoints per hand; the wrists become SE(3) targets for
the upstream xr_teleoperate G1_29 arm IK and the fingers go through the upstream BrainCo
dex-retargeting. The OBSBOT pans / tilts / zooms by itself so the hands and forearms stay in
frame, and the gimbal motion is compensated so auto-framing never moves the robot's arms.

| Folder | What |
|---|---|
| `xr_teleoperate/` | fork of unitreerobotics/xr_teleoperate, remote `upstream`, branch **`hospital-g1-webcam`** |
| `xr_teleoperate/teleop/webcam/` | webcam input (tracker, mapper, televuer-compatible wrapper, tests, tools) |
| `xr_teleoperate/teleop/robot_control/ros2_backend.py` | ROS 2 contract output + dry-run preview backend |
| `obsbot/obsbot-camera-control/` | fork of aaronsb/obsbot-camera-control, remote `upstream`, branch **`hospital-g1`** |
| `obsbot/obsbot_ptz/` | Python PTZ layer (UVC/V4L2) + `auto_framing.py` |
| `obsbot/hw_check.py`, `obsbot/tests/` | hardware check, framing-controller tests |
| `third_party/unitree_sdk2_python/` | unitree_sdk2py (only for the native DDS / real-robot path) |
| `run_webcam_teleop.sh` | launcher (ROS 2 backend by default, `--dry-run` for camera-only) |

## Architecture

```
 OBSBOT Tiny 3 (/dev/video0, 1280x720 MJPG 30 fps)            ONE process owns the camera
        |  frames                          ^ pan/tilt/zoom (UVC V4L2 ioctls, 2nd fd)
        v                                  |
 [capture thread] -- newest frame only --> [tracker thread] (xr_teleoperate/teleop/webcam/tracker.py)
                                            |  MediaPipe Tasks, CPU: PoseLandmarker(full) + HandLandmarker(2)
                                            |  hands <-> arms association (nearest pose wrist / flipped handedness)
                                            |  gimbal estimate at exposure time  (GimbalModel: latency + rate limit)
                                            |  R_base_obs = R_gimbal(yaw,pitch) * R_ray(pixel)   (compensation)
                                            |  ArmHandMapper: body frame -> calibration -> One-Euro -> scale to G1
                                            |     wrist SE(3) (IK frame) + 25 OpenXR keypoints (hand frame)
                                            |     confidence gating, hold -> ease-to-neutral, re-acquire blend
                                            |  AutoFramer(hand + forearm boxes) -> PTZ commands
                                            |  annotated overlay
                                            v
                          WebcamTeleWrapper.get_tele_data()  == televuer TeleData
                                            |
            teleop_hand_and_arm.py main loop (30 Hz, upstream structure)
                |  G1_29_ArmIK (CasADi/IPOPT, pinocchio model of g1-brainco-description/g1_29dof_brainco IK URDF)
                |  hand keypoints -> shared arrays -> BrainCo dex-retargeting (in-process thread)
                v
   --backend ros2 (sim)                          --backend none (--dry-run)        --backend dds (real G1, upstream)
   /g1/teleop/upper_body_target_raw  30 Hz       overlay: IK stick figure +        unitree_sdk2 arm LowCmd +
   /g1/teleop/hand/{l,r}/cmd_raw     50 Hz       hand command bars                 rt/brainco/*/cmd (--ee brainco)
   /g1/teleop/camera/annotated/compressed ~14 Hz
   /g1/teleop/tracking_status        10 Hz
   subscribes /g1/mode, /g1/teleop/key
```

The mode manager decides what reaches the robot: targets are published continuously and only
forwarded in **MANIP** (`m` in the keyboard node, `n` to leave). The overlay shows the mode.

## Install (already done in `.venv`)

System python 3.10 + ROS 2 Humble site-packages, so everything goes into the master repo's `.venv`:

```bash
P=.venv/bin/pip
echo numpy==1.26.4 > /tmp/c.txt                                     # rclpy/rosidl need numpy 1.x
$P install -c /tmp/c.txt torch==2.3.0 --index-url https://download.pytorch.org/whl/cpu   # dex-retargeting needs torch (CPU is enough)
$P install -c /tmp/c.txt casadi mediapipe "opencv-contrib-python==4.10.0.84" meshcat logging-mp sshkeyboard \
    "pytransform3d>=3.5" "nlopt>=2.6.1,<2.8" "trimesh>=4.4" "anytree>=2.12" "pyyaml>=6" "lxml>=5.2.2" pytest
# pinocchio 3.x: the pip 'pin' wheels declare numpy>=2 (via cmeel-boost) but run fine on numpy 1.26
# (NumPy-2-built extensions are 1.x compatible). Download with deps, install without numpy:
$P download pin==3.9.0 -d /tmp/pinwhl && $P install --no-deps $(ls /tmp/pinwhl/*.whl | grep -v numpy)
$P install --no-deps "cmeel-urdfdom>=4,<5" "cmeel-tinyxml2>=10,<11"   # the resolver picks too-new ones
$P install --no-deps -e g1-webcam-teleop/xr_teleoperate/teleop/robot_control/dex-retargeting   # unitree fork (submodule)
# native DDS path only:
CYCLONEDDS_HOME=/opt/ros/humble $P install cyclonedds==0.10.2 && $P install --no-deps -e g1-webcam-teleop/third_party/unitree_sdk2_python
g1-webcam-teleop/xr_teleoperate/teleop/webcam/fetch_models.sh                  # MediaPipe .task models (not in git)
```

Versions in use: numpy 1.26.4, pin (pinocchio) 3.9.0, casadi 3.8.1, mediapipe 1.0.1,
opencv-contrib-python 4.10.0.84, torch 2.3.0+cpu, dex_retargeting 0.4.7 (unitree fork), nlopt 2.7.1,
cyclonedds 0.10.2. Pins that matter on py3.10: **numpy<2** (ROS Humble), **opencv 4.10** (4.11+ need
numpy 2), **nlopt<2.8** (dex-retargeting), **torch 2.3.0** (dex-retargeting pin; CPU wheel).

`pip pin` has no `pinocchio.casadi`, so the fork adds `robot_control/casadi_kinematics.py`, a
symbolic-FK stand-in built from the numeric model (matches pinocchio to 1e-15); the upstream
IK problem (costs, IPOPT options) is unchanged. With a conda pinocchio that has CasADi bindings the
real `cpin` is used automatically.

## Run

```bash
# 1. camera only: tracking + framing + overlay + IK preview; nothing is commanded
g1-webcam-teleop/run_webcam_teleop.sh --dry-run

# 2. simulation (ROS 2 contract topics); start the sim + mode manager as usual, then
g1-webcam-teleop/run_webcam_teleop.sh                 # add --use-sim-time if the stack runs on /clock
#    keyboard node: r (DAMP->STAND), m (STAND->MANIP)  ...  n (leave MANIP)

# 3. real robot, native unitree path (upstream behaviour, BrainCo hands over DDS)
cd g1-webcam-teleop/xr_teleoperate/teleop && python teleop_hand_and_arm.py --input-source webcam --backend dds --arm G1_29 --ee brainco
```

Useful flags (all passed through to `teleop_hand_and_arm.py`): `--no-window` (headless; the UI
shows the annotated stream), `--no-framing`, `--camera /dev/videoN|clip.mp4|image.jpg`,
`--camera-size 1280x720`, `--pose-model lite|full|heavy`, `--ik-urdf auto|upstream|<path>`,
`--idle-when-not-manip` (track at 1/3 rate outside MANIP).
The upstream XR path (`--input-source xr`, televuer) is untouched.

Tools: `python -m webcam.bench --seconds 20` (FPS / latency, from `xr_teleoperate/teleop`),
`python g1-webcam-teleop/obsbot/hw_check.py` (PTZ moves + return to centre + capture FPS),
`python -m webcam.virtual_gimbal_check --mode pose img.jpg` (compensation-model experiment).

## Calibration (do this once per session, and whenever you change chair position)

1. Sit so head, shoulders, elbows and both hands are visible (the camera will frame you).
2. Hold the **neutral pose**: upper arms relaxed along the body, elbows bent ~90°, forearms
   pointing forward, palms facing each other, thumbs up, fingers open. This is the G1's
   zero-arm pose (its "ready" pose), so the robot will not move when you calibrate.
3. Press **`c`** (OpenCV window, or send `c` on `/g1/teleop/key` from the UI / keyboard node).
   Both arms must be visible; the overlay says "calibrated" (or why it failed).

Calibration stores: your shoulder-line yaw/roll (so you need not sit square to the camera),
your arm length (scales your reach to the G1's 0.43 m arm), a position offset (your neutral
wrist -> the robot's neutral wrist, which also absorbs monocular depth bias) and the neutral palm
orientation (wrist rotation is then relative to it). `u` clears it. Without calibration the
mapping uses a 0.55 m arm and the absolute palm frame, which is usable but less accurate.

## Controls

| Key (OpenCV window) | Action |
|---|---|
| `c` | calibrate neutral pose (also accepted on `/g1/teleop/key`) |
| `f` | auto-framing on/off (also on `/g1/teleop/key`) |
| `h` | re-centre the camera (pan/tilt/zoom 0) (also on `/g1/teleop/key`) |
| `u` | clear calibration (also on `/g1/teleop/key`) |
| `r` | start following (upstream key; automatic for `--dry-run` / ROS 2) |
| `q` | quit (camera returns to centre) |

Mode keys (`m`/`n`/`x`...) belong to the keyboard node / mode manager, not to this process.

## Topics (ROS 2 backend)

| Topic | Content |
|---|---|
| `/g1/teleop/upper_body_target_raw` | JointState, 17 names: `waist_{yaw,roll,pitch}_joint` (always 0) + 14 arm joints `left_shoulder_pitch_joint` ... `right_wrist_yaw_joint`; positions in rad, 30 Hz |
| `/g1/teleop/hand/{left,right}/cmd_raw` | JointState, 6 values 0 (open)..1 (closed) in contract order thumb, thumb_aux, index, middle, ring, pinky; names `<side>_thumb_proximal_joint, <side>_thumb_metacarpal_joint, <side>_{index,middle,ring,pinky}_proximal_joint`; 50 Hz |
| `/g1/teleop/camera/annotated/compressed` | CompressedImage jpeg (960x540 overlay: skeleton, hands, framing box, mode, status, IK preview), ~14 Hz |
| `/g1/teleop/tracking_status` | String JSON, 10 Hz: contract keys `hands_visible`, `arms_visible`, `fps`, `framing` (`tracking|searching|off`) plus `framing_detail` (`tracking|holding|searching|off`), `calibrated`, `latency_ms`, `confidence` (per side arm/hand score and state `live|hold|easing|neutral`), `ptz` (`yaw_right_deg`, `pitch_up_deg`, `zoom`), `mode`, `message` |

Hand normalisation uses the Revo2 ranges (thumb_proximal 1.047, thumb_metacarpal 1.553,
fingers 1.41 rad). Upstream's DDS path sends thumb_metacarpal to BrainCo motor 0, which
contradicts BrainCo's motor table (0 = thumb flexion); it is kept as upstream on the native path
and must be checked on the real hand. The ROS topics use the contract order above.

## OBSBOT control: why UVC / V4L2

* The OBSBOT SDK bundled with the fork (libdev v1.0.2) does **not** support the Tiny 3: it finds
  the camera but device init fails (`getDevUuidR ret -3`, see
  `obsbot-camera-control/tools/sdk_probe.cpp`). A pybind11/ctypes shim over it would not work.
* The Tiny 3 exposes the standard UVC camera-terminal controls via uvcvideo:
  `pan_absolute` ±468000 arcsec (1° steps), `tilt_absolute` ±324000, `zoom_absolute` 0..100.
  They are driven with plain `ioctl(VIDIOC_S_CTRL)` from Python (`obsbot_ptz/v4l2.py`, no apt,
  no root) on a second fd while OpenCV streams from the same node in the same process.
* Measured on this unit: +pan turns the camera **left**, +tilt **up** (the layer exposes
  `yaw_right` / `pitch_up`); ~17.4 px/deg at 1280 px (f ≈ 975 px, HFOV ≈ 67°); zoom 30 ≈ 1.85×;
  motion starts ~0.08 s after a command, ~10–12 °/s for large steps; pan repeats to ±1 px,
  tilt returns within ~0.5–1.7° (some hysteresis); capture 29.8 fps at 1280×720 MJPG.
* **Read-back returns the last commanded target, not the live angle** (uvcvideo / device report
  the target instantly). Compensation therefore uses `GimbalModel` (command history + latency +
  rate limit) evaluated at each frame's exposure time; the framer limits itself to 8 °/s so the
  model stays valid.
* **The Tiny 3's own AI tracking cannot be disabled from Linux** (not a UVC control, SDK does not
  talk to the Tiny 3). Turn it off on the camera (hand gesture / OBSBOT Center app) before
  teleoperating. Our absolute commands override it only momentarily; if it is on, the two
  controllers fight and the compensation model is wrong.

## Auto-framing (`obsbot/obsbot_ptz/auto_framing.py`)

Union of the hand boxes and elbow→wrist forearm boxes is kept at the image centre by a PI
controller per axis whose input is the desired absolute angle (estimated current pose + image
error) minus the commanded target, so gimbal lag does not cause overshoot. Deadband 3° to start,
1° to stop (the hardware step is 1°), rate ≤ 8 °/s, accel ≤ 30 °/s². Zoom: out fast (30/s) when
any fingertip is within 5 % of an edge, out (12/s) when the union box exceeds 72 % of the frame,
in slowly (5/s) below 40 %. Hands lost: hold 1 s, then zoom out and sweep yaw ±25° at 6 °/s
(pitch −5°), park at home after 25 s. Soft limits yaw ±60°, pitch −35..+30°, zoom 0..60.

## Gimbal compensation and why the "ray" model

MediaPipe's world landmarks are estimated from a crop around the person/hand. Rotating the
camera changes where the person is in the image but hardly changes the crop's content, so the
3D output stays (approximately) relative to the ray through the crop, not to the optical axis.
Observations are therefore rotated to the fixed base frame with
`R_base_obs = R_gimbal(yaw, pitch) · R_ray(pixel of shoulders / hand centre)`.
`webcam/virtual_gimbal_check.py` re-renders sample images as seen after ±20° pan / ±10° tilt:

| model | pose: wrist−shoulder vector change | hands: palm orientation change |
|---|---|---|
| no compensation | 2.9 cm | 7.7° |
| pan/tilt rotation only (naive) | **10.1 cm** | 11.3° |
| pan/tilt + ray (used) | 3.2 cm | 8.1° |

The naive rotation would itself *move the robot's arms* when the camera moves; the ray model keeps
them within MediaPipe's own noise, and unlike "no compensation" it also accounts for the operator
being off-centre in the image. Remaining limits: ~2–5° roll error for combined pan+tilt (yaw-then-
pitch crop model), and the estimate uses the commanded-angle model, not a measured angle.

## Known limits (monocular)

* **Depth (forward/back) is the weakest axis**: MediaPipe's z is learned, not measured, with a
  few cm of noise and scale bias. It is filtered harder (One-Euro min cutoff 0.4 Hz vs 1 Hz),
  `MapperConfig.depth_gain` can scale it down, and calibration removes the static bias.
  Expect reaching forward/back to be less precise than side/up-down.
* Metric scale comes from MediaPipe's body/hand priors; arm length from calibration rescales it.
* Wrist roll with the palm facing the camera, and fingers occluding each other (fist, pinch seen
  edge-on) are ambiguous; retargeted finger values are noisier than with an XR headset.
* Hands must be visible: a hand lost for >0.5 s holds its target, then eases to neutral (open hand,
  zero-arm pose) over 2 s; on re-acquisition it blends back over 0.4 s.
* Waist/torso and head are not teleoperated (seated operator); waist targets are always 0.
* CPU load: ~27 fps with nobody in view, ~20–25 fps with pose + two hands, 25–55 ms capture→target.

## Tests and verification

```bash
cd g1-webcam-teleop/xr_teleoperate/teleop && python -m pytest -q webcam/tests     # 31 tests
cd g1-webcam-teleop/obsbot && python -m pytest -q tests                            # 12 tests
```

End-to-end with the MuJoCo sim (use your own `ROS_DOMAIN_ID`, e.g. 63, and stop every node after a
test): `ros2 launch g1_bringup sim.launch.py world:=hospital viewer:=false`, then this launcher with
`--camera some_image.jpg --no-window`, keys `r`, `m` on `/g1/teleop/key`. Verified 2026-09-29: the
manager forwards the targets in MANIP, sim arm joints converge to the published IK targets within
0.03–0.08 rad, `/g1/hand/left/state` equals our `cmd_raw` in contract order, `n` returns to STAND.
Keep test poses moderate: big two-arm forward reaches can topple the placeholder policy.

Covered: landmark→wrist math and scaling, reach clamp, 21→25 keypoint mapping, palm/hand frame
conventions, body/base axes, gimbal compensation (static operator under pan/tilt, exact for both
models), ray geometry, calibration (neutral → robot neutral, yaw removal, arm length), loss
handling, One-Euro, contract hand order/names, Revo2 normalisation, open vs curled hand through
the real BrainCo retargeting, CasADi FK shim vs pinocchio, framing controller in closed loop with
the gimbal model (centring without overshoot, rate limit, deadband vs jitter, following, zoom in/
out, hold → search → park, hard limits) and the PTZ unit/sign conversion.
