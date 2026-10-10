#!/usr/bin/env bash
# Webcam (OBSBOT Tiny 3) upper-body + hand teleoperation of the G1.
#
#   g1-webcam-teleop/run_webcam_teleop.sh                 # ROS 2 backend (sim): publishes the contract topics
#   g1-webcam-teleop/run_webcam_teleop.sh --dry-run       # camera + tracking + framing + overlay + IK preview only
#   g1-webcam-teleop/run_webcam_teleop.sh --use-sim-time  # stamp messages with /clock (when the sim publishes it)
#
# Any other argument is passed to xr_teleoperate/teleop/teleop_hand_and_arm.py, e.g.
#   --no-window --no-framing --camera /dev/video0 --camera-size 1280x720 --pose-model lite
#   --idle-when-not-manip --ik-urdf upstream
# The mode manager only forwards the arm/hand targets while /g1/mode is MANIP (key 'm').
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

# shellcheck disable=SC1091
source "$ROOT/env.sh"
export OBSBOT_PY_PATH="$HERE/obsbot"

cd "$HERE/xr_teleoperate/teleop"
[ -s webcam/models/hand_landmarker.task ] && [ -s webcam/models/pose_landmarker_full.task ] || ./webcam/fetch_models.sh

ARGS=(--input-source webcam)
case " $* " in
    *" --dry-run "*) ;;
    *" --backend "*) ;;
    *) ARGS+=(--backend ros2) ;;
esac

# logical CPUs 8/9 are defective on this machine
exec taskset -c 0-7,10-31 python teleop_hand_and_arm.py "${ARGS[@]}" "$@"
