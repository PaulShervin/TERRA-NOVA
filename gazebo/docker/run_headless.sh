#!/usr/bin/env bash
# Headless run of the whole stack for N simulated-wall-clock seconds, then write the metrics.
#   run_headless.sh <industrial|cave|cave_cutaway> <seconds> <out_dir> [extra launch args, e.g. pose_source:=slam]
set -eo pipefail
WORLD=${1:-cave}; SECONDS_TO_RUN=${2:-300}; OUT=${3:-/tmp/run}; shift 3 || true
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
mkdir -p "$OUT"
ros2 launch mining_sim_gz sim.launch.py world:="$WORLD" headless:=true rviz:=false log_dir:="$OUT" "$@" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
sleep "$SECONDS_TO_RUN"
# ask the segmentation node for a score while everything is still running, then stop (SIGINT makes the SLAM node write its metrics)
ros2 service call /segment_map std_srvs/srv/Trigger "{}" > "$OUT/segment_call.log" 2>&1 || true
ros2 topic echo --once /explore/status > "$OUT/explore_status.txt" 2>&1 || true
kill -INT "$LAUNCH_PID" || true
for i in $(seq 1 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
kill -KILL "$LAUNCH_PID" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
echo "results in $OUT:"; ls -la "$OUT"
