#!/usr/bin/env bash
# Record cmd_vel to a bag so message ARRIVAL times can be checked afterwards.
#
# Run this in THREE places at the same time, during the same run:
#
#   on the base station:   ./record_conn.sh robot_1 base
#                          ./record_conn.sh robot_2 base
#   ssh'd into robot 1:    ./record_conn.sh robot_1 r1
#   ssh'd into robot 2:    ./record_conn.sh robot_2 r2
#
# The base bag is what was SENT. The robot bags are what ARRIVED. Comparing
# message counts and gap timings between them is what tells you whether the
# link dropped, and when.
#
# Ctrl-C to stop. Then analyse with:
#   python3 bag_gaps.py conn_base_* conn_r1_* conn_r2_*

set -eu

ROBOT=${1:-robot_1}
TAG=${2:-$(hostname)}
OUT="conn_${TAG}_${ROBOT}_$(date +%Y%m%d_%H%M%S)"

echo "recording /${ROBOT}/cmd_vel  ->  ${OUT}"
echo "(Ctrl-C to stop)"

# --use-sim-time is deliberately NOT set: we want wall-clock arrival times.
ros2 bag record -o "$OUT" "/${ROBOT}/cmd_vel"
