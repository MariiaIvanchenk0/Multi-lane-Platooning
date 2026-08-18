#!/usr/bin/env bash
# Emergency stop: publish zero cmd_vel to every robot until Ctrl-C.
#
#   ./stop.sh                  -> robot_1 robot_2
#   ./stop.sh robot_1          -> just robot_1
#   ./stop.sh robot_1 robot_2 robot_3
#
# NOTE ON ORDERING
#   The controller chain is  controller -> raw_cmd_vel -> safety net -> cmd_vel.
#   If the platoon launch is still running it keeps publishing cmd_vel too, and
#   the robot sees whichever message arrived last -- so this script alone only
#   *mostly* stops them. For a guaranteed stop, Ctrl-C the platoon launch FIRST,
#   then run this to flush zeros (a robot holds its last command until a new one
#   arrives, so without the zeros it keeps driving).

set -u

ROBOTS=("$@")
if [ ${#ROBOTS[@]} -eq 0 ]; then
    ROBOTS=(robot_1 robot_2 robot_4 robot_5)
fi

RATE=20
PIDS=()

cleanup() {
    echo
    echo "stopping publishers..."
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "done. robots are holding the last zero command."
    exit 0
}
trap cleanup INT TERM

echo "publishing zero Twist at ${RATE} Hz to:"
for r in "${ROBOTS[@]}"; do
    echo "   /${r}/cmd_vel"
    ros2 topic pub -r "$RATE" "/${r}/cmd_vel" geometry_msgs/msg/Twist "{}" \
        > /dev/null 2>&1 &
    PIDS+=($!)
done

echo
echo "Ctrl-C to stop publishing."
wait
