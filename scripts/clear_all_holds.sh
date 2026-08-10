#!/usr/bin/env bash
# Release every latch that can keep an AGV from moving.
#
# The holds are independent latches in different nodes, so clearing one is not
# enough -- a vehicle stays stopped while any of them is set:
#
#   collision supervisor  automatic hold on the vehicle it decided must yield
#   safety_hold           the latch that hold lands on, inside cmd_vel_safety_gate
#   emergency_stop        the manual latch, same gate
#   zone locks            dispatcher refuses to dispatch into an owned zone
#   in-flight nav goal    bt_navigator rejects new goals while one is running
#
# Usage:
#   ./scripts/clear_all_holds.sh            # clear holds, leave supervisor on
#   ./scripts/clear_all_holds.sh --cancel-goals
#   ./scripts/clear_all_holds.sh --disable-supervisor   # leaves collisions unguarded
set -uo pipefail

VEHICLES="${VEHICLES:-agv1 agv2}"
ZONE_SERVICES="${ZONE_SERVICES:-clear_b1_lock clear_a_lock clear_park1_lock clear_park2_lock}"
CANCEL_GOALS=false
DISABLE_SUPERVISOR=false

for arg in "$@"; do
    case "$arg" in
        --cancel-goals) CANCEL_GOALS=true ;;
        --disable-supervisor) DISABLE_SUPERVISOR=true ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

call() {  # call <service> <type> <request>
    printf '  %-52s ' "$1"
    if out=$(timeout 10 ros2 service call "$1" "$2" "$3" 2>&1); then
        echo "$out" | grep -q 'success=True' && echo OK || echo "${out##*response:}"
    else
        echo "UNAVAILABLE"
    fi
}

echo "== collision supervisor =="
# Disabling releases whatever it is holding; re-enabling afterwards keeps the
# protection. Only stay disabled when you deliberately want it off.
call /central/fleet/collision_supervisor/enabled std_srvs/srv/SetBool "{data: false}"
if [ "$DISABLE_SUPERVISOR" = false ]; then
    sleep 1
    call /central/fleet/collision_supervisor/enabled std_srvs/srv/SetBool "{data: true}"
else
    echo "  (left DISABLED -- collisions are not guarded)"
fi

echo "== per-vehicle latches =="
for vehicle in $VEHICLES; do
    call "/$vehicle/safety_hold" std_srvs/srv/SetBool "{data: false}"
    call "/$vehicle/emergency_stop" std_srvs/srv/SetBool "{data: false}"
done

echo "== fleet emergency =="
call /central/fleet/emergency_stop std_srvs/srv/SetBool "{data: false}"

# Goals must be cancelled before the zone locks: the dispatcher refuses to
# clear a lock while its owning vehicle is still executing a command, so
# clearing first fails with "owner is still executing a command".
if [ "$CANCEL_GOALS" = true ]; then
    echo "== in-flight navigation goals =="
    # An all-zero goal_id with a zero stamp cancels every goal on the server.
    zeros='{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}'
    for vehicle in $VEHICLES; do
        for action in navigate_to_pose navigate_through_poses park_in_spot; do
            printf '  %-52s ' "/$vehicle/$action"
            timeout 10 ros2 service call \
                "/$vehicle/$action/_action/cancel_goal" \
                action_msgs/srv/CancelGoal "$zeros" >/dev/null 2>&1 \
                && echo OK || echo "UNAVAILABLE"
        done
    done
    # Let the dispatcher observe the cancellations and drop `busy`.
    sleep 1.5
fi

echo "== zone locks =="
for service in $ZONE_SERVICES; do
    call "/central/fleet/$service" std_srvs/srv/Trigger "{}"
done

echo
echo "Verify:  ros2 topic echo /central/fleet/collision_status --once"
