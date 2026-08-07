#!/usr/bin/env bash
# Step each AGV's clock to this laptop's time and make the fix survive reboot.
#
# Run on the central laptop, with the vehicle ROS stacks STOPPED. Jumping the
# clock under a running Nav2 wrecks the TF buffers and action servers, so stop
# the stacks first, run this, then relaunch.
#
#     ./scripts/sync_vehicle_clocks.sh
#     ./scripts/check_fleet_clocks.sh      # verify before relaunching
set -euo pipefail

HOSTS="${HOSTS:-pinky pinky2}"
NTP_SERVER="${NTP_SERVER:-192.168.5.6}"
SYNC_WAIT_SEC="${SYNC_WAIT_SEC:-90}"

for host in $HOSTS; do
    echo "== $host =="

    if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$host" true 2>/dev/null; then
        echo "  unreachable, skipping" >&2
        continue
    fi

    # The bracket keeps the pattern from matching the shell that runs it --
    # a plain "porter_bringup" here matches this very command line.
    if ssh -o BatchMode=yes "$host" 'pgrep -f "porter[_]bringup" >/dev/null'; then
        echo "  ERROR: ROS stack still running. Stop it before syncing." >&2
        exit 1
    fi

    before=$(ssh -o BatchMode=yes "$host" 'date +%s.%N')
    now=$(date +%s.%N)
    echo "  offset before: $(awk -v l="$now" -v r="$before" \
        'BEGIN{printf "%+.1f s", r-l}')"

    # Hand the clock over while timesyncd is out of the way, then hand control
    # back to it now that it has a reachable server on the robot LAN.
    ssh -o BatchMode=yes "$host" "
        set -e
        sudo timedatectl set-ntp false
        sudo date -s @$(date +%s.%N) >/dev/null
        printf '[Time]\nNTP=$NTP_SERVER\nFallbackNTP=ntp.ubuntu.com\n' \
            | sudo tee /etc/systemd/timesyncd.conf >/dev/null
        sudo timedatectl set-ntp true
    "

    # `date -s` only gets within a second or so: the timestamp is taken here
    # but runs on the Pi after the SSH handshake and sudo. timesyncd closes
    # that gap against the laptop's chrony, so wait for it rather than
    # reporting the coarse value and looking like a failure.
    printf '  waiting for NTP sync '
    for _ in $(seq 1 "$SYNC_WAIT_SEC"); do
        synced=$(ssh -o BatchMode=yes "$host" \
            'timedatectl show -p NTPSynchronized --value' 2>/dev/null)
        [ "$synced" = "yes" ] && break
        printf '.'
        sleep 1
    done
    echo " ${synced:-unknown}"

    after=$(ssh -o BatchMode=yes "$host" 'date +%s.%N')
    now=$(date +%s.%N)
    echo "  offset after:  $(awk -v l="$now" -v r="$after" \
        'BEGIN{printf "%+.4f s", r-l}')"
done

echo
echo "Done. Verify with ./scripts/check_fleet_clocks.sh, then relaunch the"
echo "vehicle stacks per README."
