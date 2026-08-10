#!/usr/bin/env bash
# Pre-flight check: are the laptop and both AGVs close enough in time for TF?
#
# Nav2 here runs with transform_tolerance between 0.1 s and 1.0 s, and RViz
# shares one TF buffer across both vehicles under a common `map` parent. Once
# the vehicles disagree by more than that tolerance, tf2 resolves "latest" to
# whichever vehicle is ahead and the other one drops out with
# "extrapolation into the past" -- the RobotModel flicker.
#
# Exit 0 = fine, 1 = skew too large to navigate, 2 = a host was unreachable.
set -uo pipefail

HOSTS="${HOSTS:-pinky pinky2}"
WARN_SEC="${WARN_SEC:-0.05}"
FAIL_SEC="${FAIL_SEC:-0.20}"

status=0
declare -A offset

for host in $HOSTS; do
    remote=$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$host" 'date +%s.%N' \
        2>/dev/null)
    local_now=$(date +%s.%N)
    if [ -z "$remote" ]; then
        printf '%-8s UNREACHABLE\n' "$host"
        status=2
        continue
    fi

    delta=$(awk -v l="$local_now" -v r="$remote" 'BEGIN{printf "%.4f", r-l}')
    offset[$host]=$delta
    magnitude=${delta#-}

    verdict=OK
    if awk -v d="$magnitude" -v f="$FAIL_SEC" 'BEGIN{exit !(d>f)}'; then
        verdict=FAIL
        status=1
    elif awk -v d="$magnitude" -v w="$WARN_SEC" 'BEGIN{exit !(d>w)}'; then
        verdict=WARN
    fi

    synced=$(ssh -o BatchMode=yes "$host" \
        'timedatectl show -p NTPSynchronized --value' 2>/dev/null)
    printf '%-8s %-5s offset vs laptop: %+9.4f s   ntp_synced=%s\n' \
        "$host" "$verdict" "$delta" "${synced:-unknown}"
done

# The vehicle-to-vehicle gap is what actually breaks cross-vehicle TF, so
# report it separately from each vehicle's offset against the laptop.
set -- $HOSTS
if [ $# -eq 2 ] && [ -n "${offset[$1]:-}" ] && [ -n "${offset[$2]:-}" ]; then
    pair=$(awk -v a="${offset[$1]}" -v b="${offset[$2]}" \
        'BEGIN{d=a-b; printf "%.4f", d<0?-d:d}')
    printf '\n%-8s %s <-> %s gap: %.4f s\n' 'PAIR' "$1" "$2" "$pair"
    if awk -v d="$pair" -v f="$FAIL_SEC" 'BEGIN{exit !(d>f)}'; then
        echo "FAIL: vehicles disagree by more than ${FAIL_SEC}s;" \
             "cross-vehicle TF lookups will fail."
        status=1
    fi
fi

if [ $status -eq 1 ]; then
    echo
    echo "Fix with: ./scripts/sync_vehicle_clocks.sh (stacks stopped first)"
fi
exit $status
