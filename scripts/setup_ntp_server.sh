#!/usr/bin/env bash
# Make the central laptop the NTP source for the robot LAN.
#
# The AGVs have no RTC and the robot LAN has no route to the internet, so
# systemd-timesyncd on each Pi never reaches ntp.ubuntu.com and the clock stays
# at whatever was restored at boot -- hours away from the laptop and from the
# other vehicle. This laptop sits on both networks, so it can take time from
# the internet over WiFi and serve it to the vehicles over the wired LAN.
#
# Run once on the central laptop:  sudo ./scripts/setup_ntp_server.sh
set -euo pipefail

LAN_SUBNET="${LAN_SUBNET:-192.168.5.0/24}"
LAN_ADDR="${LAN_ADDR:-192.168.5.6}"

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root: sudo $0" >&2
    exit 1
fi

if ! ip -4 -o addr show | grep -q "$LAN_ADDR"; then
    echo "warning: $LAN_ADDR is not on this host; set LAN_ADDR= to override" >&2
fi

echo "== installing chrony =="
apt-get update -qq
apt-get install -y chrony

# timesyncd and chrony both steer the clock; only one may run.
systemctl disable --now systemd-timesyncd 2>/dev/null || true

echo "== configuring chrony to serve $LAN_SUBNET =="
config='/etc/chrony/conf.d/robot-lan.conf'
if ! grep -qE '^\s*confdir\s+/etc/chrony/conf.d' /etc/chrony/chrony.conf; then
    # Older layouts do not include conf.d; append to the main file instead.
    config='/etc/chrony/chrony.conf'
    echo "  (chrony.conf has no confdir; appending directly)"
fi
mkdir -p "$(dirname "$config")"
cat >> "$config" <<EOF

# --- porter robot LAN (added by scripts/setup_ntp_server.sh) ---
# Answer time requests from the vehicles.
allow $LAN_SUBNET

# Keep serving when this laptop's own uplink is down, so the vehicles still
# agree with each other and with this laptop rather than drifting apart.
local stratum 10
EOF

systemctl enable --now chrony
systemctl restart chrony

# ufw is active on this laptop, and NTP is UDP/123. Without this the vehicles'
# requests are dropped and the clocks silently stay wrong.
if command -v ufw >/dev/null && ufw status 2>/dev/null | grep -q '^Status: active'; then
    echo "== opening NTP on $LAN_SUBNET =="
    ufw allow from "$LAN_SUBNET" to any port 123 proto udp comment 'porter NTP'
fi

sleep 3
echo "== status =="
chronyc tracking || true
echo
chronyc clients 2>/dev/null || true
echo
echo "Done. Now point the vehicles at this host:"
echo "    ./scripts/sync_vehicle_clocks.sh"
echo "    ./scripts/check_fleet_clocks.sh"
