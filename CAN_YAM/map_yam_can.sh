#!/usr/bin/env bash
# map_can_devices.sh
# Usage: sudo ./map_can_devices.sh [--bitrate <bitrate>]
# Guides user to map two CAN devices to can_yam_l and can_yam_r
set -euo pipefail

UDEV_FILE="/etc/udev/rules.d/90-can-yam.rules"
DEFAULT_BITRATE=1000000
BITRATE="$DEFAULT_BITRATE"
IP_CMD="$(command -v ip || echo /sbin/ip)"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --bitrate)
            shift
            BITRATE="${1:-}"
            shift
            ;;
        -h|--help)
            echo "Usage: sudo $0 [--bitrate <bitrate>]"
            exit 0
            ;;
        *)
            echo "Unknown arg: $1"
            exit 1
            ;;
    esac
done

# Validate bitrate
if ! [[ "$BITRATE" =~ ^[0-9]+$ ]]; then
    echo "Invalid bitrate: $BITRATE"
    exit 1
fi

# Check root
if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root. Re-run with sudo."
    exit 1
fi

# Check for ip command
if [[ ! -x "$IP_CMD" ]]; then
    echo "Cannot find ip command ($IP_CMD). Install iproute2."
    exit 1
fi

# Function to list CAN devices
list_cans() {
    ls -1 /sys/class/net 2>/dev/null | grep -E '^can[0-9]+$' || true
}

# Function to get device attributes
get_attrs() {
    local dev="$1"
    local udev_out
    udev_out="$(udevadm info -a "/sys/class/net/$dev" 2>/dev/null || true)"
    
    local serial
    serial="$(echo "$udev_out" | grep -m1 'ATTRS{serial}' | sed -E 's/.*"([^"]+)".*/\1/')"
    
    local vid pid
    vid="$(echo "$udev_out" | grep -m1 'ATTRS{idVendor}' | sed -E 's/.*"([^"]+)".*/\1/')"
    pid="$(echo "$udev_out" | grep -m1 'ATTRS{idProduct}' | sed -E 's/.*"([^"]+)".*/\1/')"
    
    echo "$serial,$vid,$pid"
}

# Function to create udev rule
create_rule() {
    local serial="$1"
    local vid="$2"
    local pid="$3"
    local target="$4"
    
    if [[ -n "$serial" ]]; then
        printf 'SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="%s", NAME="%s"\n' \
            "$serial" "$target"
    elif [[ -n "$vid" && -n "$pid" ]]; then
        printf 'SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s", NAME="%s"\n' \
            "$vid" "$pid" "$target"
    else
        return 1
    fi
}

# Initialize udev rules file
if [[ -f "$UDEV_FILE" ]]; then
    cp -a "$UDEV_FILE" "${UDEV_FILE}.bak.$(date +%s)"
fi
echo "# CAN device mapping rules" > "$UDEV_FILE"

# Map first device
echo "=== Mapping first CAN device ==="
while true; do
    mapfile -t cans < <(list_cans)
    count=${#cans[@]}
    
    if (( count == 0 )); then
        read -r -p "Please plug in the first CAN device and press Enter..."
        continue
    elif (( count > 1 )); then
        echo "Please unplug all CAN devices except the first one."
        read -r -p "Press Enter when ready..."
        continue
    fi
    
    dev="${cans[0]}"
    IFS=',' read -r serial vid pid < <(get_attrs "$dev")
    
    echo "Detected device: $dev"
    if [[ -n "$serial" ]]; then
        echo "Serial: $serial"
    else
        echo "No serial found, using vendor:product ${vid}:${pid}"
    fi
    
    # Get user choice
    while true; do
        read -r -p "Map to can_yam_l or can_yam_r? [l/r]: " choice
        case "${choice,,}" in
            l) target="can_yam_l"; break ;;
            r) target="can_yam_r"; break ;;
            *) echo "Please enter 'l' or 'r'." ;;
        esac
    done
    
    # Create and write rule
    if rule=$(create_rule "$serial" "$vid" "$pid" "$target"); then
        echo "$rule" >> "$UDEV_FILE"
        echo "Mapped $dev to $target"
        break
    else
        echo "Error: Cannot create rule for $dev. Check device attributes."
        exit 1
    fi
done

# Map second device
echo -e "\n=== Mapping second CAN device ==="
echo "Please unplug the first device and plug in the second CAN device."

read -r -p "Press Enter when ready..."

while true; do
    mapfile -t cans < <(list_cans)
    count=${#cans[@]}
    
    if (( count == 0 )); then
        read -r -p "Please plug in the second CAN device and press Enter..."
        continue
    elif (( count > 1 )); then
        echo "Please ensure only the second device is plugged in."
        read -r -p "Press Enter when ready..."
        continue
    fi
    
    dev="${cans[0]}"
    IFS=',' read -r serial vid pid < <(get_attrs "$dev")
    
    echo "Detected device: $dev"
    if [[ -n "$serial" ]]; then
        echo "Serial: $serial"
    else
        echo "No serial found, using vendor:product ${vid}:${pid}"
    fi
    
    # Determine remaining target
    if grep -q "can_yam_l" "$UDEV_FILE"; then
        target="can_yam_r"
    else
        target="can_yam_l"
    fi
    echo "Mapping to $target"
    
    # Create and write rule
    if rule=$(create_rule "$serial" "$vid" "$pid" "$target"); then
        echo "$rule" >> "$UDEV_FILE"
        echo "Mapped $dev to $target"
        break
    else
        echo "Error: Cannot create rule for $dev. Check device attributes."
        exit 1
    fi
done

# Apply rules
echo -e "\nApplying udev rules..."
udevadm control --reload-rules
udevadm trigger --subsystem-match=net --action=add

echo -e "\nDone! Both devices mapped:"
cat "$UDEV_FILE"
echo -e "\nYou can now plug both devices. They will be automatically configured."
