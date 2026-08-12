#!/bin/bash
# Activate CAN interfaces for 4-arm master-slave teleoperation
# Run with: sudo bash bash/activate_can_master_slave.sh

CAN_INTERFACES=(can_master_l can_master_r can_slave_l can_slave_r)
BITRATE=1000000
TXQUEUELEN=1000

for iface in "${CAN_INTERFACES[@]}"; do
    # Bring down first (ignore error if already down)
    ip link set "$iface" down 2>/dev/null
    # Bring up with bitrate
    ip link set "$iface" up type can bitrate $BITRATE
    # Increase TX queue to avoid "No buffer space available"
    ip link set "$iface" txqueuelen $TXQUEUELEN
    echo "Activated $iface (bitrate=$BITRATE, txqueuelen=$TXQUEUELEN)"
done

echo ""
echo "All CAN interfaces activated. Verify:"
ip -d link show | grep -E "can_(master|slave)" | grep -E "(qlen|state)"
