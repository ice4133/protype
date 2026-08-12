#!/bin/bash
workspace=$(pwd)

source install/setup.bash

# CAN
xterm -title "can" -e "bash -c 'source install/setup.bash; cd CAN_ARX/CAN_ARX; ./CAN_ARX1.sh; exec bash'" &
xterm -title "can" -e "bash -c 'source install/setup.bash; cd CAN_ARX/CAN_ARX; ./CAN_ARX3.sh; exec bash'" &
#gnome-terminal -t "can" -x sudo bash -c "source install/setup.bash; cd CAN_ARX/CAN_ARX; ./CAN_ARX1.sh; exec bash;"
#gnome-terminal -t "can" -x sudo bash -c "source install/setup.bash; cd CAN_ARX/CAN_ARX; ./CAN_ARX3.sh; exec bash;"
wait
