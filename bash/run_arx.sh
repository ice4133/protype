#!/bin/bash

workspace=$(pwd)

source install/setup.bash

# ARX_R5
xterm -title "ARX R5" -e "bash -c 'pixi run ros2 launch arx_r5_controller open_double_arm.launch.py; exec bash'" &

xterm -title "arx_cmd_pub" -e "bash -c 'pixi run ros2 run arx_cmd_pub arx_cmd_pub ; exec bash'" &

wait

