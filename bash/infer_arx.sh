#!/bin/bash

# ARX_R5
xterm  -title "ARX R5 Infer" -e "bash -c 'pixi run ros2 launch arx_r5_controller infer_double_arm.launch.py; exec bash'" &
#xterm  -title "ARX R5 Infer v2" -e "bash -c 'pixi run ros2 launch arx_r5_planner infer_double_arm.launch.py | grep -v ARX方舟无限; exec bash'" &
sleep 3
xterm  -title "Infer Bridge" -e "bash -c 'pixi run ros2 launch inference infer_arx.launch.py; exec bash'" &
wait
