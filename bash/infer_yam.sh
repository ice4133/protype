#!/bin/bash

# ARX_R5
xterm  -title "Yam Infer" -e "bash -c 'pixi run ros2 launch yam_damiao_controller takeover_vr_double_arm.launch.py; exec bash'" &
sleep 3
xterm  -title "Infer Bridge" -e "bash -c 'pixi run ros2 launch inference infer_yam.launch.py; exec bash'" &
wait
