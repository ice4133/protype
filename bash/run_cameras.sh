#!/bin/bash

xterm -title "cam_d405" -e "bash -c 'pixi run ./scripts/d405_high.py; exec bash'" &
sleep 5s

xterm -title "cam_top" -e "bash -c 'pixi run ./scripts/d405_top.py; exec bash'" &
sleep 5s

xterm -title "cam_wrist" -e "bash -c 'pixi run ./scripts/d405_wrist.py; exec bash'" &
sleep 5s

xterm  -title "rviz2" -e "bash -c 'pixi run rviz2 -d ./config/cameras.rviz; exec bash'" &
wait
