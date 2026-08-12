#!/bin/bash

gnome-terminal -t "cam_wrist" -x bash -c "pixi run ./scripts/d435_wrist.py; exec bash;"&
sleep 5s

gnome-terminal -t "cam_hight" -x bash -c "pixi run ./scripts/launch_d457.py; exec bash;"&
sleep 5s

gnome-terminal -t "rviz2" -x bash -c " pixi run rviz2; exec bash;"

