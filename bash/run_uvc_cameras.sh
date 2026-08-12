#!/bin/bash

sudo cp bash/99-realsense-libusb.rules /etc/udev/rules.d
sudo cp bash/99-uvc-cameras.rules /etc/udev/rules.d
sudo udevadm control --reload
sudo udevadm trigger
sudo systemctl restart systemd-udevd.service
sleep 3s
ls /dev | grep video
ls -al /dev/uvc_camera | grep cam
pixi run ./scripts/uvc_cameras.py
