sudo cp 99-realsense-libusb.rules /etc/udev/rules.d
sudo udevadm control --reload
sudo udevadm trigger
sudo systemctl restart systemd-udevd.service
