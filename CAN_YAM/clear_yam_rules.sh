sudo rm /etc/udev/rules.d/90-can-yam.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
