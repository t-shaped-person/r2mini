#!/bin/bash
echo ""
echo "This script create udev rules to /etc/udev/rules.d/"
echo ""

echo "motor rules (serial connection) : /dev/ttyTHS1 to /dev/ttyMCU :"
if [ -f "/etc/udev/rules.d/98-r2mini-mcu.rules" ]; then
    echo "98-r2mini-mcu.rules file already exist."
else 
    echo 'KERNEL=="ttyTHS1", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyMCU"' > /etc/udev/rules.d/98-r2mini-mcu.rules   
    echo '98-r2mini-mcu.rules file is created'
fi

echo ""
echo "ydlidar rules (usb connection) : /dev/ttyUSBx to /dev/ttyLIDAR :"
if [ -f "/etc/udev/rules.d/97-r2mini-lidar.rules" ]; then
    echo "97-r2mini-lidar.rules file already exist."
else 
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLIDAR"' > /etc/udev/rules.d/97-r2mini-lidar.rules
    echo '97-r2mini-lidar.rules file is created'
fi

echo ""
echo "enable uart function"
echo ""
systemctl stop nvgetty
systemctl disable nvgetty

echo ""
echo "reload udev rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
