# ROS-packages
ros packages


1. Find your idVendor, idProduct and port path

user@localhost:~$ lsusb
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 003: ID 5986:212b Acer, Inc Integrated Camera

2. Create a rule in /etc/udev/rules.d start with a number and specify the idVendor and idProduct, example:

SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="5986", ATTRS{idProduct}=="212b", MODE:="0666"

3. Reload rules

sudo udevadm control --reload

4. Edit /launch/elp.launch file to modify idVendor and iProduct with your id's. If you prefer, you can add a fixed frame or more parameters.  

5. Modify port permissions

sudo chmod o+w /dev/bus/usb/001/003 

6. Enjoy

roslaunch package node








