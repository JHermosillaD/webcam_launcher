# :pushpin: webcam_launcher

Install 

```console
user@localhost:~$ sudo apt install libuvc-dev
user@localhost:~$ sudo apt-get install ros-noetic-libuvc-camera
```

> Steps to use your computer's built-in camera in ROS.

1. Find the idVendor, idProduct and port path of your camera using `lsusb`

```console
user@localhost:~$ lsusb
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 003: ID 5986:212b Acer, Inc Integrated Camera
```

where

```
idVendor = 5986
idProduct = 212b
port path = /dev/bus/usb/001/003
```
2. Create a rule in `/etc/udev/rules.d`. Starts with a number and specifies the `idVendor` and `idProduct` using the following template.

```console
user@localhost:~$ sudo nano /etc/udev/rules.d/70-webcam.rules
SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="5986", ATTRS{idProduct}=="212b", MODE:="0666"
```
3. Reload rules

```console
user@localhost:~$ sudo udevadm control --reload
```
4. Edits file `/launch/elp.launch` to include the `idVendor` and `idProduct`. Modify the other parameters according to the dimensions of your camera or add new ones.

```launch
<launch>
  <node pkg="nodelet" type="nodelet" name="nodelet_manager" args="manager" output="screen"/>

  <node pkg="nodelet" type="nodelet" name="libuvc_camera" args="load libuvc_camera/driver /nodelet_manager" output="screen">
    <param name="frame_id" value="elp_camera" />
    
    <!-- Parameters used to find the camera -->
    <param name="vendor" value="0x5986"/>
    <param name="product" value="0x212b"/>

    <!-- Image size and type -->
    <param name="width" value="1280"/>
    <param name="height" value="720"/>
    <param name="video_mode" value="mjpeg"/>
    <param name="frame_rate" value="30"/>

  </node>
</launch>
```

5. Modifies the camera port permissions

```console
user@localhost:~$ sudo chmod o+w /dev/bus/usb/001/003 
```

6. Enjoy!

```console
user@localhost:~/workspace$ catkin_make 
user@localhost:~$ roslaunch webcam_launcher elp.launcher
```
References

- [libuvc_camera](http://wiki.ros.org/libuvc_camera)
- [Sadowski's full explanation](https://msadowski.github.io/ros-web-tutorial-pt2-cameras/)
