# rbiz_autorace
RBiz TurtleBot3 Autonomous Driving Challenge Tool packages

- put `rbiz_autorace_msgs` in `arduino_lib_ros_msg` into

```
~/.arduino15/packages/OpenCR/hardware/OpenCR/X.Y.Z/libraries/turtlebot3_ros_lib/
```

- upload each source in the folder `arduino_source` into each OpenCR

- put each in the folder `ros_package` into your ROS catkin workspace, and `catkin_make`:

```
~/catkin_ws/src/
```

How to run rbiz_autorace system

1. roscore

2. Check your /dev/ttyACM number. (i.e traffic_stopwatch : /dev/ttyACM0, level_crossing : /dev/ttyACM1)

3. rosrun rosserial_python serial_node.py __name:=traffic_stopwatch _port:=/dev/ttyACM0 _baud:=115200

4. rosrun rosserial_python serial_node.py __name:=level_crossing _port:=/dev/ttyACM1 _baud:=115200

5. rosrun rbiz_autorace_monitor rbiz_autorace_monitor  

You can download autorace mission 3D file : https://docs.google.com/spreadsheets/d/1pUWFL_SShR0quUDjbueKaUGFv-jw9NO942ab4mymrxU/edit#gid=0

## If you need more information, please contact `kkjong@robotis.com`
