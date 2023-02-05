# ros2_pid_library
A ROS2 Humble fully configurable PID library based on Brett Beauregard Arduino PID library

The package contains :
 - pid_library : the library itself 
 - use_library : a simple example node that allows you to use the library (the controller)
 - example_system : a first/second order system to apply control to
## How to use the package
You can use the example_sys_launch.py launch file to run the controller and the system to control, publishing the desired set point in another terminal
 - ``ros2 launch example_system example_sys_launch.py``
 - ``ros2 topic pub -r 1  /set_point_topic std_msgs/msg/Float32 "data: 0.0"``
***
!!!FULL DOCUMENTATION WILL BE RELEASED SOON!!!