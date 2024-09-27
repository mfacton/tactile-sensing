# ROS2 and UR RTDE control
## ROS2 System
### Nodes
control - Some impedence controllers
pose - Echo TCP and Joints of robot
map - Maze and Map stuff
sense - Reads sensor info from probe
interpret - Interprets sense node readings

### Topics
TODO

## Setup PREEMPT_RT kernel
Not necessarily needed because we have a CB3 robot not an e-Series, so running at 125hz, but if we need better preformance controlling the robot, it is possible with a realtime kernel. I'm not gonna do this because I'm using my personal laptop, but for lab pcs, installation can be found [here](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/real_time.html).

## Network Setup
Connect the UR control box directly to the remote PC with an ethernet cable.

Open the network settings from the UR teach pendant (Setup Robot -> Network) and enter these settings:
```markdown
IP address: 192.168.1.102
Subnet mask: 255.255.255.0
Default gateway: 192.168.1.1
Preferred DNS server: 192.168.1.1
Alternative DNS server: 0.0.0.0
```

Create Wired Connection.
```markdown
IPv4
Manual
Address: 192.168.1.101
Netmask: 255.255.255.0
Gateway: 192.168.1.1
```

Verify connection with PC.
```markdown
ping 192.168.1.101
```

## UR5 RTDE Setup
[UR RTDE GUIDE](https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/)

[UR RTDE C++/Python Interface](https://pypi.org/project/ur-rtde/)

[UR RTDE Interface Docs](https://sdurobotics.gitlab.io/ur_rtde/)
```bash
pip install ur_rtde
```

## Dynamixel Setup
```bash
sudo apt-get install ros-jazzy-dynamixel-sdk
sudo apt-get install ros-jazzy-dynamixel-sdk-examples
```

## Good to know ROS2 commands
### Create package
```bash
cd tactile-sensing/ros2/src

# To create python or c++ package
ros2 pkg create --build-type <ament_python|ament_cmake> <package_name> --dependencies <rclpy|rclcpp>
```
### Build packages
```bash
cd tactile-sensing/ros2
colcon build
```
### Source packages
```bash
# Recommended to source in new terminal
source tactile-sensing/ros2/install/local_setup.bash
```
### Run package
```bash
ros2 run <package_name> <executable_name>
```
### List nodes
```bash
ros2 node list
```
### Get node info
```bash
ros2 node info <node>
```
### List topics
```bash
ros2 topic list
```
### Echo topic
```bash
ros2 topic echo <topic_name>
```

## Other commands
### Create calibration service
```bash
ros2 service call /calibrate soft_msgs/srv/Calibrate "{measurements: 200}"
```

