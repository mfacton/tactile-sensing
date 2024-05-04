# Robot Setup and Usage
## ROS 2 Installation and UR5 Setup on Ubuntu Jammy with ROS 2 Humble
Based off of the official [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Update and Upgrade Packages

```bash
sudo apt update && sudo apt upgrade
```
### Set Locale to UTF-8
```bash
# Check for UTF-8
locale

# Install EN locales
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify settings
locale
```

### Enable Ubuntu Universe Repository
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Add ROS 2 GPG Key and Repository
```bash
sudo apt update && sudo apt upgrade
sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
```

### Install ROS2 Base Packages
```bash
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```

### Setup Environment
```bash
# Add setup.bash to .bashrc so it initializes automatically
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set ROS_DOMAIN_ID to 0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# Set ROS automatic discovery on subnet range
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" >> ~/.bashrc
```
```bash
# Verify environment variables
printenv | grep -i ROS

# ROS_VERSION=2
# ROS_PYTHON_VERSION=3
# ROS_DISTRO=humble
```

### Setup PREEMPT_RT kernel
Not necessarily needed because we have a CB3 robot not an e-Series, but if we need better preformance controlling the robot, it is possible with a realtime kernel. I'm not gonna do this because I'm using my personal laptop, but for lab pcs, installation can be found [here](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/real_time.html).

### UR5 Setup
Based off of the official [UR Installation Guide](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/installation.html)

### Install UR Controller Package
```bash
sudo apt-get install ros-humble-ur

# Verify installed packages
ros2 pkg list | grep -i ur
```

### Network Setup
Connect the UR control box directly to the remote PC with an ethernet cable.

Open the network settings from the UR teach pendant (Setup Robot -> Network) and enter these settings:
```markdown
IP address: 192.168.1.101
Subnet mask: 255.255.255.0
Default gateway: 192.168.1.1
Preferred DNS server: 192.168.1.1
Alternative DNS server: 0.0.0.0
```

Create Wired Connection.
```markdown
IPv4
Manual
Address: 192.168.1.102
Netmask: 255.255.255.0
Gateway: 192.168.1.1
```

Verify connection with PC.
```markdown
ping 192.168.1.101
```

### Setup Robot
[Installing a URCap](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/user_docs/installation/install_urcap_cb3.html#install-urcap-cb3)

### Extract Calibration Data
```bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.1.102 target_filename:="robot_calibration.yaml"
```

### Setup URSim (Optional)
```bash
sudo docker network create --subnet=192.168.56.0/24 ursim_net
```

### Run URSim
```bash
# one
sudo docker run --rm -it -p 5900:5900 -p 6080:6080 --net ursim_net --ip 192.168.56.101 -v ./ursim/urcaps:/urcaps -v ./ursim/programs:/ursim/programs --name ursim universalrobots/ursim_cb3
sudo docker run --rm -it -p 5900:5900 -p 6080:6080 -v ./ursim/urcaps:/urcaps -v ./ursim/programs:/ursim/programs --name ursim universalrobots/ursim_cb3
```

## Usage
### Launch UR Robot Controller and MoveIt
```bash
# Using scaled joint controller
# If using URSim no need for calibration file
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 launch_rviz:=false target_filename:="robot_calibration.yaml"
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

## Good to know ROS2 commands
### Create package
```bash
cd tactile-sensing/ros/src

# To create python or c++ package
ros2 pkg create --build-type <ament_python|ament_cmake> <package_name>
```
### Build packages
```bash
cd tactile-sensing/ros
colcon build
```
### Source packages
```bash
# Recommended to source in new terminal
source tactile-sensing/ros/install/local_setup.bash
```
### Run package
```bash
ros2 run <package_name> <executable_name>
```
### List nodes
```bash
ros2 node list
```
### List topics
```bash
ros2 topic list
```
### Echo topic
```bash
ros2 topic echo <topic_name>
