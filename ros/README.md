## ROS 2 Installation and UR5 Setup on Ubuntu Jammy with ROS 2 Humble
Based off of the official [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Update and Upgrade Packages

```bash
sudo apt update
sudo apt upgrade
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
sudo apt update
sudo apt upgrade
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
```

### Setup Environment
```bash
# Add setup.bash to .bashrc so it initializes automatically
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set ROS_DOMAIN_ID to 0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# Set ROS automatic discovery on subnet range
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" >> ~/.bashrc

# Verify environment variables
printenv | grep -i ROS
# Should print
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
ROS_DOMAIN_ID=0
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
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

### Usage: Launch UR Robot Controller
```bash
# Using basic controller
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.2
```
