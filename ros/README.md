installation
##Installation
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
Ubuntu Jammy with ros2 humble

sudo apt update
sudo apt upgrade

locale #check for UTF-8
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

Enable #ubuntu universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

###add ros2 GPG key
sudo apt update
sudo apt upgrade
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools

###setup environment
add setup.bash to ./bashrc so it initializes with new terminals

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

###check environment variables
printenv | grep -i ROS

should print among other these 3 lines
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble

###setup domain id to 0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

###setup ros to discover on subnet range
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" >> ~/.bashrc

#setup PREEMPT_RT kernel
Not necessarily needed, but if we need better preformance controlling the robot
I'm not gonna do this because it's my personal laptop, but for lab pcs, installation can be found here:
https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/real_time.html#


#setup ur_controller
sudo apt-get install ros-humble-ur
Show new packages to make sure they're installed:
ros2 pkg list | grep -i ur

#using basic controller
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.2
