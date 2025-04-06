#!/bin/bash

# Script to install ROS TurtleSim on Ubuntu

# **Important:** This script assumes you want to install the full ROS desktop installation
# which includes TurtleSim. If you only want the bare minimum for TurtleSim,
# you might be able to install just the turtlesim package, but the desktop install
# is generally recommended for beginners.

# **Note:** Replace 'iron' with your desired ROS 2 distribution name if needed
# (e.g., 'humble', 'rolling'). 'iron' is the latest LTS as of my last knowledge update.
ROS_DISTRO="iron"

echo "Starting ROS installation for ${ROS_DISTRO}..."

# 1. Set up your computer to accept software from packages.ros.org
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

# Add the ROS 2 apt repository
sudo sh -c "echo \"deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros2.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2.list"

# Alternatively, for newer Ubuntu versions (>= 22.04), you might need to create a keyring file:
# sudo mkdir -p /etc/apt/keyrings
# curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros2.gpg
# sudo sh -c "echo \"deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros2.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2.list"

# 2. Update apt index
sudo apt update

# 3. Install the ROS 2 desktop package (includes TurtleSim)
echo "Installing ros-${ROS_DISTRO}-desktop..."
sudo apt install -y ros-${ROS_DISTRO}-desktop

# 4. Source the ROS 2 environment (you'll need to do this in every new terminal)
echo "Setting up ROS environment..."
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Install rosdep (dependency management tool)
sudo apt install -y python3-rosdep
sudo rosdep init
sudo rosdep update

echo "ROS ${ROS_DISTRO} with TurtleSim should now be installed!"
echo "You can run TurtleSim by opening a new terminal and typing: ros2 run turtlesim turtlesim_node"
echo "Then, in another terminal, run the teleoperation node: ros2 run turtlesim turtle_teleop_key"
