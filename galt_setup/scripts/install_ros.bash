#!/usr/bin/env bash

# This file will install ros and all dependencies we need from it

# Stop if command ends in an error
set -e

# Check to see if we are running with root privileges
if [[ $(id -u) -ne 0 ]] ; then
    echo "Please run this script as root (eg using sudo)"
    exit 1
fi

# Finish if ros is already installed
if [[ -f "/opt/ros/indigo/setup.bash" ]] ; then
    echo "ROS already installed."
else
    # Setup your sources.list
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

    # Set up your keys
    wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -

    # Installation
    apt-get update
    apt-get install ros-indigo-desktop-full

    # Initialize rosdep
    rosdep init
    rosdep update

    # Environment setup
    if [[ $(grep -Fxq "/opt/ros/indigo/setup.bash" ~/.bash_local) ]]
    then
        echo "Environment already set up"
    else
        echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
        source ~/.bashrc
    fi

    # Getting rosinstall
    apt-get install python-rosinstall
fi

# Install other ros pkgs we need
echo "Install other ros packages"
PACAKGES=(
ros-indigo-octomap
ros-indigo-octomap-ros
ros-indigo-octomap-msgs
ros-indigo-octovis
ros-indigo-octomap-rviz-plugins
ros-indigo-urg-node
)
apt-get -y --force-yes install ${PACKAGES[@]}
