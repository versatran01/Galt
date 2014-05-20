#!bin/bash
# source ros and add Galt to package path
source /opt/ros/indigo/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)

# colors
NO='\e[0m'
RED='\033[01;31m'
GREEN='\033[01;32m'
YELLOW='\033[01;33m'
BLUE='\033[01;34m'
PURPLE='\033[01;35m'

# display env variables
echo -e "$YELLOW-- ROS_PACKAGE_PATH$NO\t$GREEN$ROS_PACKAGE_PATH $NO"
