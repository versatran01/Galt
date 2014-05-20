#!bin/bash
source setup.bash
# set master uri to nuc
export ROS_MASTER_URI=http://192.168.129.248:11311

# display env variables
echo -e "$YELLOW-- ROS_MASTER_URI$NO\t$GREEN$ROS_MASTER_URI $NO"
