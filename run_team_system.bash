#!/usr/bin/env bash
. /opt/ros/${ROS_DISTRO}/setup.bash
. ~/rubot/install/setup.bash
cd ~/rubot
source devel/setup.bash
sleep 10s
rosrun ariac_example ariac_example_node
