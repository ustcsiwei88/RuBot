#!/usr/bin/env bash
. /opt/ros/${ROS_DISTRO}/setup.bash
cd ~
git clone https://github.com/ustcsiwei88/rubot.git
cd rubot
git submodule update --init
catkin_make install