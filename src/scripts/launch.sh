#!/bin/sh
cd /home/workspace/catkin_ws
source devel/setup.bash
xterm -e "gazebo" &
sleep 5
xterm -e "source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5
xterm -e "rosrun rviz rviz"
