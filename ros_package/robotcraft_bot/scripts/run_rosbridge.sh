#!/bin/bash

cmd=`ps aux|grep ip_control/app.js|wc -l`

if [ $cmd -le 1 ]; then

nodejs /home/pi/.ip_control/app.js &

fi

source /opt/ros/kinetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
roslaunch robotcraft_bot authentication_server.launch

