#!/bin/bash

#This script runs the whole mechine using roslaunch

#we need to be sudo so the driver nodes can access /dev and the serial devices
#   that get mounted there
# We need a way to do this without resorting to running as root!
# su

#build the system
#$(catkin_make clean)
#$(catkin_make install)

#we need to export this ROS workspace's environment variables
source ~/bruin_2_code/devel/setup.bash

echo "########################"
echo "Remember when using ros utilities to run the following first:"
echo "   source ~/bruin_2_code/devel/setup.bash"
echo "########################"

#Now we can run everything using roslaunch and the launch.xml config file
roslaunch launch.xml
