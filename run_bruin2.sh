#!/bin/bash

#This script runs the whole mechine using roslaunch

#build the system
#$(catkin_make clean)
#$(catkin_make install)

#we need to export this ROS workspace's environment variables
source ~/bruin_2_code/devel/setup.bash

echo "########################"
echo "Remember when using ros utilities to run the following first:"
echo "   source ~/bruin_2_code/devel/setup.bash"
echo "########################"

#We need to copy the data directory to the cwd that all the nodes work out of so they
# can access the files
rm -rf ~/.ros/data && cp -r data ~/.ros

#Now we can run everything using roslaunch and the launch.xml config file
roslaunch launch.xml

