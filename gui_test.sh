# we need to export this ROS workspace's environment variables
source ./devel/setup.bash

# We need to copy the data directory to the cwd that all the nodes work out of so they
# can access the files
rm -rf ~/.ros/data && cp -r data ~/.ros

# Now we can run everything using roslaunch and the launch.xml config file
roslaunch gui_test.xml