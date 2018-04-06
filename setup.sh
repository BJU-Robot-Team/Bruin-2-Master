# install some stuff from apt
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y g++ python python3 cmake build-essential git mercurial screen putty python3-pip python3-pyqt5 pyqt5-dev-tools gcc make
pip3 install screeninfo

# install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full python-rosinstall libsdl2-dev
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install openCV
#cd ~ && mkdir bruin_libraries && cd bruin_libraries
#git clone https://github.com/opencv/opencv.git
#cd ~/bruin_libraries/opencv && mkdir release && cd release
#cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
#sudo make && sudo make install

# init the workspace
cd ~/bruin_2_code/src
sudo rm CMakeLists.txt
catkin_init_workspace

# fix a bug with SDL 2
# see https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=822210
echo 'set(SDL2_LIBRARIES "-L${SDL2_LIBDIR}  -lSDL2")' | sudo tee -a /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake
