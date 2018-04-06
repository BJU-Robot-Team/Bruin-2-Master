sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get -y install g++ cmake build-essential git mercurial screen putty python3-pip python3-pyqt5 pyqt5-dev-tools g++ gcc python python3 make cmake python3-pip
pip3 install screeninfo

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full python-rosinstall libsdl2-dev
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ~ && mkdir bruin_libraries && cd bruin_libraries
git clone https://github.com/opencv/opencv.git
cd ~/bruin_libraries/opencv && mkdir release && cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
sudo make && sudo make install

cd ~/bruin_2_code/src
sudo rm CMakeLists.txt
catkin_init_workspace
