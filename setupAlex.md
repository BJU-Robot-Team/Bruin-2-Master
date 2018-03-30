#Setup
##Ubuntu
1. Install [VMWare](https://my.vmware.com/en/web/vmware/free#desktop_end_user_computing/vmware_workstation_player/14_0)
2. Download [Ubuntu](https://www.ubuntu.com/download/desktop) and run it with VMWare
##Tools
1. If Ubuntu doesn't come with a suitable browser, download Chrome
2. Download [VS Code](https://code.visualstudio.com/)
* Note that you can bring up a terminal inside VS Code by typing Ctrl+` (that's the character above the left tab key)
3. Install git: from a terminal, type `sudo apt install git`
4. Install other tools (some may already be installed): `sudo apt install g++ gcc python python3 make cmake`
##ROS Tools (from Google Doc)
```bash
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get -y install g++ cmake build-essential git mercurial screen putty
sudo apt-get -y install python3-pip python3-pyqt5 pyqt5-dev-tools
pip3 install screeninfo

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install -y python-rosinstall

sudo apt-get install libsdl2-dev
```
###Important Note
***Edit /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake to remove a trailing space from one of the strings***
##Install the Bruin-2 Repository
```bash
cd ~
git clone https://github.com/BJU-Robot-Team/Bruin-2-Master.git bruin_2_code
git checkout -b alex-work # This creates a branch for you to work on your code seperately and eventually merge it into the main branch
```
Open up the ~/bruin_2_code folder in VS Code and start working on it.
##Start working
###Replacing cout
Replace lines that use cout such as `cout << "Hello, world" << (5+3*4) << "!" << endl;` 
with `ROS_DEBUG_STREAM("Hello, world" << (5+3*4) << "!");` 
or `ROS_ERROR_STREAM("Hello, world" << (5+3*4) << "!");` as appropriate. 
Note that 
* endl is not needed, since ROS... appends its own newline
* You still use the << syntax within the ROS... call. I'm guessing that ROS... is a macro that sends its text to some sort of ostream
###Building and running
To test whether the code works, 
* Build: `catkin_make`
* Test: `./run_bruin2.sh`
###Commit your code
Add the changes to staging by clicking the + icon next to the file name in the git tab of VS Code 
Type a commit message, and press the check mark to commit. 
Push the changes up to GitHub with the rotate icon in the bottom corner of VS Code
