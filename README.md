# Bruin-2-Master
The master repository for the Bob Jones University Bruin 2 autonomous vehicle 

# Getting started
## Prerequisites
1. [VMWare](https://my.vmware.com/en/web/vmware/free#desktop_end_user_computing/vmware_workstation_player/14_0) or some other hypervisor, or if running natively, no hypervisor is needed
2. [Ubuntu](https://www.ubuntu.com/download/desktop) or some other Debian-based Linux
3. [VS Code](https://code.visualstudio.com/) or some other code editor
4. git: `sudo apt install git`

## Setup
- Clone the repo: `cd ~ && git clone https://github.com/BJU-Robot-Team/Bruin-2-Master.git bruin_2_code`
- Run `source ~/bruin_2_code/setup.sh` to install the dependencies (This may take up to an hour)
- Run `catkin_make` from ~/bruin_2_code to build the code
- Run `./run_bruin2.sh` from ~/bruin_2_code to run the code

# Architecture
The Bruin-2 Robot is built on a golf cart frame with several important electronic pieces that help control it
## Electronic components
### Sensors
- GPS: connected over serial at 4800 baud
- Compass: connected over serial at 19200 baud
- Camera: connected over USB
- LIDAR: connected over USB. Not currently connected to anything

### Actuators
- [Relay Board](https://docs.numato.com/doc/16-channel-usb-relay-module/): connected over USB, with a serial-like interface. Also has GPIO pins

