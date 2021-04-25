# Bebop 2 Autonomous Development Guide
## Introduction

This repository is used to replicate and track the changes made to implement formation flying of the Parrot Bebop 2 within ROS and communicating via MAVlink.

[Development Workflow](docs/development_assist.md)

## Software Installation
[1. Installing Ardupilot and MAVProxy](docs/installing_ardupilot_mavproxy.md)

[2. Installing Gazebo and ArduPilot Plugins](docs/installing_gazebo_ardupilotplugin.md)

[3. Installing ROS and MAVROS](docs/installing_ros.md)

## Modifying ArduPilot for simulation
As multiple instances of the same drone are used for the SITL simulation, duplicates will need to be created which have been detailed in another page:

[Editing ArduPilot Package](docs/installing_ros.md##8.-Editing-ArduPilot-Package)

## Connecting multiple Bebops (In Development)
[Configuring the Bebop 2 (Deprecated)](docs/connect_bebop)

## Moving installation (MAYBE)

To reduce the file size, clear the catkin build.
```
cd <old directory>
catkin clear
```
Return to root folder
```
cd
```
Open `~/.bashrc` for editing:
```
gedit ~/.bashrc
```
Remove the `source ~/<old directory/devel/setup.bash` and `GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/bebop_ws/src/b2_sim/models` from `~/.bashrc`

Rename the workspace folder as desired and rebuild the catkin workspace and global variables.
```
cd <new directory>
catkin build
echo "source ~/<new directory>/devel/setup.bash" >> ~/.bashrc
```
update global variables
```
source ~/.bashrc
```

## Associated Repositories
[b2_sim](https://github.com/minhlam61/b2_sim)
- Repo containing gazebo worlds designed for AruPilot SITL control and quadcopter spawn layouts

[b2_gnc](https://github.com/minhlam61/b2_gnc)
- Repo containing guidance, navigation and control logic including examples mission programs

## Resources 

[Common Linux, ROS and MAVproxy Commands](docs/helpful_commands.md)

[GNC API Documentation](docs/GNC_functions_documentation.md)

## References 
http://ardupilot.org/copter/index.html

http://ardupilot.org/copter/docs/parameters.html#wpnav-parameters


https://discuss.ardupilot.org/

http://ardupilot.org/dev/

https://www.ros.org/