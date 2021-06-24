# Workflow Process and useful commands

## Launch Process in single, two or three drones respectively

Run Gazebo sim
```
roslaunch mav_form droneOnly.launch
roslaunch mav_form 2drone.launch
roslaunch mav_form 3drone.launch
```
In a new tab (Ctrl + Shift + T)

Run ArduPilot instances
```
./1sitl.sh
./2sitl.sh
./3sitl.sh
```
In a new tab (Ctrl + Shift + T)
Run the MAVROS instance
```
roslaunch mav_form apm.launch
roslaunch mav_form 2apm.launch
roslaunch mav_form 3apm.launch
```
In a new terminal (Ctrl + Alt + T)

Run the guidance program (example)

```
roslaunch mav_form 1d_test.launch
roslaunch mav_form 2d_fly_pid.launch
roslaunch mav_form 2d_fly_pid.launch
```
---

## Tuning the PID Controller(s)
Add the following line to the final launch file that runs the mission:
```
<node name="rqt_reconfigure" pkg="rqt_reconfigure" type="rqt_reconfigure" />
```
This will allow a live reconfiguration of the Kp, Ki and Kd values. The new values will need to be manually updated in the launch file.

There are various other values inside the source code which are not linked to launch file parameters.
Within drone_lead.cpp:
```
float upper_limit = 1;
float yaw_limit = 0.4;
cmd_twist.twist.linear.z = (wp_in[n].z - d_pose.pose.pose.position.z) * 0.2
move(0.3);
takeoff(3);
```
Within drone_follow.cpp:
```
cmd_twist.twist.angular.z = ::atan2(::sin(yaw_error),::cos(yaw_error)) * 0.6;
float xoff = 2;
takeoff(3);
```

## Bashrc
Open `~/.bashrc` for editing:
```
gedit ~/.bashrc
```

Add these lines to end of `~/.bashrc` (the file open in the text editor):
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Save and close the text editor.

Reload `~/.bashrc`:
```
. ~/.bashrc
```

## Editing control code

When making changes to the workspace, the catkin workspace will need to be updated to show the changes made.

Generally the following commands will need to be run in the Terminal:
```
cd ~/mav_ws
catkin build
source ~/.bashrc
```
You do not need to rebuild the workspace if launch files are changed.

## New control files

The cmake files need to be updated to enable to control code to be run. This is updated in the `CMakeLists.txt` file in `mav_form` and `mav_form` respectively.

Edit or add this at the end of the file:

```
add_executable(<nodename> src/<filename>.cpp)
target_link_libraries(<filename> ${catkin_LIBRARIES})
```
---
## Running MAVROS instance for each drone

A MAVROS instance is required for each drone, as such the following command can be used to control a single drone.

```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
The main process something launch desktop with `./1sitl.sh` contained in the external folder:
```
cd
cp ~/mav_ws/src/mav_form/external/1sitl.sh ~
./1sitl.sh
```
The following commands will also copy the other scripts to the home folder:
```
cp ~/mav_ws/src/mav_form/external/2sitl.sh ~
cp ~/mav_ws/src/mav_form/external/3sitl.sh ~
cp ~/mav_ws/src/mav_form/external/4sitl.sh ~
```
---
## Moving/Renaming installation

To reduce the file size, clear the catkin build.
```
cd <old directory>
catkin clear
```
Open `~/.bashrc` for editing:
```
cd
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
