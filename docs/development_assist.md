# Workflow Process and useful commands

## Launch Process in single, two or four drones respectively

Run Gazebo sim
```
roslaunch b2_sim droneOnly.launch
roslaunch b2_sim 2drone.launch
roslaunch b2_sim multi_drone.launch
```
In a new tab (Ctrl + Shift + T)

Run ArduPilot instances
```
./startsitl.sh
./multisitl.sh
./multi-ardupilot.sh
```
In a new tab (Ctrl + Shift + T)
Run the MAVROS instance
```
roslaunch b2_sim apm.launch
roslaunch b2_sim 2apm.launch
roslaunch b2_sim multi-apm.launch
```
In a new terminal (Ctrl + Alt + T)

Run the guidance program (example)

```
roslaunch b2_gnc follow.launch
```
### Legacy Control
For all 4 drones to fly in a square
```
roslaunch b2_gnc multi_square_sol.launch
```

## Editing control code

When making changes to the workspace, the catkin workspace will need to be updated to show the changes made.

Generally the following commands will need to be run in the Terminal:
```
cd ~/bebop_form
catkin build
source ~/.bashrc
```

## New control files

The cmake files need to be updated to enable to control code to be run. This is updated in the `CMakeLists.txt` file in `b2_gnc` and `b2_sim` respectively.

Edit or add this at the end of the file:

```
add_executable(<nodename> src/<filename>.cpp)
target_link_libraries(<filename> ${catkin_LIBRARIES})
```

## Running MAVROS instance for each drone

A MAVROS instance is required for each drone, as such the following command can be used to control a single drone.

```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
The main process something launch desktop with `~/multisitl.sh`:
```
cd
cp ~/bebop_ws/src/b2_sim/scripts/multisitl.sh ~
```
OR
```
cd
cp ~/bebop_guide/supp/multisitl.sh ~
```