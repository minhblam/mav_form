# 3. Install ROS and Setup Catkin  

## 1. Install ROS

   - Do _Desktop-full Install_
   - Follow until _Step 1.7_ at the end of the page

First, install **ROS Melodic** using the following instructions: http://wiki.ros.org/melodic/Installation/Ubuntu


## 2. Set Up Catkin workspace

We use `catkin build` instead of `catkin_make`. Please install the following:
```
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

Then, initialize the catkin workspace:
```
mkdir -p ~/bebop_ws/src
cd ~/bebop_ws
catkin init
```

## 3. Dependencies installation

Install `mavros` and `mavlink` from source:
```
cd ~/bebop_ws
wstool init ~/bebop_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```
Add a line to end of `~/.bashrc` by running the following command:
```
echo "source ~/bebop_ws/devel/setup.bash" >> ~/.bashrc
```

update global variables
```
source ~/.bashrc
```

install geographiclib dependancy 
```
sudo ~/bebop_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

## 4. Clone Simulation ROS package 

```
cd ~/bebop_ws/src
git clone https://github.com/minhlam61/b2_sim.git
```
Repository should now be copied to `~/bebop_ws/src/b2_sim/` (don't run this line. This is just saying that if you browse in the file manager, you will see those folders).

Run the following to tell gazebo where to look for the models 
```
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/bebop_ws/src/b2_sim/models" >> ~/.bashrc
```

## 5. Clone GNC ROS package 

This ROS package comes with a GNC API that will make scripting the drone easy.

```
git clone https://github.com/minhlam61/b2_gnc.git
```

## 6. Build instructions
   Inside `bebop_ws`, run `catkin build`:

```
cd ~/bebop_ws
catkin build
```
update global variables
```
source ~/.bashrc
```

## 7. Verify ROS Plugins for Gazebo are installed
```
sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-plugins
```

## 8. Editing ArduPilot Package

To support multi-drone presence, extra modifications are needed to the ArduPilot package.

### Easy Process TEST 

```
cp bebop_form/src/b2_sim/scripts/ardu-parms/gazebo-drone1.parm ardupilot/Tools/autotest/default_params/
cp bebop_form/src/b2_sim/scripts/ardu-parms/gazebo-drone2.parm ardupilot/Tools/autotest/default_params/
cp bebop_form/src/b2_sim/scripts/ardu-parms/gazebo-drone3.parm ardupilot/Tools/autotest/default_params/
cp bebop_form/src/b2_sim/scripts/ardu-parms/gazebo-drone4.parm ardupilot/Tools/autotest/default_params/
```

### Manual Process

In  `ardupilot/Tools/autotest/default_params`, create 2 or 4 `.parm` files under the name `gazebo-drone1` and onwards with the following text inside:

```
# Iris is X frame
FRAME_CLASS 1
FRAME_TYPE  1
# IRLOCK FEATURE
RC8_OPTION 39
PLND_ENABLED    1
PLND_TYPE       3
# SONAR FOR IRLOCK
SIM_SONAR_SCALE 10
RNGFND1_TYPE 1
RNGFND1_SCALING 10
RNGFND1_PIN 0
RNGFND1_MAX_CM 5000
SYSID_THISMAV 1
```
Each file with their corresponding `SYSID_THISMAV` parameter value incrementing accordingly

With these additions, the `vehicleinfo.py` file needs to be updated and is found in `ardupilot/Tools/autotest/pysim`
```
gedit ardupilot/Tools/autotest/pysim/vehicleinfo.py
```

Under the ArduCopter "frames" block under `#SIM` under "IrisRos", add the following lines as needed:

```
            "gazebo-drone1": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-drone1.parm"],
            },
            "gazebo-drone2": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-drone2.parm"],
            },
            "gazebo-drone3": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-drone3.parm"],
            },
            "gazebo-drone4": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-drone4.parm"],
            },
```

These files can also be found in the `supp/` folder.