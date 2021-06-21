# 4. Installing main packages 


## 1. Clone Simulation Control Package

```
cd ~/mav_ws/src
git clone https://github.com/minhlam61/mav_form.git
```
Repository should now be copied to `~/mav_ws/src/mav_form/` (don't run this line. This is just saying that if you browse in the file manager, you will see those folders).

Run the following to tell gazebo where to look for the models 
```
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/mav_ws/src/mav_form/models" >> ~/.bashrc
```

## 2. Clone PID Control Package

```
cd ~/mav_ws/src
git clone https://bitbucket.org/AndyZe/pid.git
```
For information on how this package works, refer to [pid ROS wiki](http://wiki.ros.org/pid).


## 3. Build instructions
   Inside `mav_ws`, run `catkin build`:

```
cd ~/mav_ws
catkin build
```
update global variables
```
source ~/.bashrc
```

## 4. Editing ArduPilot Package

To support multi-drone presence, extra modifications are needed to the ArduPilot package.

The `vehicleinfo.py` file needs to be updated and is found in `ardupilot/Tools/autotest/pysim`
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

These files can also be found in the `external/` folder.

### Updating .parm files - Easy Process

```
cp mav_form/external/ardu-parms/gazebo-drone1.parm ardupilot/Tools/autotest/default_params/
cp mav_form/external/ardu-parms/gazebo-drone2.parm ardupilot/Tools/autotest/default_params/
cp mav_form/external/ardu-parms/gazebo-drone3.parm ardupilot/Tools/autotest/default_params/
cp mav_form/external/ardu-parms/gazebo-drone4.parm ardupilot/Tools/autotest/default_params/
```

### Updating .parm files - Manual Process

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