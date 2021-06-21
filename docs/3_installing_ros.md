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
mkdir -p ~/mav_ws/src
cd ~/mav_ws
catkin init
```

## 3. Dependencies installation

Install `mavros` and `mavlink` from source:
```
cd ~/mav_ws
wstool init ~/mav_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```
Add a line to end of `~/.bashrc` by running the following command:
```
echo "source ~/mav_ws/devel/setup.bash" >> ~/.bashrc
```

update global variables
```
source ~/.bashrc
```

install geographiclib dependancy 
```
sudo ~/mav_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

## 4. Build instructions
   Inside `mav_ws`, run `catkin build`:

```
cd ~/mav_ws
catkin build
```
update global variables
```
source ~/.bashrc
```

## 5. Verify ROS Plugins for Gazebo are installed
```
sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-plugins
```

