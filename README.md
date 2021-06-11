# PID-Based Formation Control
## Introduction

This repository was used to replicate and track the changes made to implement formation flying of the Parrot Bebop 2 within ROS and communicating via MAVlink.

This project was originally built around compatibility around the Parrot Bebop 2 and the [bebop_autonomy](https://github.com/AutonomyLab/bebop_autonomy) package. Due to incompatibility, this functionality was cancelled and the project was rescoped to be completely within simulation.

[Leader Drone Functions Documentation](docs/leader.md)

[Follower Drone Functions Documentation](docs/follower.md)

[Navigation Manager Functions Documentation](docs/nav_manager.md)

Simulation workspace foundation was based from [Intelligent-Quads](https://github.com/Intelligent-Quads) on GitHub from packages [iq_sim](https://github.com/Intelligent-Quads/iq_sim) and [iq_gnc](https://github.com/Intelligent-Quads/iq_gnc)

---

## Software Installation
[1. Installing Ardupilot and MAVProxy](docs/installing_ardupilot_mavproxy.md)

[2. Installing Gazebo and ArduPilot Plugins](docs/installing_gazebo_ardupilotplugin.md)

[3. Installing ROS and MAVROS](docs/installing_ros.md)

## Modifying ArduPilot for simulation
As multiple instances of the same drone are used for the SITL simulation, duplicates will need to be created which have been detailed in another page:

[Editing ArduPilot Package](docs/installing_ros.md##7.-Editing-ArduPilot-Package)

---

## Connecting multiple Bebops (Deprecated)
[Configuring the Bebop 2](docs/connect_bebop)

## Resources 

[Common Linux, ROS and MAVproxy Commands](docs/helpful_commands.md)

[MAVROS ROS Documentation](http://wiki.ros.org/mavros)

[GNC API Documentation](docs/GNC_functions_documentation.md)

## References 
http://ardupilot.org/copter/index.html

http://ardupilot.org/copter/docs/parameters.html#wpnav-parameters

https://discuss.ardupilot.org/

http://ardupilot.org/dev/

https://www.ros.org/