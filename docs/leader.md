# Leader Drone Functions Documentation
Something about how it operates

---

## Subscribed Topics

### /mavros/global_position/local
[geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)

[nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)
Above is in use

### /mavros/global_position/gp_vel
[geometry_msgs/TwistStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistStamped.html)

### /mavros/state
[mavros_msgs/State](http://docs.ros.org/en/api/mavros_msgs/html/msg/State.html)

### /gnc/pos_error
[geometry_msgs/Point](http://docs.ros.org/en/api/geometry_msgs/html/msg/Point.html)


## Published Topics

### /mavros/setpoint_position/local (not in use)
[geometry_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)

### /mavros/setpoint_velocity/cmd_vel
[geometry_msgs/TwistStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistStamped.html)


## Service Clients

### /mavros/set_mode
[mavros_msgs/SetMode](http://docs.ros.org/en/api/mavros_msgs/html/srv/SetMode.html)

### /mavros/cmd/arming
[mavros_msgs/CommandBool](http://docs.ros.org/en/api/mavros_msgs/html/srv/CommandBool.html)

### /mavros/cmd/takeoff
[mavros_msgs/CommandTOL](http://docs.ros.org/en/api/mavros_msgs/html/srv/CommandTOL.html)

### /mavros/cmd/land
[mavros_msgs/CommandTOL](http://docs.ros.org/en/api/mavros_msgs/html/srv/CommandTOL.html)

### /mavros/cmd/command
[mavros_msgs/CommandLong](http://docs.ros.org/en/api/mavros_msgs/html/srv/CommandLong.html)