# GNC Functions Documentation

## int set_speed(float speed__mps)
```
returns 0 for success
```
This function is used to change the speed of the vehicle in guided mode. it takes the speed in meters per second as a float as the input


## int wait4connect()
```
Returns 	0 - connected to fcu
```
Wait for connect is a function that will hold the program until communication with the FCU is established.

## int wait4start()
```
Returns 	0 - mission started
```
Wait for strat will hold the program until the user signals the FCU to enther mode guided. This is typically done from a switch on the safety pilot’s remote or from the ground control station.

## int takeoff(float takeoff_alt)
```
Returns 	0 - nominal takeoff
```
The takeoff function will arm the drone and put the drone in a hover above the initial position.

## int land()
```
Returns 	1 - mode change successful
```
This function changes the mode of the drone to land

## int ros_inumber(ros::NodeHandle controlnode)
```
Returns ros_number
```
This function reads the `number` parameter from the mission .launch file to determine what number the drone is for the formation. Ideally this would go from the namespace but I couldn't find one that worked due to the / in the name. The function requires the program's ros nodehandle as an input

## float spawn_offset(std::string axis, ros::NodeHandle controlnode)
```
Returns d_spawn - 0 if there is no value in .launch, otherwise parameter specified in .launch file
```
This function reads the `x_offset` of `y_offset` parameter values based on first function input and returns a float. This function is a workaround due to ROS/MAVROS assuming the origin is wherever the drone is initially spawned. The launch file will specify the coordinate system while the control .cpp will convert the coordinate system appropriately. The function requires the program's ros nodehandle as an input.

## void pos_print()
```
No return function
```
This function is a lazy debugging solution to assist in checking in math or message errors that affect the position of the drone.

## void vel_print()
```
No return function
```
This function is a lazy debugging solution to assist in checking in math or message errors that affect the velocity of the drone.

## int init_publisher_subscriber(ros::NodeHandle controlnode)
```
Returns 	n/a
```
This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input

