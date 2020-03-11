# Picking part from the conveyer belt.

## Running the package:

	- Unpack the zip file "group3_rwa3.zip" into "group3_rwa3" folder into the workspace src directory.
	- Open two terminals.
	- In terminal-1:
	```
	source /opt/ros/melodic/setup.bash
	source ~/catkin_ws/devel/setup.bash (Hoping the folder is in home directory)
	roslaunch group3_rwa3 group3_rwa3.launch
	```
	- In terminal-2:
	```
	source /opt/ros/melodic/setup.bash
	source ~/ariac_ws/install/setup.bash (The ariac workspace where moveit package is present)
	roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
	```
	- In terminal-3:
	```
	source /opt/ros/melodic/setup.bash
	catkin_make --only-pkg-with-deps group3_rwa3
	source ~/catkin_ws/devel/setup.bash (Hoping the folder is in home directory)
	rosrun group3_rwa3 ariac_example_node
	```

	
