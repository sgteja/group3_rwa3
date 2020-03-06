# Picking part from the conveyer belt.

## Current status --
	
- The robot is able to pick the static part from the conveyor belt and place it in the AGV.

- To run the following in sequence

```
roslaunch group3_rwa3 group3_rwa3.launch

roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1

rosservice call /ariac/start_competition
rosservice call /ariac/conveyor/control "power: 0.0"
rosrun gazebo_ros spawn_model -sdf -x 1.21 -y 0.816 -z 0.9477 -R -0.06 -P 0.0128 -Y 1.628 -file `catkin_find osrf_gear --share --first-only`/models/piston_rod_part_ariac/model.sdf -reference_frame world -model piston_rod_part_8

rosrunroup3_rwa3 ariac_example_node
```
