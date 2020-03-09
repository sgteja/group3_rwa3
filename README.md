# Picking part from the conveyer belt.

## Current status --

- The robot is able to pick the static part from the conveyor belt and place it in the AGV.

- To run the following in sequence


Open terminal
```
roslaunch group3_rwa3 group3_rwa3.launch
```
Open another terminal
```
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
```
Open third terminal
```
rosservice call /ariac/start_competition
```
Type when the first gear part is within the FOV of the Logical camera 4
```
rosservice call /ariac/conveyor/control "power: 0.0"
rosrunroup3_rwa3 ariac_example_node
```

TO DO
Approach 1 - stopping the conveyor belt
[] Conveyor belt has to be stopped when gear part is detected by the logical camera 4
[] Record the video

Approach 2 - Not stopping the conveyor belt
[] Predict where the gear part will be after detected by the Logical camera 4
[] Record the video
