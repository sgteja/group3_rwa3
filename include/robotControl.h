
int robotControl() {
    //--Work with MoveIt
    // ros::init(argc, argv, "robot_control_node");
    ros::NodeHandle nodeRobo("/ariac/arm1");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface::Options loadOptions("manipulator","/ariac/arm1/robot_description",nodeRobo);
    moveit::planning_interface::MoveGroupInterface move_group(loadOptions);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("Lecture5", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("Lecture5", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = -0.0167545;
    target_pose1.orientation.y = 0.027048;
    target_pose1.orientation.z = 0.7265685;
    target_pose1.orientation.w = 0.686357;
    // target_pose1.position.x = -0.200012;
    target_pose1.position.x = 1.21;
    // target_pose1.position.y = 0.583010;
    target_pose1.position.y = 0.816737;
    // target_pose1.position.z = 0.724102;
    target_pose1.position.z = 0.95;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();
    ros::Duration(1.0).sleep();
    // gripperToggle(true);

    ros::shutdown();

    return 0;
}
