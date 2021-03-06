//
// Created by zeid on 2/27/20.
//
#include "sensor.h"

AriacSensorManager::AriacSensorManager():
// camera1_part_list{},
// camera2_part_list{},
// camera3_part_list{},
camera4_part_list{}{
    ROS_INFO_STREAM(">>>>> Subscribing to logical sensors");
    camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10,
                                                &AriacSensorManager::LogicalCamera1Callback, this);
    camera_2_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_2", 10,
                                                &AriacSensorManager::LogicalCamera2Callback, this);
    camera_3_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_3", 10,
                                                &AriacSensorManager::LogicalCamera3Callback, this);
    camera_4_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_4", 10,
                                                &AriacSensorManager::LogicalCamera4Callback, this);
    break_beam_1_subscriber_ = sensor_nh_.subscribe("/ariac/break_beam_1_change", 1000,
                                                &AriacSensorManager::breakBeam1Callback, this);
    break_beam_2_subscriber_ = sensor_nh_.subscribe("/ariac/break_beam_2_change", 1000,
                                                &AriacSensorManager::breakBeam2Callback, this);
    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;
    camera4_frame_counter_ = 0;

    break_beam_1_trig_counter_ = 0;
    break_beam_2_trig_counter_ = 0;

    init_ = false;
    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;
    cam_4_ = false;
    break_beam_1_ = false;
    break_beam_2_ = false;

    // ros::spin();

}

AriacSensorManager::~AriacSensorManager() {}

void AriacSensorManager::LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 1: '" << image_msg->models.size() << "' objects.");

    if (image_msg->models.size() == 0) {
        ROS_ERROR_STREAM("Logical Camera 1 does not see anything");
    }

    current_parts_1_ = *image_msg;
    this->BuildProductFrames(1);
//    //--Object which represents a part
//    AriacPartManager part_manager;
//    //--Logical camera always represents the first product seen in frame 1
//
//    int part_frame{1};
//
//    for (auto& msg : image_msg->models) {
//        geometry_msgs::Pose part_pose;
//
//        //--position
//        part_pose.position.x = msg.pose.position.x;
//        part_pose.position.y = msg.pose.position.y;
//        part_pose.position.z = msg.pose.position.z;
//        //--orientation
//        part_pose.orientation.x = msg.pose.orientation.x;
//        part_pose.orientation.y = msg.pose.orientation.y;
//        part_pose.orientation.z = msg.pose.orientation.z;
//        part_pose.orientation.w = msg.pose.orientation.w;
//
//        part_manager.set_part_frame(part_frame);
//        part_manager.set_part_type(msg.type);
//        part_manager.set_part_pose(part_pose);
//            camera1_part_list.push_back(part_manager);
//        //--next frame id is incremented by 1
//        part_frame++;
//    }
//
//    for (auto &part: camera1_part_list)
//    {
//        ROS_INFO_STREAM(">>>>> Part type:" << part.get_part_type());
//        ROS_INFO_STREAM(">>>>> Part Pose x:" << part.get_part_pose().position.x);
//        ROS_INFO_STREAM(">>>>> Part frame:" << part.get_part_frame());
//    }
}


void AriacSensorManager::LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 2: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 2 does not see anything");

    current_parts_2_ = *image_msg;
    this->BuildProductFrames(2);
}

void AriacSensorManager::LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 3: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 3 does not see anything");

    current_parts_3_ = *image_msg;
    this->BuildProductFrames(3);
}

void AriacSensorManager::LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    // if (init_) return;
    ROS_INFO_STREAM_THROTTLE(20,
                             "Logical camera 4: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM_THROTTLE(20,"Logical Camera 4 does not see anything");

    current_parts_4_ = *image_msg;
    this->BuildProductFrames(4);
}

void AriacSensorManager::BuildProductFrames(int camera_id){
    if (camera_id == 1) {
        for (auto& msg : current_parts_1_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_1_" + msg.type + "_" +
                                        std::to_string(camera1_frame_counter_) + "_frame";
    
            product_frame_list_[msg.type].emplace_back(product_frame);
            camera1_frame_counter_++;
        }
        cam_1_ = true;
    }
    else if (camera_id == 2) {
        for (auto& msg : current_parts_2_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_2_" + msg.type + "_" +
                                        std::to_string(camera2_frame_counter_) + "_frame";
    
            product_frame_list_[msg.type].emplace_back(product_frame);
            camera2_frame_counter_++;
        }
        cam_2_ = true;
    }
    else if (camera_id == 3) {
        for (auto& msg : current_parts_3_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_3_" + msg.type + "_" +
                                        std::to_string(camera3_frame_counter_) + "_frame";
    
            product_frame_list_[msg.type].emplace_back(product_frame);
            camera3_frame_counter_++;
        }
        cam_3_ = true;
    }
    if (camera_id == 4) {
        ROS_INFO_STREAM_THROTTLE(10,"Logical camera 4 is built");
        for (auto& msg : current_parts_4_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_4_" + msg.type + "_" +
                  std::to_string(camera4_frame_counter_) + "_frame";
            product_frame_list_conv_[msg.type].emplace_back(product_frame);
            // camera4_frame_counter_++;
        }
        // ROS_INFO("Product Frame" << product_frame);
        cam_4_ = true;
    }
    if (cam_1_ && cam_2_ && cam_3_) {
        init_ = true;
    }
}


geometry_msgs::Pose AriacSensorManager::GetPartPose(const std::string& src_frame,
                                        const std::string& target_frame) {
    geometry_msgs::Pose part_pose;

    ROS_INFO_STREAM("Getting part pose...");
    ROS_INFO_STREAM("Source Frame" << src_frame);
    ROS_INFO_STREAM("Target Frame" << target_frame);
    if (init_) {
    camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                             ros::Duration(3));
    camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                            camera_tf_transform_);

    part_pose.position.x = camera_tf_transform_.getOrigin().x();
    part_pose.position.y = camera_tf_transform_.getOrigin().y();
    part_pose.position.z = camera_tf_transform_.getOrigin().z();

    } else {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(4);
        // ros::spinOnce();
        // ros::Duration(1.0).sleep();
        // this->BuildProductFrames(2);

        part_pose = this->GetPartPose(src_frame, target_frame);
    }

    return part_pose;
}

geometry_msgs::Pose AriacSensorManager::GetPartPoseFromConv(const std::string& src_frame,
                                        const std::string& target_frame) {
    geometry_msgs::Pose part_pose;

    ROS_INFO_STREAM("Getting part pose...");
    ROS_INFO_STREAM("Source Frame" << src_frame);
    ROS_INFO_STREAM("Target Frame" << target_frame);

    camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                             ros::Duration(3));
    camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                            camera_tf_transform_);

    part_pose.position.x = camera_tf_transform_.getOrigin().x();
    part_pose.position.y = camera_tf_transform_.getOrigin().y();
    part_pose.position.z = camera_tf_transform_.getOrigin().z();

    return part_pose;
}

//-- Break beam sensor 1 call back done for RWA-3
void AriacSensorManager::breakBeam1Callback(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_WARN_STREAM("Break beam 1 triggered.");
        break_beam_1_ = true;
        break_beam_1_trig_counter_++;
    } else {
        break_beam_1_ = false;
    }
}

//-- Break beam sensor 2 call back which is right under the logical cam 4 done for RWA-3
void AriacSensorManager::breakBeam2Callback(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
        ROS_WARN_STREAM("Break beam 2 triggered.");
        break_beam_2_ = true;
        break_beam_2_trig_counter_++;
    } else {
        break_beam_2_ = false;
    }
    // else{
        // ROS_ERROR_STREAM("Break not triggered");
    // }
}

// void AriacSensorManager::reset(){

//     // camera_4_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_4", 10,
//                                                 // &AriacSensorManager::LogicalCamera4Callback, this);
//     break_beam_1_subscriber_ = sensor_nh_.subscribe("/ariac/break_beam_1_change", 1000,
//                                                 &AriacSensorManager::breakBeam1Callback, this);
//     break_beam_2_subscriber_ = sensor_nh_.subscribe("/ariac/break_beam_2_change", 1000,
//                                                 &AriacSensorManager::breakBeam2Callback, this);
//     camera4_frame_counter_ = 1;

//     cam_4_ = false;
//     // break_beam_1_ = false;
//     // break_beam_2_ = false;
// }
