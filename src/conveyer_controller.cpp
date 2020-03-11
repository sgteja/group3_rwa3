
//--Created by Gnyana Teja.S 03/09/20

#import "conveyor_controller.h" 

//todo RWA-3 remove this later, stopping conveyer belt    
    

    ros::ServiceClient conv_client = 
            node.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    if (!conv_client.exists()){
        conv_client.waitForExistence();
    }
    osrf_gear::ConveyorBeltControl beltControl;
    beltControl.request.power = 0.0;
    conv_client.call(beltControl);
    if (!beltControl.response.success) {  // If not successful, print out why.
        ROS_ERROR_STREAM("Failed to control conveyer: " << beltControl.response.success);
    } else {
        ROS_INFO("Conveyor controlled!");
    }