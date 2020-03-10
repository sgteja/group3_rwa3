#include <osrf_gear/ConveyorBeltControl.h>
#include <ros/ros.h>

class AriacConveyerManager {
public:
    AriacConveyerManager();
    ~AriacConveyerManager();

 	void setConveyerPower(double power);
    

private:
    ros::NodeHandle conveyer_nh_;
   	ros::ServiceClient conveyer_client_;
};