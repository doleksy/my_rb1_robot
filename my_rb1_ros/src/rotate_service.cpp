#include <ros/ros.h>
#include "my_rb1_ros/Rotate.h"
#include "my_rb1_ros/RotateRequest.h"
#include "my_rb1_ros/RotateResponse.h"

bool server_callback(my_rb1_ros::RotateRequest  &req,
                     my_rb1_ros::RotateResponse &res)
{
    ROS_INFO("Received: %d", req.degrees);
    res.result = "Rotation Successful";
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotate_service");
    ros::NodeHandle nh;

    ros::ServiceServer my_srv = nh.advertiseService("/rotate_robot", server_callback);
    ros::spin();

    return 0;
}