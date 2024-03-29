#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

#include "my_rb1_ros/Rotate.h"

namespace
{
    double odomYaw{};
}   // unnamed

bool server_callback(my_rb1_ros::RotateRequest  &req,
                     my_rb1_ros::RotateResponse &res)
{
    const double rotationRadians{angles::from_degrees(req.degrees)};
    ROS_INFO("Received: %d degrees: %f radians", req.degrees, rotationRadians);

    ros::NodeHandle nh;
    ros::Publisher rotate = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    double destRadians{odomYaw + rotationRadians};
    if (destRadians > TF2SIMD_PI)
    {
        destRadians -= TF2SIMD_2_PI;
    }
    else if (destRadians < -TF2SIMD_PI)
    {
        destRadians += TF2SIMD_2_PI;
    }
    ROS_INFO("odomYaw: %f (%f degrees); dest: %f radians (%f degrees)", odomYaw, angles::to_degrees(odomYaw), destRadians, angles::to_degrees(destRadians));

    geometry_msgs::Twist rotateMsg;
    if (req.degrees > 0)
    {
        rotateMsg.angular.z = 0.1;
    }
    else
    {
        rotateMsg.angular.z = -0.1;
    }

    ros::Rate rotateRate(20);
    constexpr double epsilon{0.02};

    while (std::abs(destRadians - odomYaw) > epsilon)
    {
        //ROS_INFO("dest: %f yaw: %f diff: %f", destRadians, odomYaw, std::abs(destRadians - odomYaw));
        rotate.publish(rotateMsg);
        ros::spinOnce();
        rotateRate.sleep();
    }

    rotateMsg.angular.z = 0;
    for (int i = 0; i < 4; ++i)
    {
        rotate.publish(rotateMsg);
        rotateRate.sleep();
    }

    res.result = "Rotation Successful";
    return true;
}

void odom_callback(const nav_msgs::OdometryConstPtr &odom)
{
    geometry_msgs::Quaternion orient = odom->pose.pose.orientation;
    //ROS_INFO("Odom orientation: x=%f y=%f z=%f w=%f", orient.x, orient.y, orient.z, orient.w);

    tf2::Quaternion quat_tf;
    tf2::fromMsg(orient, quat_tf);

    tf2::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //ROS_INFO("Odom roll=%f pitch=%f yaw=%f", roll, pitch, yaw);

    odomYaw = yaw;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotate_service");
    ros::NodeHandle nh;

    ros::ServiceServer my_srv = nh.advertiseService("/rotate_robot", server_callback);

    ros::Subscriber odom = nh.subscribe("/odom", 1000, odom_callback);
    ros::spin();

    return 0;
}