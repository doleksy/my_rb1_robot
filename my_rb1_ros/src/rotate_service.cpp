#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <sstream>

#include "my_rb1_ros/Rotate.h"

class RotateRobotServer
{
public:
    RotateRobotServer()
        : nh_{}
        , server_(nh_.advertiseService("/rotate_robot", &RotateRobotServer::server_callback, this))
        , odom_(nh_.subscribe("/odom", 1000, &RotateRobotServer::odom_callback, this))
        , rotate_(nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000))
    {
        ROS_INFO("Service Ready");
    }


private:
    ros::NodeHandle nh_;
    ros::ServiceServer server_;
    ros::Subscriber odom_;
    ros::Publisher rotate_;
    
    ros::Rate rotateRate_{20};

    double odomYaw_{};


private:
    bool server_callback(my_rb1_ros::RotateRequest  &req,
                         my_rb1_ros::RotateResponse &res)
    {
        const double rotationRadians{angles::from_degrees(req.degrees)};
        ROS_INFO("Service Requested: %d degrees: %f radians", req.degrees, rotationRadians);

        double destRadians{odomYaw_ + rotationRadians};
        if (destRadians > TF2SIMD_PI)
        {
            destRadians -= TF2SIMD_2_PI;
        }
        else if (destRadians < -TF2SIMD_PI)
        {
            destRadians += TF2SIMD_2_PI;
        }
        ROS_DEBUG("odomYaw_: %f (%f degrees); dest: %f radians (%f degrees)", odomYaw_, angles::to_degrees(odomYaw_), destRadians, angles::to_degrees(destRadians));

        geometry_msgs::Twist rotateMsg{};
        if (req.degrees > 0)
        {
            rotateMsg.angular.z = 0.65;
        }
        else
        {
            rotateMsg.angular.z = -0.65;
        }

        constexpr double epsilon{0.015};

        while (std::abs(destRadians - odomYaw_) > epsilon)
        {
            ROS_DEBUG("dest: %f yaw: %f diff: %f", destRadians, odomYaw_, std::abs(destRadians - odomYaw_));
            rotate_.publish(rotateMsg);
            ros::spinOnce();
            rotateRate_.sleep();
        }

        rotateMsg.angular.z = 0;
        for (int i = 0; i < 4; ++i)
        {
            rotate_.publish(rotateMsg);
            rotateRate_.sleep();
        }

        ROS_INFO("Service Completed");

        std::ostringstream resultBuffer;
        resultBuffer << "Service /rotate_robot: Success: " << req.degrees << " degrees.";
        res.result = resultBuffer.str().c_str();
        return true;
    }

    void odom_callback(const nav_msgs::OdometryConstPtr &odom)
    {
        geometry_msgs::Quaternion orient = odom->pose.pose.orientation;
        ROS_DEBUG("Odom orientation: x=%f y=%f z=%f w=%f", orient.x, orient.y, orient.z, orient.w);

        tf2::Quaternion quat_tf;
        tf2::fromMsg(orient, quat_tf);

        tf2::Matrix3x3 m(quat_tf);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_DEBUG("Odom roll=%f pitch=%f yaw=%f", roll, pitch, yaw);

        odomYaw_ = yaw;
    }

};  // class RotateRobotServer


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotate_service");

    RotateRobotServer robot_server;    

    ros::spin();

    return 0;
}