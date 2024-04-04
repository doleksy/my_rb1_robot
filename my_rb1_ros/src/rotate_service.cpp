#include <cstdint>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

#include "my_rb1_ros/Rotate.h"
#include "tf2/LinearMath/Scalar.h"

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

    int32_t degrees_{};
    double odomYaw_{};

    static constexpr const double rotationAmount_{ 0.65 };
    static constexpr const double epsilon_{ 0.015 };


private:
    bool server_callback(my_rb1_ros::RotateRequest  &req,
                         my_rb1_ros::RotateResponse &res)
    {
        degrees_ = req.degrees;
        const double rotationRadians{angles::from_degrees(degrees_)};
        ROS_INFO("Service Requested: %d degrees: %f radians", degrees_, rotationRadians);

        auto remainingRotationRadians = rotate_full_circles(rotationRadians);

        rotate_sub_360(remainingRotationRadians);

        stop_rotation();

        ROS_INFO("Service Completed");

        res.result = "Rotation Successful";
        return true;
    }

    double rotate_full_circles(double rotationRadians)
    {
        if (rotationRadians < TF2SIMD_PI) return rotationRadians;

        ROS_INFO(">= 360: odomYaw_: %f (%f degrees); dest: %f radians (%f degrees)", odomYaw_, angles::to_degrees(odomYaw_), rotationRadians, angles::to_degrees(rotationRadians));

        geometry_msgs::Twist rotateMsg{};
        if (degrees_ > 0)
        {
            rotateMsg.angular.z = rotationAmount_;
        }
        else
        {
            rotateMsg.angular.z = -rotationAmount_;
        }

        double startingOdomYaw{ odomYaw_ };
        do {
            ROS_DEBUG(">= 360: dest: %f yaw: %f diff: %f", startingOdomYaw, odomYaw_, std::abs(startingOdomYaw - odomYaw_));
            rotate_.publish(rotateMsg);
            ros::spinOnce();
            rotateRate_.sleep();
        } while (std::abs(startingOdomYaw - odomYaw_) > epsilon_);

        rotationRadians -= TF2SIMD_2_PI;

        return rotationRadians;
    }

    void rotate_sub_360(double remainingRotationRadians)
    {
        assert(remainingRotationRadians < TF2SIMD_2_PI);

        double destRadians{odomYaw_ + remainingRotationRadians};
        if (destRadians > TF2SIMD_PI)
        {
            destRadians -= TF2SIMD_2_PI;
        }
        else if (destRadians < -TF2SIMD_PI)
        {
            destRadians += TF2SIMD_2_PI;
        }
        ROS_INFO("< 360: odomYaw_: %f (%f degrees); dest: %f radians (%f degrees)", odomYaw_, angles::to_degrees(odomYaw_), destRadians, angles::to_degrees(destRadians));

        geometry_msgs::Twist rotateMsg{};
        if (degrees_ > 0)
        {
            rotateMsg.angular.z = rotationAmount_;
        }
        else
        {
            rotateMsg.angular.z = -rotationAmount_;
        }

        while (std::abs(destRadians - odomYaw_) > epsilon_)
        {
            //ROS_DEBUG("dest: %f yaw: %f diff: %f", destRadians, odomYaw_, std::abs(destRadians - odomYaw_));
            rotate_.publish(rotateMsg);
            ros::spinOnce();
            rotateRate_.sleep();
        }
    }

    void stop_rotation(void)
    {
        geometry_msgs::Twist rotateMsg{};
        for (int i = 0; i < 4; ++i)
        {
            rotate_.publish(rotateMsg);
            rotateRate_.sleep();
        }
    }
    
    void odom_callback(const nav_msgs::OdometryConstPtr &odom)
    {
        geometry_msgs::Quaternion orient = odom->pose.pose.orientation;
        //ROS_DEBUG("Odom orientation: x=%f y=%f z=%f w=%f", orient.x, orient.y, orient.z, orient.w);

        tf2::Quaternion quat_tf;
        tf2::fromMsg(orient, quat_tf);

        tf2::Matrix3x3 m(quat_tf);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        //ROS_DEBUG("Odom roll=%f pitch=%f yaw=%f", roll, pitch, yaw);

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