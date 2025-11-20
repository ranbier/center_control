#pragma once

#include <ros/ros.h>
#include <coss_msgs/Coss.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class CmdControl {
public:
    explicit CmdControl(ros::NodeHandle& nh);

private:

    ros::NodeHandle nh_;

    //ros subscribers
    ros::Subscriber cmd_goal_sub_;
    ros::Subscriber odom_sub_;

    //ros publishers
    ros::Publisher final_cmd_pub_;

    //PID parameters
    double Kp_;
    double Ki_;
    double Kd_;
    double integral_{0.0};
    double previous_error_{0.0};

    // odometry variables
    double current_speed_;

    double PIDControl(double target, double current, double& integral, double& previous_error, double dt);

    void CmdGoalCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

