#include <algorithm>

#include "cmd_control.hpp"


CmdControl::CmdControl(ros::NodeHandle& nh) : nh_(nh) {

    nh_.param("Kp", Kp_, 1.0);
    nh_.param("Ki", Ki_, 0.0);
    nh_.param("Kd", Kd_, 0.1);
    nh_.param("steering_angle_to_servo_gain", steering_angle_to_servo_gain_, -1.2135);
    nh_.param("steering_angle_to_servo_offset", steering_angle_to_servo_offset_, 0.5304);

    //topic subscriptions
    cmd_goal_sub_ = nh_.subscribe("/cmd_goal", 10, &CmdControl::CmdGoalCallback, this);
    servo_sub_ = nh_.subscribe("/sensors/servo_position_command", 10, &CmdControl::ServoCallback, this);
    last_cmd_time_ = ros::Time::now();

    //topic publications
    final_cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/high_level/ackermann_cmd_mux/input/nav_0", 10);
}


double CmdControl::PIDControl(double target, double current, double& integral, double& previous_error, double dt) {
    double error = target - current;
    integral += error * dt;
    double derivative = (error - previous_error) / dt;
    previous_error = error;

    double output = Kp_ * error + Ki_ * integral + Kd_ * derivative;
    return output;
}

void CmdControl::ServoCallback(const std_msgs::Float64::ConstPtr& msg) {
    
    current_steering_angle_ = (msg->data - steering_angle_to_servo_offset_) / steering_angle_to_servo_gain_;
    
}

void CmdControl::CmdGoalCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    // Extract commands from the incoming message
    double steer_command = msg->drive.steering_angle;  
    double speed_command = msg->drive.speed;

    ros::Time now = ros::Time::now();
    double dt = last_cmd_time_.isZero() ? 0.0 : (now - last_cmd_time_).toSec();
    last_cmd_time_ = now;
    if (dt <= 0.0) {
        dt = 0.1;  // fallback to reasonable loop interval
    }

    double correction = PIDControl(steer_command, current_steering_angle_, integral_, previous_error_, dt);
    double corrected_angle = steer_command + correction;
    corrected_angle = std::max(-0.27, std::min(0.31, corrected_angle));

    // Create and publish the AckermannDriveStamped message
    ackermann_msgs::AckermannDriveStamped cmd_msg;
    //cmd_msg.drive.steering_angle = corrected_angle;
    cmd_msg.drive.steering_angle = steer_command;
    cmd_msg.drive.speed = speed_command;
    final_cmd_pub_.publish(cmd_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_control_node");
    ros::NodeHandle nh("~");

    CmdControl cmd_control(nh);
    ros::spin();
    return 0;
}


