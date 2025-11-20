#include <cmd_control.hpp>


CmdControl::CmdControl(ros::NodeHandle& nh) : nh_(nh) {

    nh_.param("Kp", Kp_, 1.0);
    nh_.param("Ki", Ki_, 0.0);
    nh_.param("Kd", Kd_, 0.1);

    //topic subscriptions
    cmd_goal_sub_ = nh_.subscribe("/cmd_goal", 10, &CmdControl::CmdGoalCallback, this);
    odom_sub_ = nh_.subscribe("/vesc/odom", 10, &CmdControl::OdomCallback, this);
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

void CmdControl::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 현재 속도나 위치 정보를 여기서 처리할 수 있습니다.
    double current_speed = msg->twist.twist.linear.x;
    // 필요에 따라 다른 처리를 추가하세요.
}

void CmdControl::CmdGoalCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    // Extract commands from the incoming message
    double steer_command = msg->drive.steering_angle;  
    double speed_command = msg->drive.speed;

    // Create and publish the AckermannDriveStamped message
    ackermann_msgs::AckermannDriveStamped cmd_msg;
    cmd_msg.drive.steering_angle = steer_command;
    cmd_msg.drive.speed = speed_command;
    final_cmd_pub_.publish(cmd_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_control_node");
    ros::NodeHandle nh;

    CmdControl cmd_control(nh);
    ros::spin();
    return 0;
}


