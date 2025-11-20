#include "mission_control.hpp"
#include "process.cpp"

MissionControl::MissionControl(ros::NodeHandle& nh) : nh_(nh) {
    mission_from_lidar_sub_ = nh_.subscribe("/mission/lidar", 10, &MissionControl::LidarCallback, this);
    mission_from_camera_sub_ = nh_.subscribe("/mission/camera", 10, &MissionControl::CameraCallback, this);
    
    //topic publications
    cmd_goal_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_goal", 10);
}
void MissionControl::LidarCallback(const coss_msgs::Coss::ConstPtr& msg) {
        lidar_steer = msg->lidar_steer;
        lidar_speed = msg->lidar_speed;
        lidar_stop_flag_ = msg->lidar_stop_flag;

}
void MissionControl::CameraCallback(const coss_msgs::Coss::ConstPtr& msg) {
        cam_steer = msg->cam_steer;
        cam_speed = msg->cam_speed;
        mission_state = msg->mission_state;

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);
    MissionControl mission_control(nh);
    while(ros::ok()) {
        ros::spinOnce();
        mission_control.Process();
        loop_rate.sleep();
    }

    return 0;
}