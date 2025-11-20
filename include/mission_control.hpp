#pragma once

#include <ros/ros.h>
#include <coss_msgs/Coss.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


using namespace std;

class MissionControl {
public:
    explicit MissionControl(ros::NodeHandle& nh);
    void Process();

private:
    ros::NodeHandle nh_;
    
    //ros subscribers
    ros::Subscriber mission_from_lidar_sub_;
    ros::Subscriber mission_from_camera_sub_;
    
    //ros publishers
    ros::Publisher cmd_goal_pub_;    
    

    bool lidar_stop_flag_{false};
    double lidar_steer = 0.0;
    double lidar_speed = 0.0;

    int mission_state = 0;
    double cam_steer = 0.0;
    double cam_speed = 0.0;

    void LidarCallback(const coss_msgs::Coss::ConstPtr& msg);

    void CameraCallback(const coss_msgs::Coss::ConstPtr& msg);

};
