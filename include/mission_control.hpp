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
    

    bool lidar_flag_{false};
    double lidar_steer = 0.0;
    double lidar_speed = 0.0;

    int mission_state = 0;
    double cam_steer = 0.0;
    double cam_speed = 0.0;
    bool cone_finish = false;
    bool tunnel_finish = false;
    bool parking_flag = false;
    bool cam_red_detection = false;
    bool cam_blue_detection = false;

    double steer1 = 0.0;
    double steer2 = 0.0;
    double steer3 = 0.0;
    double steer4 = 0.0;
    double steer5 = 0.0;
    double steer6 = 0.0;
    double steer7 = 0.0;
    double steer8 = 0.0;
    double steer9 = 0.0;
    double steer10 = 0.0;
    double steer11 = 0.0;
    double steer12 = 0.0;
    double steer13 = 0.0;
    double steer14 = 0.0;
    double steer15 = 0.0;

    double speed1 = 0.0;
    double speed2 = 0.0;
    double speed3 = 0.0;
    double speed4 = 0.0;
    double speed5 = 0.0;
    double speed6 = 0.0;
    double speed7 = 0.0;
    double speed8 = 0.0;
    double speed9 = 0.0;
    double speed10 = 0.0;
    double speed11 = 0.0;
    double speed12 = 0.0;
    double speed13 = 0.0;
    double speed14 = 0.0;
    double speed15 = 0.0;

    int count1 = 0;
    int count2 = 0;
    int count3 = 0;
    int count4 = 0;
    int count5 = 0;
    int count6 = 0;
    int count7 = 0;
    int count8 = 0;
    int count9 = 0;
    int count10 = 0;
    int count11 = 0;
    int count12 = 0;
    int count13 = 0;
    int count14 = 0;
    int count15 = 0;

    
    /*Legend of COUNT*/
    int count_cone = 0;
    int count_cone2 = 0;
    int count_parking = 0;

    bool keep_parking = false;

    void LidarCallback(const coss_msgs::Coss::ConstPtr& msg);

    void CameraCallback(const coss_msgs::Coss::ConstPtr& msg);

};
