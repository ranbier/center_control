#include <ros/ros.h>
#include <coss_msgs/Coss.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace std;

class MissionControl {
public:
    explicit MissionControl(ros::NodeHandle& nh) : nh_(nh) {
        mission_from_lidar_sub_ = nh_.subscribe("/mission/lidar", 10, &MissionControl::LidarCallback, this);
        mission_from_camera_sub_ = nh_.subscribe("/mission/camera", 10, &MissionControl::CameraCallback, this);
        

        //topic publications
        cmd_goal_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_goal", 10);
    }
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

    void LidarCallback(const coss_msgs::Coss::ConstPtr& msg) {
        lidar_steer = msg->lidar_steer;
        lidar_speed = msg->lidar_speed;
        lidar_stop_flag_ = msg->lidar_stop_flag;

    }

    void CameraCallback(const coss_msgs::Coss::ConstPtr& msg) {
        cam_steer = msg->cam_steer;
        cam_speed = msg->cam_speed;
        mission_state = msg->mission_state;

    }

};

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