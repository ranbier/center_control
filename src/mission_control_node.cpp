#include "mission_control.hpp"
#include "process.cpp"

MissionControl::MissionControl(ros::NodeHandle& nh) : nh_(nh) {
    mission_from_lidar_sub_ = nh_.subscribe("/mission/lidar", 10, &MissionControl::LidarCallback, this);
    mission_from_camera_sub_ = nh_.subscribe("/mission/camera", 10, &MissionControl::CameraCallback, this);
    
    //topic publications
    cmd_goal_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_goal", 10);

    nh_.param("steer1", steer1, 0.0);
    nh_.param("steer2", steer2, 0.0);
    nh_.param("steer3", steer3, 0.0);
    nh_.param("steer4", steer4, 0.0);
    nh_.param("steer5", steer5, 0.0);
    nh_.param("steer6", steer6, 0.0);
    nh_.param("steer7", steer7, 0.0);
    nh_.param("steer8", steer8, 0.0);
    nh_.param("steer9", steer9, 0.0);
    nh_.param("steer10", steer10, 0.0);
    nh_.param("steer11", steer11, 0.0);
    nh_.param("steer12", steer12, 0.0);
    nh_.param("steer13", steer13, 0.0);
    nh_.param("steer14", steer14, 0.0);
    nh_.param("steer15", steer15, 0.0);

    nh_.param("speed1", speed1, 0.0);
    nh_.param("speed2", speed2, 0.0);
    nh_.param("speed3", speed3, 0.0);
    nh_.param("speed4", speed4, 0.0);
    nh_.param("speed5", speed5, 0.0);
    nh_.param("speed6", speed6, 0.0);
    nh_.param("speed7", speed7, 0.0);
    nh_.param("speed8", speed8, 0.0);
    nh_.param("speed9", speed9, 0.0);
    nh_.param("speed10", speed10, 0.0);
    nh_.param("speed11", speed11, 0.0);
    nh_.param("speed12", speed12, 0.0);
    nh_.param("speed13", speed13, 0.0);
    nh_.param("speed14", speed14, 0.0);
    nh_.param("speed15", speed15, 0.0);
    nh_.param("count1", count1, 0);
    nh_.param("count2", count2, 0);
    nh_.param("count3", count3, 0);
    nh_.param("count4", count4, 0);
    nh_.param("count5", count5, 0);
    nh_.param("count6", count6, 0);
    nh_.param("count7", count7, 0);
    nh_.param("count8", count8, 0);
    nh_.param("count9", count9, 0);
    nh_.param("count10", count10, 0);
    nh_.param("count11", count11, 0);
    nh_.param("count12", count12, 0);
    nh_.param("count13", count13, 0);
    nh_.param("count14", count14, 0);
    nh_.param("count15", count15, 0);
}
void MissionControl::LidarCallback(const coss_msgs::Coss::ConstPtr& msg) {
        lidar_steer = msg->lidar_steer;
        lidar_speed = msg->lidar_speed;
        lidar_flag_ = msg->lidar_flag;
        cone_finish = msg->cone_finish;
        tunnel_finish = msg->tunnel_finish;
        parking_flag = msg->parking_flag;

}
void MissionControl::CameraCallback(const coss_msgs::Coss::ConstPtr& msg) {
        cam_steer = msg->cam_steer;
        cam_speed = msg->cam_speed;
        mission_state = msg->mission_state;
        cam_red_detection = msg->cam_red_detection;
        cam_blue_detection = msg->cam_blue_detection;

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh("~");

    ros::Rate loop_rate(10);
    MissionControl mission_control(nh);
    while(ros::ok()) {
        ros::spinOnce();
        mission_control.Process();
        loop_rate.sleep();
    }

    return 0;
}