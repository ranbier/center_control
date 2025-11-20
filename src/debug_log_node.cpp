#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class DebugLogNode {
public:
	explicit DebugLogNode(ros::NodeHandle& nh) {
		cmd_sub_ = nh.subscribe("/cmd_goal", 10, &DebugLogNode::CmdCallback, this);
		lidar_mission_sub_ = nh.subscribe("/mission/lidar", 10, &DebugLogNode::LidarMissionCallback, this);
		camera_mission_sub_ = nh.subscribe("/mission/camera", 10, &DebugLogNode::CameraMissionCallback, this);
	}
	void RosInfo() {
		ROS_INFO("------------------ Debug Log ------------------");
		ROS_INFO("goal -> steer: %f, speed: %f\nlidar -> steer: %f, speed: %f, stop_flag: %d\ncamera -> steer: %f, speed: %f, state: %d\n\n", 
			goal_steer_, goal_speed_,
			lidar_steer_, lidar_speed_, lidar_stop_flag_,
			cam_steer_, cam_speed_, mission_state_);
	}

private:
	ros::Subscriber cmd_sub_;
	ros::Subscriber lidar_mission_sub_;
	ros::Subscriber camera_mission_sub_;

	double goal_steer_;
	double goal_speed_;
	double lidar_steer_;
	double lidar_speed_;
	int lidar_stop_flag_;
	double cam_steer_;
	double cam_speed_;
	int mission_state_;

	void CmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
		goal_steer_ = msg->drive.steering_angle;
		goal_speed_ = msg->drive.speed;

	
	}

	void LidarMissionCallback(const coss_msgs::Coss::ConstPtr& msg) {
		lidar_steer_ = msg->lidar_steer;
		lidar_speed_ = msg->lidar_speed;
		lidar_stop_flag_ = msg->lidar_stop_flag;

	}
	void CameraMissionCallback(const coss_msgs::Coss::ConstPtr& msg) {
		cam_steer_ = msg->cam_steer;
		cam_speed_ = msg->cam_speed;
		mission_state_ = msg->mission_state;

	}
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "debug_log_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);
    DebugLogNode node(nh);
    while(ros::ok()) {
        ros::spinOnce();
        node.RosInfo();
        loop_rate.sleep();
    }

    return 0;
}

