
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class DebugLogNode {
public:
	explicit DebugLogNode(ros::NodeHandle& nh) {
		cmd_sub_ = nh.subscribe("/cmd_goal", 10, &DebugLogNode::CmdCallback, this);
	}

private:
	ros::Subscriber cmd_sub_;

	void CmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
		ROS_INFO_STREAM_THROTTLE(0.5,
								 "cmd_goal -> steer: " << msg->drive.steering_angle
								 << ", speed: " << msg->drive.speed);
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "debug_log_node");
	ros::NodeHandle nh;

	DebugLogNode node(nh);
	ros::spin();
	return 0;
}


