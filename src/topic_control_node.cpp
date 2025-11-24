#include <ros/ros.h>
#include <coss_msgs/Coss.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace std;


class TopicControl {
public:
	TopicControl(ros::NodeHandle& nh) : nh_(nh) {


		//topic subscriptions
		lidar_sub_ = nh_.subscribe("/lidar", 10, &TopicControl::LidarCallback, this);
		camera_sub_ = nh_.subscribe("/camera", 10, &TopicControl::CameraCallback, this);


		//topic publications
		lidar_to_mission_pub_ = nh_.advertise<coss_msgs::Coss>("/mission/lidar", 10);
		camera_to_mission_pub_ = nh_.advertise<coss_msgs::Coss>("/mission/camera", 10);
		

	}
private:

	ros::NodeHandle nh_;


	//ros subscribers
	ros::Subscriber lidar_sub_;
	ros::Subscriber camera_sub_;
	
	
	//ros publishers
	ros::Publisher lidar_to_mission_pub_;
	ros::Publisher camera_to_mission_pub_;
	ros::Publisher final_cmd_pub_;



	//callback functions
	void LidarCallback(const coss_msgs::Coss::ConstPtr& msg) {
		double lidar_steer = msg->lidar_steer;
		double lidar_speed = msg->lidar_speed;
		double lidar_flag = msg->lidar_flag;
		bool cone_finish = msg->cone_finish;
		bool tunnel_finish = msg->tunnel_finish;

		coss_msgs::Coss lidar_msg;
		lidar_msg.lidar_steer = lidar_steer;
		lidar_msg.lidar_speed = lidar_speed;
		lidar_msg.lidar_flag = lidar_flag;
		lidar_msg.cone_finish = cone_finish;
		lidar_msg.tunnel_finish = tunnel_finish;

		lidar_to_mission_pub_.publish(lidar_msg);
	}

	void CameraCallback(const coss_msgs::Coss::ConstPtr& msg) {
		double cam_steer = msg->cam_steer;
		double cam_speed = msg->cam_speed;
		int mission_state = msg->mission_state;
		bool cam_red_detection = msg->cam_red_detection;
		bool cam_blue_detection = msg->cam_blue_detection;

		coss_msgs::Coss camera_msg;
		camera_msg.cam_steer = cam_steer;
		camera_msg.cam_speed = cam_speed;
		camera_msg.mission_state = mission_state;
		camera_msg.cam_red_detection = cam_red_detection;
		camera_msg.cam_blue_detection = cam_blue_detection;

		camera_to_mission_pub_.publish(camera_msg);
	}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "topic_control_node");
	ros::NodeHandle nh("~");

	TopicControl topic_control(nh);

	ros::spin();
	return 0;
}