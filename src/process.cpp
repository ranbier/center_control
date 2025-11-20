#include <mission_control_node.cpp>

void MissionControl::Process() {
    double steer_command;   //rad
    double speed_command;   //m/s

    switch(mission_state) {
        // 처음 시작구간
        case 0: 
            steer_command = cam_steer;
            speed_command = cam_speed;
            break;

        // 레드, 블루존
        case 1:
            steer_command = cam_steer;
            speed_command = 0.3;
            break;
        // 횡단보도
        case 2:
            steer_command = cam_steer;
            break;

        // 라이다 라바콘
        case 3:
            steer_command = lidar_steer;
            speed_command = lidar_speed;
            break;
        
        // 차선 변경 구간
        case 4:
            steer_command = cam_steer;
            speed_command = cam_speed;
            break;
        // 회전교차로
        case 5:
            steer_command = cam_steer;
            speed_command = cam_speed;
            break;
        // 라이다 라바콘 2
        case 6:
            steer_command = cam_steer;
            speed_command = cam_speed;
            break;
        // 터널 구간
        case 7:
            steer_command = lidar_steer;
            speed_command = lidar_speed;
            break;
        // 차단기 정지 구간
        case 8:
            if (lidar_stop_flag_) {
                speed_command = 0.0;
            } else {
                steer_command = cam_steer;
                speed_command = cam_speed;
            }
            break;
        // 주차 구간
        case 9:
            steer_command = cam_steer;
            speed_command = cam_speed;
            break;
        default:
            // Keep the most recent camera command when mission_state is unknown
            break;
    }
    ackermann_msgs::AckermannDriveStamped cmd_msg;
    cmd_msg.drive.steering_angle = steer_command;
    cmd_msg.drive.speed = speed_command;
    cmd_goal_pub_.publish(cmd_msg);


}