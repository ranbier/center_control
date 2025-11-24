#include "mission_control.hpp"

void MissionControl::Process() {
    double steer_command;   //rad
    double speed_command;   //m/s
    ackermann_msgs::AckermannDriveStamped cmd_msg;
    
    switch(mission_state) {
        // 처음 시작구간
        case 0: 
            steer_command = cam_steer;
            speed_command = 0.25;
            break;

        // 레드, 블루존
        case 1:
            if(cam_red_detection && !cam_blue_detection){
                steer_command = cam_steer;
                speed_command = 0.2;
            }
            else if(!cam_red_detection && cam_blue_detection) {
                steer_command = cam_steer;
                speed_command = 0.6;
            }
            else{ 
                steer_command = cam_steer; // 민우 추가
                speed_command = 0.25;                
            }
            
            break;
        // 횡단보도
        case 2:
            steer_command = 0.0;
            speed_command = 0.0;
            break;
        // 횡단보도 출발 후 라이다 라바콘 주행
        case 3:
            if(count_cone < 30) {           // 피팅 필요
                steer_command = 0.0;
                speed_command = 0.5;
                count_cone++;
            } else {
               if(cone_finish) {
                steer_command = cam_steer;
                speed_command = 0.25;
                } else {
                    steer_command = lidar_steer;
                    speed_command = lidar_speed;
                }
            }
            break;
        // 차선 변경 구간
        case 4:
            steer_command = cam_steer;
            speed_command = 0.25;
            break;
        // 차선 변경 구간 완료 후 회전교차로 진입 전
        case 5:
            steer_command = cam_steer;
            speed_command = 0.25;
            break;
        // 회전교차로
        case 6:
            if(lidar_flag_) {
                steer_command = cam_steer;
                speed_command = 0.25;
            } else {
                steer_command = 0.0;
                speed_command = 0.0;
            }
            break;
        // 라이다 라바콘 2
        case 7:
            if(count_cone2 < 40) {           // 피팅 필요
                steer_command = cam_steer;
                speed_command = 0.25;
                count_cone2++;
            }
            // } else if(count_cone2 < 50) {           // 피팅 필요
            //     steer_command = 0.0;
            //     speed_command = 0.5;
            //     count_cone2++;
            // } 
            else {
               if(cone_finish) {
                steer_command = cam_steer;
                speed_command = 0.4;
                } else {
                    steer_command = lidar_steer;
                    speed_command = lidar_speed;
                }
            }
            break;
        // 터널 구간
        case 8:
            if(tunnel_finish) {
                steer_command = cam_steer;
                speed_command = 0.4;
            } else {
                steer_command = lidar_steer;
                speed_command = lidar_speed;
            }
            break;
        // 차단기 정지 구간
        case 9:
            if (lidar_flag_) {
                steer_command = 0.0;
                speed_command = 0.0;
            } else {
                steer_command = cam_steer;
                speed_command = 0.4;
            }
            break;
        // 주차 구간
        case 10:
            if (count_parking < 10) {      // 피팅 필요
                steer_command = 0.0;
                speed_command = -0.3;
                count_parking++;
            } else if (count_parking >= 10 && count_parking < 20) {
                steer_command = 0.4;
                speed_command = -0.3;
                count_parking++;
            } else if (count_parking >= 20 && count_parking < 30) {
                steer_command = -0.4;
                speed_command = -0.3;
                count_parking++;
            } else if (count_parking >= 30 && count_parking < 40) {
                steer_command = 0.0;
                speed_command = 0.0;
                cmd_msg.drive.steering_angle = steer_command;
                cmd_msg.drive.speed = speed_command;
                cmd_goal_pub_.publish(cmd_msg);
                count_parking++;
                ros::Duration(1.0).sleep();
            }
            else if (count_parking >= 40 && count_parking < 50) {
                steer_command = -0.4;
                speed_command = 0.3;
                count_parking++;
            }
            else if (count_parking >= 50 && count_parking < 60) {
                steer_command = 0.4;
                speed_command = 0.3;
                count_parking++;
            }
            else {
                steer_command = 0.0;
                speed_command = 0.0;
            }
            break;
        default:
            // Keep the most recent camera command when mission_state is unknown
            break;
    }
   
    cmd_msg.drive.steering_angle = steer_command;
    cmd_msg.drive.speed = speed_command;
    cmd_goal_pub_.publish(cmd_msg);


}