#include "mission_control.hpp"

void MissionControl::Process() {
    double steer_command;   //rad
    double speed_command;   //m/s
    ackermann_msgs::AckermannDriveStamped cmd_msg;
    
    
    switch(mission_state) { // mission_state
        // 처음 시작구간
        case 0: 
            steer_command = cam_steer;
            speed_command = speed8;
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
                speed_command = speed8;                
            }
            
            break;
        // 횡단보도
        case 2:
            steer_command = 0.0;
            speed_command = 0.0;
            break;
        // 횡단보도 출발 후 라이다 라바콘 주행
        case 3:
            if(count_cone < count8) {           // 피팅 필요
                steer_command = 0.0;
                speed_command = speed10;
                count_cone++;
            } else {
               if(cone_finish) {
                steer_command = cam_steer;
                speed_command = speed9;
                } else {
                    steer_command = lidar_steer;
                    speed_command = lidar_speed;
                }
            }
            break;
        // 차선 변경 구간
        case 4:
            steer_command = cam_steer;
            speed_command = speed8;
            break;
        // 차선 변경 구간 완료 후 회전교차로 진입 전
        case 5:
            steer_command = cam_steer;
            speed_command = speed8;
            break;
        // 회전교차로
        case 6:
            if(lidar_flag_) {
                steer_command = cam_steer;
                speed_command = speed8;
            } else {
                steer_command = 0.0;
                speed_command = 0.0;
            }
            break;
        // 라이다 라바콘 2
        case 7:
            if(count_cone2 < count9) {           // 피팅 필요
                steer_command = cam_steer;
                speed_command = speed10;
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
                speed_command = speed9;
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
                speed_command = speed8;
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
                    // steer_command = cam_steer;
                    steer_command = -0.03;
                    speed_command = speed8;
            }
            break;
        // 주차 구간
        case 10:
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03; // 카메라로 바꾸기
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
            }
            
            
            break;
        case 11:
            if(keep_parking && !parking_once1) {
                keep_parking = false;
                parking_once1 = true;
                count_parking = 0;
            }
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03;
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
            }
            
            
            break;
        case 12:
            if(keep_parking && !parking_once2) {
                keep_parking = false;
                parking_once2 = true;
                count_parking = 0;
            }
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03;
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
            }
            
            
            break;
        case 13:
            if(keep_parking && !parking_once3) {
                keep_parking = false;
                parking_once3 = true;
                count_parking = 0;
            }
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03;
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
            }
            
            
            break;
        case 14:
            if(keep_parking && !parking_once4) {
                keep_parking = false;
                parking_once4 = true;
                count_parking = 0;
            }
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03;
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
            }
            
            
            break;
        case 15:
            if(keep_parking && !parking_once5) {
                keep_parking = false;
                parking_once5 = true;
                count_parking = 0;
            }
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03;
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
            }
            
            
            break;
        case 16:
            if(keep_parking && !parking_once6) {
                keep_parking = false;
                parking_once6 = true;
                count_parking = 0;
            }
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03;
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
            }
            
            
            break;

        case 17:
            if(keep_parking && !parking_once7) {
                keep_parking = false;
                parking_once6 = true;
                count_parking = 0;
            }
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03;
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
            }
            
            
            break;
        case 18:
            if(keep_parking && !parking_once8) {
                keep_parking = false;
                parking_once8 = true;
                count_parking = 0;
            }
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03;
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
            }
            
            
            break;
        case 19:
            if(keep_parking && !parking_once9) {
                keep_parking = false;
                parking_once9 = true;
                count_parking = 0;
            }
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03;
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
            }
            
            
            break;
        case 20:
            if(keep_parking && !parking_once10) {
                keep_parking = false;
                parking_once10 = true;
                count_parking = 0;
            }
            if (parking_flag) {
                if (!keep_parking) {
                
                    keep_parking = true;
                }
            }
            else {
                // steer_command = cam_steer;
                steer_command = -0.03;
                speed_command = speed8;
            }
            if (keep_parking) {
                if (count_parking < count1) {      // 피팅 필요
                    steer_command = steer1;
                    speed_command = speed1;
                    count_parking++;
                } else if (count_parking >= count1 && count_parking < count2) {
                    steer_command = steer2;
                    speed_command = speed2;
                    count_parking++;
                } else if (count_parking >= count2 && count_parking < count3) {
                    steer_command = steer3;
                    speed_command = speed3;
                    count_parking++;
                } else if (count_parking >= count3 && count_parking < count4) {
                    steer_command = steer4;
                    speed_command = speed4;
                    count_parking++;
                }
                else if (count_parking >= count4 && count_parking < count5) {
                    steer_command = steer5;
                    speed_command = speed5;
                    count_parking++;
                }
                else if (count_parking >= count5 && count_parking < count6) {
                    steer_command = steer6;
                    speed_command = speed6;
                    count_parking++;
                }
                else if (count_parking >= count6 && count_parking < count7) {
                    steer_command = steer7;
                    speed_command = speed7;
                }
                else if (count_parking >= count7 && count_parking < count11) {
                    steer_command = steer11;
                    speed_command = speed11;
                    count_parking++;
                }
                else if (count_parking >= count11 && count_parking < count12) {
                    steer_command = steer12;
                    speed_command = speed12;
                    count_parking++;
                }
                
                
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