#ifndef SETTING_H
#define SETTING_H

#include "struct_typedef.h"


#define LENGTH 1000
#define gen2 1.414213562373/2.0

#define MOTOR_OFFLINE_TIMEMAX   50
#define REMOTE_OFFLINE_TIMEMAX  550
#define AIMBOT_OFFLINE_TIMEMAX  550
#define REFEREE_OFFLINE_TIMEMAX 3000
#define PTZ_OFFLINE_TIMEMAX     50

#define PARAMETER_FILE "Setting.h"

// imu安装方向
#define IMU_DIRECTION_rzrxy_XYZ
// imu yaw轴零飘偏置
#define IMU_GYRO_YAW_BIAS    							-0.003f
// 电机ID分配
#define LEFT_FRONT_6020         			    0x205
#define RIGHT_FRONT_6020         			    0x206
#define RIGHT_BACK_6020                         0x207
#define LEFT_BACK_6020                          0x208
#define LEFT_FRONT_3508         			    0x201
#define RIGHT_FRONT_3508         			    0x202
#define RIGHT_BACK_3508                         0x203
#define LEFT_BACK_3508                          0x204 


#define YawMotorId                                     	0x205

#define FollowAngle              						116.134f



#define FOLLOW_KP						0.2f
#define FOLLOW_KI						0.0f
#define FOLLOW_KD						20.0f



#define TURN_KP							1000.0f//200.0f
#define TURN_KI							0.0f//0.0f
#define TURN_KD							0.0f//50000.0f
 
#define SPEED_6020_KP							100.0f
#define SPEED_6020_KI							0.0f
#define SPEED_6020_KD							1.0f

#define POSITION_6020_KP                        10.0f
#define POSITION_6020_KI                        0.0f
#define POSITION_6020_KD                        0.03f

#define speed_3508_KP							3000.0f
#define speed_3508_KI							0.0f
#define speed_3508_KD							0.0f

#define power_control_KP                        0.5f
#define power_control_KI                        0.01000000016f 
#define power_control_KD                        0.0f

// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan1
#define CONTROL_CANPORT             hcan2

#define ROTING_SPEED_60					1.6f
#define ROTING_SPEED_80					1.8f
#define ROTING_SPEED_100				2.2f

#define OFFLINE_TIME                    100

#define CMSCurrentSendID 0x210
#define CMSRecceiveID 0x211



#endif

