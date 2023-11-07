#include "CalculateThread.h"
#include "feet_motor.h"
#include "Remote.h"
#include "AttitudeThread.h"
#include "cmsis_os.h"
#include "pid.h"
#include "Setting.h"
#include "user_lib.h"
#include "CanPacket.h"
#include "stdio.h"
#include "InterruptService.h"
#include "RefereeBehaviour.h"
#include "usart.h"
#include "MahonyAHRS.h"
#include "arm_math.h"
#include "CMS.h"


Chassis_t Chassis;
RC_ctrl_t Remote;
EulerSystemMeasure_t Imu;
Aim_t Aim;
PTZ_t PTZ;
ext_game_robot_status_t Referee;
extern ext_power_heat_data_t power_heat_data_t;
uint32_t F_Motor[8];

fp32 roting_speed;
fp32 Angle_zero_6020[4] = {-147.9, -168.0, -31.1, -117.7};
fp32 Direction[5] = {-1.0, -1.0, 1.0, 1.0, -1.0};
fp32 Maxspeed = 6000.0f;
fp32 speed[4];
fp32 angle[4];
KFP Power_kf;
fp32 Power_Max = 45.0f;
float angle_minus;
float run_per;


uint8_t Mode_last;
uint8_t Mode_now;

pid_type_def follow;
pid_type_def left_front_6020_speed_pid;
pid_type_def right_front_6020_speed_pid;
pid_type_def right_back_6020_speed_pid;
pid_type_def left_back_6020_speed_pid;
pid_type_def left_front_6020_position_pid;
pid_type_def right_front_6020_position_pid;
pid_type_def right_back_6020_position_pid;
pid_type_def left_back_6020_position_pid;
pid_type_def left_front_3508_pid;
pid_type_def right_front_3508_pid;
pid_type_def right_back_3508_pid;
pid_type_def left_back_3508_pid;
pid_type_def power_control_pid;
first_order_filter_type_t current_6020_filter_type;
first_order_filter_type_t current_3508_filter_type;
first_order_filter_type_t referee_power;
first_order_filter_type_t wheel_angle_1;
first_order_filter_type_t wheel_angle_2;
first_order_filter_type_t wheel_angle_3;
first_order_filter_type_t wheel_angle_4;
first_order_filter_type_t wz_filter;



fp32 follow_angle;
fp32 follow_PID[3]={FOLLOW_KP,FOLLOW_KI,FOLLOW_KD};
fp32 left_front_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 right_front_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 right_back_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 left_back_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 left_front_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 right_front_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 right_back_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 left_back_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 left_front_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 right_front_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 right_back_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 left_back_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 power_control_PID[3] = {power_control_KP, power_control_KI, power_control_KD};

extern motor_measure_t LEFT_FRONT_6020_Measure;
extern motor_measure_t RIGHT_FRONT_6020_Measure;
extern motor_measure_t RIGHT_BACK_6020_Measure;
extern motor_measure_t LEFT_BACK_6020_Measure;
extern motor_measure_t LEFT_FRONT_3508_Measure;
extern motor_measure_t RIGHT_FRONT_3508_Measure;
extern motor_measure_t RIGHT_BACK_3508_Measure;
extern motor_measure_t LEFT_BACK_3508_Measure;
extern motor_measure_t YawMotorMeasure;
OfflineMonitor_t Offline;

void ChassisInit();
void ChassisModeUpdate();
void ChassisPidUpadte();
void ChassisCommandUpdate();
void ChassisCurrentUpdate();
void RefereeInfUpdate(ext_game_robot_status_t *referee);
void ChassisInfUpdate();
void Angle_Speed_calc();
void PowerControl();
void CMS__();


void CalculateThread(void const *pvParameters)
{

	ChassisInit();

	while (1)
	{
		//Remote = *get_remote_control_point();
		
		DeviceOfflineMonitorUpdate(&Offline);
		ChassisModeUpdate();
		ChassisInfUpdate();
		RefereeInfUpdate(&Referee);
		GimbalEulerSystemMeasureUpdate(&Imu);
		ChassisCommandUpdate();
		PowerControl();
		CMS__();
		Chassis_Control(Chassis.Current[0],
						Chassis.Current[1],
						Chassis.Current[2],
						Chassis.Current[3],
						Chassis.Current[4],
						Chassis.Current[5],
						Chassis.Current[6],
						Chassis.Current[7]);
	
		osDelay(1);
	}
}

void ChassisInit()
{
	PID_init(&left_front_6020_speed_pid, PID_POSITION, left_front_6020_speed_PID, 30000, 10000);
	PID_init(&right_front_6020_speed_pid, PID_POSITION, right_front_6020_speed_PID, 30000, 10000);
	PID_init(&right_back_6020_speed_pid, PID_POSITION, right_back_6020_speed_PID, 30000, 10000);
	PID_init(&left_back_6020_speed_pid, PID_POSITION, left_back_6020_speed_PID, 30000, 10000);
	PID_init(&left_front_6020_position_pid, PID_POSITION, left_front_6020_position_PID, 300, 60);              //6020
	PID_init(&right_front_6020_position_pid, PID_POSITION, right_front_6020_position_PID, 300, 60);
	PID_init(&right_back_6020_position_pid, PID_POSITION, right_back_6020_position_PID, 300, 60);
	PID_init(&left_back_6020_position_pid, PID_POSITION, left_back_6020_position_PID, 300, 60);

	PID_init(&left_front_3508_pid, PID_POSITION, left_front_3508_PID, 16384, 1000);
	PID_init(&right_front_3508_pid, PID_POSITION, right_front_3508_PID, 16384, 1000);
	PID_init(&right_back_3508_pid, PID_POSITION, right_back_3508_PID, 16384, 1000);							//3508
	PID_init(&left_back_3508_pid, PID_POSITION, left_back_3508_PID, 16384, 1000);
	
	PID_init(&follow,PID_POSITION,follow_PID,1,1);
	
	KalmanFilter_init(&Power_kf, 0.0f , 0.0001f,0.0118f ,0.0,50.0,2.0);//A,B,P,Q,R                   //功率
	first_order_filter_init(&current_6020_filter_type,0.002,0.1);
	first_order_filter_init(&current_3508_filter_type,0.002,0.1);
	PID_init(&power_control_pid,PID_POSITION,power_control_PID,100.0f,100.0f);
	first_order_filter_init(&referee_power,0.001,0.1);
	first_order_filter_init(&wheel_angle_1,0.001,0.1);
	first_order_filter_init(&wheel_angle_2,0.001,0.1);
	first_order_filter_init(&wheel_angle_3,0.001,0.1);
	first_order_filter_init(&wheel_angle_4,0.001,0.1);
	first_order_filter_init(&wz_filter,0.001,1);

};

void ChassisInfUpdate()
{
	memcpy(&Chassis.Motor3508[0], &LEFT_FRONT_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[1], &RIGHT_FRONT_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[2], &RIGHT_BACK_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[3], &LEFT_BACK_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[0], &LEFT_FRONT_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[1], &RIGHT_FRONT_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[2], &RIGHT_BACK_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[3], &LEFT_BACK_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
}

void ChassisModeUpdate()
{
	
	switch (PTZ.ChassisStatueRequest)
	{
	case 0x01:
		Chassis.Mode = NOFORCE;
		break;
	case 0x12:
	case 0x32:
		Chassis.Mode = ROTING;
		break;
	case 0x0A:
	case 0x2A:
		Chassis.Mode = FALLOW;
		break;
	case 0x06:
	case 0x26:
		Chassis.Mode = STOP;
		break;

	default:
		break;
	}
	if((PTZ.ChassisStatueRequest & (0x01 << 5)) != 0)
	{
		Chassis.CapKey = 1;
	}
	else Chassis.CapKey = 0;
}

void ChassisCommandUpdate()
{
	
	
		
		//Chassis.wz = -Remote.rc.ch[2] / 660.0f * (1.0f + Chassis.Power_Proportion / Power_Max);

	if (Chassis.Mode == NOFORCE || Offline.PTZnode ==1)
	{
		Chassis.Current[0] = 0;
		Chassis.Current[1] = 0;
		Chassis.Current[2] = 0;
		Chassis.Current[3] = 0;
		Chassis.Current[4] = 0;
		Chassis.Current[5] = 0;
		Chassis.Current[6] = 0;
		Chassis.Current[7] = 0;
		return;
	}
	if (Chassis.Mode == FALLOW || Chassis.Mode == ROTING || Chassis.Mode == STOP ) //
	{
		follow_angle = loop_fp32_constrain(FollowAngle, YawMotorMeasure.angle - 180.0f,YawMotorMeasure.angle + 180.0f);
		
		if (Chassis.Mode == FALLOW)
		{
//			Chassis.vx = PTZ.FBSpeed / 32767.0f * (1.0f + Chassis.Power_Proportion /Power_Max );
//			Chassis.vy = -PTZ.LRSpeed / 32767.0f * (1.0f + Chassis.Power_Proportion / Power_Max);
			angle_minus = -YawMotorMeasure.angle + FollowAngle;
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (-PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI)) * (1.0f + Chassis.Power_Proportion /Power_Max );
			Chassis.vy =  ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (-PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI)) * (1.0f + Chassis.Power_Proportion /Power_Max );
			Chassis.wz = -PID_calc(&follow,YawMotorMeasure.angle,follow_angle); //* (1.0f + Chassis.Power_Proportion /Power_Max );
			first_order_filter_cali(&wz_filter,Chassis.wz);
			//Chassis.wz = wz_filter.out;
		}
		else if (Chassis.Mode == ROTING)
		{
			angle_minus = -YawMotorMeasure.angle + FollowAngle - YawMotorMeasure.speed_rpm * 0.9;
			Chassis.wz = (1.0f + Chassis.Power_Proportion /Power_Max );
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (-PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI)) / run_per ;//* (1.0f + Chassis.Power_Proportion /Power_Max );
			Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (-PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI)) / run_per ;//* (1.0f + Chassis.Power_Proportion /Power_Max );
			
		}
		else if (Chassis.Mode == STOP)
		{
			angle_minus = -YawMotorMeasure.angle + FollowAngle;
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (-PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI)) * (1.0f + Chassis.Power_Proportion /Power_Max );
			Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (-PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI)) * (1.0f + Chassis.Power_Proportion /Power_Max );
			Chassis.wz = 0.0;
		}
		
	}
			/********************************	6020角度解算         ***********************/ // 问题   i++  &&

		
		if (Fabs(PTZ.FBSpeed / 32767.0) > 0.05 || Fabs(PTZ.LRSpeed / 32767.0) > 0.05 )
		{
			for (uint8_t i = 0; i < 4; )
			{
				if(Chassis.Mode == FALLOW)
				Chassis.WheelAngle[i] = atan2((Chassis.vy + wz_filter.out * gen2 * Direction[i]), (Chassis.vx + wz_filter.out * gen2 * Direction[i + 1])) / 3.1415927 * 180.0 + Angle_zero_6020[i];
				else
				Chassis.WheelAngle[i] = atan2((Chassis.vy + Chassis.wz * gen2 * Direction[i]), (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1])) / 3.1415927 * 180.0 + Angle_zero_6020[i]; // ?????????
				 
				i++;
			}
			

		}
		else
		{
			if(Chassis.wz > 0)
			{
			Chassis.WheelAngle[0] = -135.0f + Angle_zero_6020[0];
			Chassis.WheelAngle[1] = 0 + Angle_zero_6020[1]; // 默认角度
			Chassis.WheelAngle[2] = 45.0f + Angle_zero_6020[2];
			Chassis.WheelAngle[3] = 180 + Angle_zero_6020[3];
			}
			else if(Chassis.wz < 0)
			{
			Chassis.WheelAngle[0] = 45.0f + Angle_zero_6020[0];
			Chassis.WheelAngle[1] = 180.0f + Angle_zero_6020[1]; // 默认角度
			Chassis.WheelAngle[2] = -135.0f + Angle_zero_6020[2];
			Chassis.WheelAngle[3] = 0.0f + Angle_zero_6020[3];							
			}	
			else if(Chassis.wz == 0)
			{
			Chassis.WheelAngle[0] = 45.0f + Angle_zero_6020[0];
			Chassis.WheelAngle[1] = 0 + Angle_zero_6020[1]; // 默认角度
			Chassis.WheelAngle[2] = 45.0f + Angle_zero_6020[2];
			Chassis.WheelAngle[3] = 0 + Angle_zero_6020[3];						
			}
			
		}
			
		
		Chassis.WheelAngle[0] = loop_fp32_constrain(Chassis.WheelAngle[0], LEFT_FRONT_6020_Measure.angle - 180.0f, LEFT_FRONT_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[1] = loop_fp32_constrain(Chassis.WheelAngle[1], RIGHT_FRONT_6020_Measure.angle - 180.0f, RIGHT_FRONT_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[2] = loop_fp32_constrain(Chassis.WheelAngle[2], RIGHT_BACK_6020_Measure.angle - 180.0f, RIGHT_BACK_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[3] = loop_fp32_constrain(Chassis.WheelAngle[3], LEFT_BACK_6020_Measure.angle - 180.0f, LEFT_BACK_6020_Measure.angle + 180.0f);		

		/***********************                 3508速度解算                    ******************************/
		
				//电容的使用
		if(CMS.RxOpen == 1 && CMS.TxOpen == 1)
		{
			Chassis.vx = 3 * Chassis.vx /(1.0f + Chassis.Power_Proportion /Power_Max );
			Chassis.vy = 3 * Chassis.vy /(1.0f + Chassis.Power_Proportion /Power_Max );
			Chassis.wz = 3 * Chassis.wz /(1.0f + Chassis.Power_Proportion /Power_Max );
			CMS.Mode = 1;
			power_control_pid.out = 0;
		}
		else CMS.Mode = 0;

		for (uint8_t i = 0; i < 4;)
		{
			speed[i] = sqrtf((Chassis.vy + Chassis.wz * gen2 * Direction[i]) * (Chassis.vy + Chassis.wz * gen2 * Direction[i]) 
							+ (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1]) * (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1]));
			i++;
		}
		Chassis.WheelSpeed[0] = -speed[0];
		Chassis.WheelSpeed[1] = speed[1];
		Chassis.WheelSpeed[2] = speed[2];
		Chassis.WheelSpeed[3] = -speed[3];
	

		Angle_Speed_calc(); // 角度优化

		Chassis.speed_6020[0] = PID_calc(&left_front_6020_position_pid, LEFT_FRONT_6020_Measure.angle, Chassis.WheelAngle[0]);
		Chassis.speed_6020[1] = PID_calc(&right_front_6020_position_pid, RIGHT_FRONT_6020_Measure.angle, Chassis.WheelAngle[1]);
		Chassis.speed_6020[2] = PID_calc(&right_back_6020_position_pid, RIGHT_BACK_6020_Measure.angle, Chassis.WheelAngle[2]);
		Chassis.speed_6020[3] = PID_calc(&left_back_6020_position_pid, LEFT_BACK_6020_Measure.angle, Chassis.WheelAngle[3]);
	


		
	ChassisCurrentUpdate();

	
	Mode_last = Mode_now;
	Mode_now = Chassis.Mode;

}

	

void Angle_Speed_calc()
{
	for (uint8_t i = 0; i < 4; )
	{
		if (Chassis.WheelAngle[i] - Chassis.Motor6020[i].angle > 90.0f)
		{
			Chassis.WheelAngle[i] -= 180.0f;
			Chassis.WheelSpeed[i] = -Chassis.WheelSpeed[i];
		}
		if (Chassis.WheelAngle[i] - Chassis.Motor6020[i].angle < -90.0f)
		{
			Chassis.WheelAngle[i] += 180.0f;
			Chassis.WheelSpeed[i] = -Chassis.WheelSpeed[i];
		}
		i++;
	}
	
//	first_order_filter_cali(&wheel_angle_1,Chassis.WheelAngle[0]);
//			first_order_filter_cali(&wheel_angle_2,Chassis.WheelAngle[1]);
//			first_order_filter_cali(&wheel_angle_3,Chassis.WheelAngle[2]);
//			first_order_filter_cali(&wheel_angle_4,Chassis.WheelAngle[3]);
//			Chassis.WheelAngle[0] = wheel_angle_1.out;
//			Chassis.WheelAngle[1] = wheel_angle_2.out;
//			Chassis.WheelAngle[2] = wheel_angle_3.out;
//			Chassis.WheelAngle[3] = wheel_angle_4.out;
	
}

void ChassisCurrentUpdate()
{
	Chassis.Current[0] = 0;
	Chassis.Current[1] = PID_calc(&right_front_6020_speed_pid, RIGHT_FRONT_6020_Measure.speed_rpm, Chassis.speed_6020[1]);
	Chassis.Current[2] = 0;
	Chassis.Current[3] = PID_calc(&left_back_6020_speed_pid, LEFT_BACK_6020_Measure.speed_rpm, Chassis.speed_6020[3]);

	Chassis.Current[4] = 0;
	Chassis.Current[5] = PID_calc(&right_front_3508_pid, RIGHT_FRONT_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[1]);
	Chassis.Current[6] = 0;
	Chassis.Current[7] = PID_calc(&left_back_3508_pid, LEFT_BACK_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[3]);
}

void RefereeInfUpdate(ext_game_robot_status_t *referee)
{
	memcpy(referee, &robot_state, sizeof(ext_game_robot_status_t));
	switch(referee->chassis_power_limit)
	{
		case 45:
			Power_Max = 45;run_per = 0.8;break;
		case 50:
			Power_Max = 50;run_per = 0.9;break;
		case 55:
			Power_Max = 55;run_per = 1.6;break;
		case 60:
			Power_Max = 60;run_per = 1.5;break;
		case 80:
			Power_Max = 80;run_per = 1;break;
		case 100:
			Power_Max = 100;run_per = 0.8;break;
		default:
			Power_Max = 45;run_per = 2;break;
		
	}
	
	
	
	
//	switch (Referee.chassis_power_limit)
//	{
//	case 60:
//		roting_speed = ROTING_SPEED_60;
//		break;

//	case 80:
//		roting_speed = ROTING_SPEED_80;
//		break;

//	case 100:
//		roting_speed = ROTING_SPEED_100;
//		break;

//	default:
//		roting_speed = ROTING_SPEED_60;
//		break;
//	}
}

void PowerControl()
{
	first_order_filter_cali(&referee_power,power_heat_data_t.chassis_power);
	Chassis.Power_Proportion = PID_calc(&power_control_pid,referee_power.out,Power_Max-2.0f);

	first_order_filter_cali(&current_6020_filter_type,Fabs(Chassis.Current[0])
	 														+Fabs(Chassis.Current[1])
	 														+Fabs(Chassis.Current[2])
	 														+Fabs(Chassis.Current[3]));
	first_order_filter_cali(&current_3508_filter_type, Fabs(Chassis.Current[4])
	 														+Fabs(Chassis.Current[5])
	 														+Fabs(Chassis.Current[6])
	 														+Fabs(Chassis.Current[7]));
	Chassis.Power_pre = KalmanFilter(&Power_kf,power_heat_data_t.chassis_power,current_6020_filter_type.out,current_3508_filter_type.out);
	// if(Chassis.Power_pre > Power_Max)
	// {
	// 	for(uint8_t i = 0;i<4;i++)
	// 	{
	// 		Chassis.speed_6020[i] *= Power_Max / Chassis.Power_pre  ;
	// 		Chassis.WheelSpeed[i] *= Power_Max / Chassis.Power_pre  ;
	// 	}
	// 	ChassisCurrentUpdate();

}

extern uint16_t cms_offline_counter;
void CMS__()
{
	if((Chassis.CapKey) && CMS.Electricity > 1200)
	{
		CMS.TxOpen = 1;
	}
	else CMS.TxOpen =0;
	
	if(CMS.Mode == 1)
		CMS.charge_limit = (Power_Max)*100;
	else if(CMS.Mode == 0)
		CMS.charge_limit = (Power_Max - Chassis.Power_pre)*100;

	
	
	if(CMS.Electricity < 1200 || CMS.Enable == 0 || cms_offline_counter > 200) //cms用不了
	{
		CMS.TxOpen = 0;	
		
	}
	cms_offline_counter ++;
	CMS_Referee_Send((uint16_t)CMS.charge_limit , CMS.TxOpen);
}



