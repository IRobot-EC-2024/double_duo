#include "InterruptService.h"
#include "main.h"
#include "feet_motor.h"
#include "CanPacket.h"
#include "struct_typedef.h"
#include "Setting.h"
#include "usart.h"
#include "user_lib.h"
#include "CalculateThread.h"
#include "CMS.h"


extern uint8_t u1_buf[32];
extern EulerSystemMeasure_t    Imu;

double t00=0,tk[4],t01=0,t02=0,ct=0,ct2=0,t03=0;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;

int receive_times=0,send_times=0;

uint8_t left_flag=0,right_flag=1;

extern int16_t left_foot_position;
extern int16_t right_foot_position;
extern int16_t wz_current_right;
extern int16_t wz_current_left;
extern int16_t wz_current;
extern uint16_t init_time;
extern fp32 left_slider_foot_current;
extern fp32 right_slider_foot_current;
extern uint8_t offline_flag;

CAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8];
uint16_t left_counter = 0;
uint16_t right_counter = 0;
uint16_t feet_left_counter = 0;
uint16_t feet_right_counter = 0;
uint16_t cms_offline_counter = 0;
extern uint32_t F_Motor[8];
OfflineCounter_t OfflineCounter;
OfflineMonitor_t OfflineMonitor;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
 
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	if(hcan->Instance == CAN2)
	{
		switch (rx_header.StdId)
		{
			
			case LEFT_FRONT_6020:
			{
				OfflineCounter.Motor[0] = 0;
				F_Motor[0] ++;
            MotorProcess(rx_header.StdId, hcan, rx_data);
		
            break;

			}
			case RIGHT_FRONT_6020:
			{
				OfflineCounter.Motor[1] = 0;
				F_Motor[1] ++;
            MotorProcess(rx_header.StdId, hcan, rx_data);
				
            break;
			}			
			case RIGHT_BACK_6020:
			{
				OfflineCounter.Motor[2] = 0;
				F_Motor[2] ++;
		
            MotorProcess(rx_header.StdId, hcan, rx_data);
				
            break;
			}
			case LEFT_BACK_6020:
			{
				OfflineCounter.Motor[3] = 0;
			F_Motor[3] ++;
            MotorProcess(rx_header.StdId, hcan, rx_data);

            break;
			}
			case LEFT_FRONT_3508:
			{
				OfflineCounter.Motor[4] = 0;
				F_Motor[4] ++;
            MotorProcess(rx_header.StdId, hcan, rx_data);
			
			break;
			}
			case RIGHT_FRONT_3508:
			{
				OfflineCounter.Motor[5] = 0;
				F_Motor[5] ++;
            MotorProcess(rx_header.StdId, hcan, rx_data);
			
			break;
			}
			case RIGHT_BACK_3508:
			{
				OfflineCounter.Motor[6] = 0;
				F_Motor[6] ++;
            MotorProcess(rx_header.StdId, hcan, rx_data);
			
			break;
			}
			case LEFT_BACK_3508:
			{
				OfflineCounter.Motor[7] = 0;
				F_Motor[7] ++;
            MotorProcess(rx_header.StdId, hcan, rx_data);
			
			break;
			}
			
			
			default:
			{
			break;
			}
		}
	}
	else if(hcan->Instance == CAN1)
	{
		switch(rx_header.StdId)
		{
			case YawMotorId:
			{
			//MotorProcess(rx_header.StdId,hcan,rx_data);
				get_motor_measure(&YawMotorMeasure,rx_data);
			break;
			}
			/*-------------------------------------云台数据下发接收-------------------------------------*/
			case DefaultAimStatusAndTargetId:
			{
				memcpy(&Aim,rx_data,sizeof(Aim_t));
				break;
			}
			case SentryAimStatusAndTargetId:
			{
				break;//哨兵相关，暂时不管
			}
			case DefaulPTZRequestAndStatusId:
			{
				OfflineCounter.PTZnode = 0;
				memcpy(&PTZ,rx_data,sizeof(PTZ_t));
				break;
			}
			case SentryPTZRequestAndStatusId:
			{
				break;//哨兵相关，不管
			}
			case CMSRecceiveID:
			{
				cms_offline_counter = 0;
				CMS.Enable = rx_data[0];// 使能
				CMS.RxOpen = rx_data[1]; //cms放电
				CMS.Power = (int16_t)rx_data[2]<<8|rx_data[3]; //地盘功率*100
				CMS.Electricity = (int16_t)rx_data[4]<<8|rx_data[5]; //电压 * 100
				break;
			}
			default:
			{
				break;
			}
		
		
		}
	
	
	
	}
	
}





void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop1000Hz();
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop500Hz_1();
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
    TimerTaskLoop500Hz_2();
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    TimerTaskLoop100Hz();
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}
void CommuniteOfflineCounterUpdate(void);
void CommuniteOfflineStateUpdate(void);
extern Chassis_t Chassis;
extern KFP Power_kf;
extern ext_power_heat_data_t power_heat_data_t;
extern first_order_filter_type_t current_6020_filter_type;
extern first_order_filter_type_t current_3508_filter_type;
extern first_order_filter_type_t wz_filter;

void TimerTaskLoop1000Hz()
{
	
	CommuniteOfflineCounterUpdate();
	CommuniteOfflineStateUpdate();
	
	//DMA_printf("%f,%d\r\n",power_heat_data_t.chassis_power,power_heat_data_t.chassis_power_buffer);
	//DMA_printf("%f,%f\r\n",wz_filter.out,Chassis.wz);
		
	//t0++;
	
	



}

void TimerTaskLoop500Hz_1()
{
	

		


}

void TimerTaskLoop500Hz_2()
{
			
		
	
	
}

void TimerTaskLoop100Hz()
{


}




void CommuniteOfflineCounterUpdate(void)
{
    OfflineCounter.Motor[0]++;
	OfflineCounter.Motor[1]++;
	OfflineCounter.Motor[2]++;
	OfflineCounter.Motor[3]++;
	OfflineCounter.Motor[4]++;
	OfflineCounter.Motor[5]++;
	OfflineCounter.Motor[6]++;
	OfflineCounter.Motor[7]++;
	
	OfflineCounter.PTZnode++;
}

void CommuniteOfflineStateUpdate(void)
{
    // Motor
	for(uint8_t i=0;i<8;i++)
	{
		if(OfflineCounter.Motor[i] > MOTOR_OFFLINE_TIMEMAX)
		{
			OfflineMonitor.Motor[i]=1;
		}
		else
		{
			OfflineMonitor.Motor[i]=0;
		}
	}
	if(OfflineCounter.PTZnode > PTZ_OFFLINE_TIMEMAX)
		OfflineMonitor.PTZnode = 1;
	else 
		OfflineMonitor.PTZnode = 0;
    
}

void DeviceOfflineMonitorUpdate(OfflineMonitor_t *Monitor)
{
    memcpy(Monitor, &OfflineMonitor, sizeof(OfflineMonitor_t));
}