#include "CMS.h"
#include "CanPacket.h"
#include "setting.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern PTZ_t PTZ;

CMS_t CMS;

static CAN_TxHeaderTypeDef  can_tx_message;
uint8_t can_send_data[3];

 //Ö¸Áî·¢ËÍ


void CMS_Referee_Send(uint16_t charge_limit, uint8_t enable)
{
        uint32_t send_mail_box;
    can_tx_message.StdId = CMSCurrentSendID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x03;
    can_send_data[0] = charge_limit >> 8;
    can_send_data[1] = charge_limit;
    can_send_data[2] = enable;
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}

