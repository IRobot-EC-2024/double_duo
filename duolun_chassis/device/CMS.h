#ifndef __CMS_H
#define __CMS_H

#include "struct_typedef.h"



typedef struct
{
	uint8_t Enable;
	uint8_t RxOpen;
	uint16_t Power;
	int16_t Electricity;
	int16_t charge_limit;//*100
	uint8_t TxOpen;
	
	uint8_t Mode;  //1 π”√µÁ»›
	
}CMS_t;

extern CMS_t CMS;

void CMS_STATUS();
void Choose_CMS();
void CMS_Referee_Send(uint16_t charge_limit, uint8_t enable);


#endif
