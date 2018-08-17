#ifndef _ULT_H
#define _ULT_H

#include "stm32f10x.h"


void Systick_Init (uint8_t SYSCLK);
void Delay_s( uint32_t time_s );
void Delay_us(uint32_t time_us);
void Delay_ms(uint32_t time_ms);

#endif
