#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

#define BASIC_TIM                           TIM6
#define BASIC_TIM_APBxClock_FUN             RCC_APB1PeriphClockCmd
#define BASIC_TIM_CLK                       RCC_APB1Periph_TIM6
#define BASIC_TIM_IRQ                       TIM6_IRQn
#define BASIC_TIM_IRQHandler                TIM6_IRQHandler

void basicTimer_configuration(void);

#endif
