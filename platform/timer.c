
#include "./timer.h"

void basicTimer_configuration(void)
{
    TIM_TimeBaseInitTypeDef TIME_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel                   = BASIC_TIM_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    BASIC_TIM_APBxClock_FUN(BASIC_TIM_CLK, ENABLE);

    TIME_TimeBaseStructure.TIM_Period    = 1000; // timer period: 1ms
    TIME_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseInit(BASIC_TIM, &TIME_TimeBaseStructure);

    // TIM_ClearFlag(BASIC_TIM, TIM_FLAG_Update);
    TIM_ClearITPendingBit(BASIC_TIM, TIM_FLAG_Update);
    TIM_ITConfig(BASIC_TIM, TIM_IT_Update, ENABLE);

    TIM_Cmd(BASIC_TIM, DISABLE);
}
