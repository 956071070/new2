#include "ultility.h"


static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数

//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void Systick_Init (uint8_t SYSCLK)	// 定义SYSCLK为系统AHB时钟
{
	SysTick->CTRL&=0xfffffffb;	// SysTick的CTRL(控制及状态寄存器)的bit2(CLKSOURCE)置0,选择外部时钟(STCLK);若为1选择内部时钟(FCLK)
	fac_us=SYSCLK/8;		    		// 在STM32中SysTick以HCLK(AHB时钟)或HCLK/8作为运行时钟
	fac_ms=(u16)fac_us*1000;
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,time_ms<=1864
//当需要延时s以上时，请调用Delay_s函数
void Delay_s( uint32_t time_s )
{
  for(;time_s>0;time_s--)
    Delay_ms(1000);
}
void Delay_ms( uint32_t time_ms )
{
	u32 temp;
	SysTick->LOAD=(u32)time_ms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达
	// SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器
}
//延时nus
//nus为要延时的us数.
void Delay_us( uint32_t time_us )
{
	u32 temp;
	SysTick->LOAD=time_us*fac_us; //时间加载
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器
}