#ifndef __UART_H
#define __UART_H

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "stm32f10x.h"
/*! ------------------------------------------------------------------------------------------------------------------
 * UART4串口变量预定义
 */
#define DEBUG_USART                     UART4
#define DEBUG_USART_CLK                 RCC_APB1Periph_UART4
#define DEBUG_USART_APBxClkCmd          RCC_APB1PeriphClockCmd

// #define DEBUG_USART_BAUDRATE
#define DEBUG_USART_BAUDRATE            460800

#define DEBUG_USART_GPIO_CLK            (RCC_APB2Periph_GPIOC)
#define DEBUG_USART_GPIO_APBxClkCmd     RCC_APB2PeriphClockCmd

#define DEBUG_USART_TX_GPIO_PORT        GPIOC
#define DEBUG_USART_TX_GPIO_PIN         GPIO_Pin_10
#define DEBUG_USART_RX_GPIO_PORT        GPIOC
#define DEBUG_USART_RX_GPIO_PIN         GPIO_Pin_11

#define DEBUG_USART_IRQ                 UART4_IRQn
#define DEBUG_USART_IRQHandler          UART4_IRQHandler

/*! ------------------------------------------------------------------------------------------------------------------
 * 环形缓冲队列数据结构声明
 */
#define MAX_BUFF_SIZE                   5000
typedef struct
{
    int headPosition;
    int tailPosition;
    int length;
    uint8_t ringBuffer[MAX_BUFF_SIZE];
} ringBuff_t;

/*! ------------------------------------------------------------------------------------------------------------------
 * 串口通用函数
 */
void DEBUG_USART_Config(void);
void DEBUG_USART_SendByte(uint8_t data);
void DEBUG_USART_SendArray(uint8_t data[], uint8_t len);
void DEBUG_USART_SendHalfWord(uint16_t data);
void DEBUG_USART_SendString(char *string);
void uart_printf(const char *format, ...);

/*! ------------------------------------------------------------------------------------------------------------------
 * 串口环形缓冲队列函数声明
 */
void ringBuffer_writeByte(ringBuff_t* buffer, uint8_t pdata);
void ringBuffer_writeArray(ringBuff_t* buffer, uint8_t pdata[], uint16_t size);
uint8_t ringBuffer_readByte(ringBuff_t* buffer, uint8_t* pdata);
uint8_t ringBuffer_readArray(ringBuff_t* buffer, uint8_t pdata[], uint16_t size);
void ringBuffer_print(ringBuff_t* buffer);
void ringBuffer_CLS(ringBuff_t *buffer);

#endif
