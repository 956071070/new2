/*! ------------------------------------------------------------------------------------------------------------------
 * @file    uart.c
 * @brief   STM32 USART串口处理函数，包括初始化配置、发送单字节/多字节函数以及环形缓冲队列实现函数等。
 */
#include <string.h>

#include "sleep.h"
#include "port.h"
#include "uart.h"

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn NVIC_Configuration()
 *
 * @brief  USART 中断嵌套（NVIC）配置函数
 *
 * input parameters
 *
 * output parameters
 *
 * @return no return value
 */
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel                   = DEBUG_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DEBUG_USART_Config()
 *
 * @brief  USART 配置函数
 *
 * input parameters
 *
 * output parameters
 *
 * @return no return value
 */
void DEBUG_USART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = DEBUG_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = DEBUG_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = DEBUG_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(DEBUG_USART, &USART_InitStructure);

    NVIC_Configuration();

    USART_ClearITPendingBit(DEBUG_USART, USART_IT_TC);
    USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(DEBUG_USART, ENABLE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DEBUG_USART_SendByte()
 *
 * @brief  通过 USART 发送单个字节
 *
 * input parameters:
 * @param USARTx        - 指定串口（1，2，3，4，5）
 * @param pdata         - 待发送的字节内容
 *
 * output parameters
 *
 * @return no return value
 */
void DEBUG_USART_SendByte(uint8_t data)
{
    USART_SendData(DEBUG_USART, data);
    while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DEBUG_USART_SendArray()
 *
 * @brief  通过 USART 发送数列
 *
 * input parameters:
 * @param USARTx        - 指定串口（1，2，3，4，5）
 * @param pdata         - 待发送的字节内容
 *
 * output parameters
 *
 * @return no return value
 */
void DEBUG_USART_SendArray(uint8_t data[], uint8_t len)
{
    int i = 0;
    for(i = 0; i < len; i++)
    {
        DEBUG_USART_SendByte(*data++);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DEBUG_USART_SendHalfWord()
 *
 * @brief  通过 USART 发送半字（2个字节）
 *
 * input parameters:
 * @param USARTx        - 指定串口（1，2，3，4，5）
 * @param pdata         - 待发送的字节内容
 *
 * output parameters
 *
 * @return no return value
 */
void DEBUG_USART_SendHalfWord(uint16_t data)
{
    uint8_t temp_h,temp_l;

    temp_h = (data&0xff00) >> 8 ;
    temp_l = data&0xff;

    USART_SendData(DEBUG_USART, temp_h);
    while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);

    USART_SendData(DEBUG_USART, temp_l);
    while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DEBUG_USART_SendString()
 *
 * @brief  通过 USART 发送字符串
 *
 * input parameters:
 * @param USARTx        - 指定串口（1，2，3，4，5）
 * @param pdata         - 要发送的字符串指针
 *
 * output parameters
 *
 * @return no return value
 */
void DEBUG_USART_SendString(char *string)
{
    do
    {
    	DEBUG_USART_SendByte(*string++);
    } while (*string != '\0');
    while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET);
}

void uart_printf(const char *format, ...)
{
	va_list list;
	va_start(list, format);

	int len = vsnprintf(0, 0, format, list);
	char *s;

	s = (char *)malloc(len + 1);
	vsprintf(s, format, list);

	DEBUG_USART_SendString(s);

	free(s);
	va_end(list);
	return;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ringBuffer_writeByte()
 *
 * @brief  此函数用于往环形缓冲队列中写入数据
 *
 * input parameters:
 * @param buffer        - 队列指针
 * @param pdata         - 将要存入的数据
 *
 * output parameters
 *
 * @return no return value
 */
void ringBuffer_writeByte(ringBuff_t* buffer, uint8_t pdata)
{
    buffer->ringBuffer[buffer->tailPosition] = pdata;

    buffer->length++;

    if(++buffer->tailPosition >= MAX_BUFF_SIZE)
        buffer->tailPosition = 0;

    if(buffer->tailPosition == buffer->headPosition)
        if(++buffer->headPosition >= MAX_BUFF_SIZE)
            buffer->headPosition = 0;
}

void ringBuffer_writeArray(ringBuff_t* buffer, uint8_t pdata[], uint16_t size)
{
    uint16_t i = 0;

    for(i = 0; i < size; ++i)
        ringBuffer_writeByte(buffer, pdata[i]);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ringBuffer_readByte()
 *
 * @brief  此函数用于显示环形缓冲队列中的数据内容，注意不要再USART中断中调用此函数。
 *
 * input parameters:
 * @param buffer        - 队列指针
 * @param pdata         - 单字节指针，用于存储取出的数据
 *
 * output parameters
 *
 * @return 队列中有数据返回‘0’，取出成功返回‘1’
 */
uint8_t ringBuffer_readByte(ringBuff_t* buffer, uint8_t* pdata)
{
    if(buffer->headPosition == buffer->tailPosition)
        return 0;
    else
        *pdata = buffer->ringBuffer[buffer->headPosition];

    buffer->length--;

    if(++buffer->headPosition >= MAX_BUFF_SIZE)
        buffer->headPosition = 0;

    return 1;
}

uint8_t ringBuffer_readArray(ringBuff_t* buffer, uint8_t pdata[], uint16_t size)
{
    uint16_t i = 0;

    if(size > buffer->length)
        return 0;

    for(i = 0; i < size; ++i)
    {
        pdata[i] = buffer->ringBuffer[buffer->headPosition];

        if(++buffer->headPosition >= MAX_BUFF_SIZE)
            buffer->headPosition = 0;

        buffer->length--;
    }

    return 1;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ringBuffer_print()
 *
 * @brief  此函数用于显示环形缓冲队列中的数据内容，注意不要再USART中断中调用此函数。
 *
 * input parameters:
 * @param buffer        - 队列指针
 *
 * output parameters
 *
 * @return no return value
 */
void ringBuffer_print(ringBuff_t* buffer)
{
	int counter = 0;
	counter = buffer->headPosition;
	while(counter != buffer->tailPosition)
	{
		DEBUG_USART_SendByte(buffer->ringBuffer[counter]);
		counter++;
		if(counter >= MAX_BUFF_SIZE)
			counter = 0;
	}
}

void ringBuffer_CLS(ringBuff_t *buffer)
{
    buffer->headPosition = buffer->tailPosition = 0;
    buffer->length = 0;
}