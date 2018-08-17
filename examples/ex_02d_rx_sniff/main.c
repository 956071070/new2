/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   RX using SNIFF mode example code
 *
 * @attention
 *
 * Copyright 2016 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include "deca_device_api.h"
#include "deca_regs.h"
#include "lcd.h"
#include "port.h"
#include "uart.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "RX SNIFF v1.0"

/* Default communication configuration. */
static dwt_config_t config = {
    // 2,               /* Channel number. */
    // DWT_PRF_64M,     /* Pulse repetition frequency. */
    // DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    // DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    // 9,               /* TX preamble code. Used in TX only. */
    // 9,               /* RX preamble code. Used in RX only. */
    // 1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    // DWT_BR_110K,     /* Data rate. */
    // DWT_PHRMODE_STD, /* PHY header mode. */
    // (1025 + 64 - 8)  /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */

        2,              // channel
        DWT_PRF_16M,    // prf
        DWT_PLEN_1024,  // preambleLength
        DWT_PAC64,      // pacSize
        3,              // preambleCode
        3,
        1,       // non-standard SFD
        DWT_BR_6M8,    // datarate
        DWT_PHRMODE_EXT,
        (1024 + 1 + 8 - 64) //SFD timeout
};

/* SNIFF mode on/off times.
 * ON time is expressed in multiples of PAC size (with the IC adding 1 PAC automatically). So the ON time of 1 here gives 2 PAC times and, since the
 * configuration (above) specifies DWT_PAC8, we get an ON time of 2x8 symbols, or around 16 �s.
 * OFF time is expressed in multiples of 128/125 �s (~1 �s).
 * These values will lead to a roughly 50% duty-cycle, each ON and OFF phase lasting for about 16 �s. */
#define SNIFF_ON_TIME 1
#define SNIFF_OFF_TIME 16

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 1023
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;
ringBuff_t wait2sendByUart4 = {0, 0, 0, {0}};
/**
 * 串口中断服务函数.
 */
void DEBUG_USART_IRQHandler(void)
{
    if(USART_GetITStatus(DEBUG_USART, USART_FLAG_TC) != RESET)
    {
        USART_ClearITPendingBit(DEBUG_USART, USART_IT_TC);
    }
    if(//(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_ORE) != RESET) &&
       (USART_GetITStatus(DEBUG_USART, USART_FLAG_RXNE) != RESET))
    {
    	USART_ClearITPendingBit(DEBUG_USART, USART_IT_RXNE);
    }
}

/**
 * Application entry point.
 */
int main(void)
{
    int counter = 0;
    /* Start with board specific hardware init. */
    peripherals_init();

    /* Display application name on LCD. */
    lcd_display_str(APP_NAME);

    /* Reset and initialise DW1000. See NOTE 2 below.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        lcd_display_str("INIT FAILED");
        while (1)
        { };
    }
    spi_set_rate_high();

    dwt_setlnapamode(1, 1);
    dwt_setleds(DWT_LEDS_ENABLE);

    /* This is put here for testing, so that we can see the receiver ON/OFF pattern using an oscilloscope. */
    // dwt_setlnapamode(1, 1);

    /* Configure DW1000. */
    dwt_configure(&config);

    /* Configure SNIFF mode. */
    // dwt_setsniffmode(1, SNIFF_ON_TIME, SNIFF_OFF_TIME);

    /* Loop forever receiving frames. */
    while (1)
    {
        int i;
        uint8_t pdata; // 环形队列
//
//        while(ringBuffer_readByte(&wait2sendByUart4, &pdata))
//        {
//            DEBUG_USART_SendByte(pdata);
//        }

//        if(wait2sendByUart4.headPosition != wait2sendByUart4.tailPosition)
//        {
//        	while(ringBuffer_readByte(&wait2sendByUart4, &pdata))
//        		DEBUG_USART_SendByte(pdata);
//        }

        /* TESTING BREAKPOINT LOCATION #1 */

        /* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading
         * the RX buffer.
         * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
         * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
//        for (i = 0 ; i < FRAME_LEN_MAX; i++ )
//        {
//            rx_buffer[i] = 0;
//        }

        /* Activate reception immediately. See NOTE 3 below. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll until a frame is properly received or an RX error occurs. See NOTE 4 below.
         * STATUS register is 5 bytes long but we are not interested in the high byte here, so we read a more manageable 32-bits with this API call. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            /* A frame has been received, copy it to our local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
               DEBUG_USART_SendArray(&rx_buffer[1], frame_len-3);
//
//                for(i = 0; i > frame_len-3; i++)
//                {
//                	ringBuffer_writeByte(&wait2sendByUart4, rx_buffer[1+i]);
//                }

            }
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW1000 supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. In this example, LDE microcode is not loaded upon calling dwt_initialise(). This will prevent the IC from generating an RX timestamp. If
 *    time-stamping is required, DWT_LOADUCODE parameter should be used. See two-way ranging examples (e.g. examples 5a/5b).
 * 3. Manual reception activation is performed here but DW1000 offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 4. We use polled mode of operation here to keep the example as simple as possible but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW1000 User Manual for more details on "interrupts".
 * 5. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
