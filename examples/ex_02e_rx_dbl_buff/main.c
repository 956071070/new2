/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   RX using double buffering example code
 *
 *           This example keeps listening for any incoming frames, storing in a local buffer any frame received before going back to listening. This
 *           examples activates interrupt handling and the double buffering feature of the DW1000 (but automatic RX re-enabling is not supported).
 *           Frame processing and manual RX re-enabling are performed in the RX good frame callback.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
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
#include "platform/comm.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "RX DBL BUFF v1.0"

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
        5,                /* Channel number. */
        DWT_PRF_16M,      /* Pulse repetition frequency. */
        DWT_PLEN_128,     /* Preamble length. Used in TX only. */
        DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
        4,                /* TX preamble code. Used in TX only. */
        4,                /* RX preamble code. Used in RX only. */
        0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_6M8,      /* Data rate. */
        DWT_PHRMODE_STD,  /* PHY header mode. */
        (129 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX   127
#define NETWORK_ID        0x01
static uint8 rx_buffer[FRAME_LEN_MAX];

ringBuff_t usart4_receive_buffer = {0, 0, 0, {0}};

/* SNIFF mode on/off times.
 * ON time is expressed in multiples of PAC size (with the IC adding 1 PAC automatically). So the ON time of 1 here gives 2 PAC times and, since the
 * configuration (above) specifies DWT_PAC8, we get an ON time of 2x8 symbols, or around 16 �s.
 * OFF time is expressed in multiples of 128/125 �s (~1 �s).
 * These values will lead to a roughly 50% duty-cycle, each ON and OFF phase lasting for about 16 �s. */
#define SNIFF_ON_TIME 1
#define SNIFF_OFF_TIME 16

/* Declaration of static functions. */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DEBUG_USART_IRQHandler()
 *
 * @brief  串口（UART4）中断函数
 *
 * input parameters
 *
 * output parameters
 *
 * @return no return value
 */
void DEBUG_USART_IRQHandler(void)
{
	uint8_t temp = 0;

    if(USART_GetITStatus(DEBUG_USART, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(DEBUG_USART, USART_IT_TC);
    }

    if(USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)
    {
    	USART_ClearITPendingBit(DEBUG_USART, USART_IT_RXNE); // clear the USART_IT_RXNE flag.
    	temp = USART_ReceiveData(DEBUG_USART);
    }
}

/**
 * Application entry point.
 */
int main(void)
{
	uint8_t pdata;
    /* Start with board specific hardware init. */
    peripherals_init();

    /* Display application name on LCD. */
    lcd_display_str(APP_NAME);

    /* Install DW1000 IRQ handler. */
    port_set_deca_isr(dwt_isr);

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

    /* Configure DW1000. */
    dwt_configure(&config);

    /* Activate double buffering. */
    dwt_setdblrxbuffmode(1);

    dwt_setsniffmode(1, SNIFF_ON_TIME, SNIFF_OFF_TIME);

    /* Register RX call-back. */
    dwt_setcallbacks(NULL, &rx_ok_cb, NULL, &rx_err_cb);

    /* Enable wanted interrupts (RX good frames and RX errors). */
    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

    /* Activate reception immediately. See NOTE 3 below. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Loop forever receiving frames. See NOTE 4 below. */
    while (1)
    {
    	while(ringBuffer_readByte(&usart4_receive_buffer, &pdata)) // fetch the data stored in 'usart4_receive_buffer' buffer.
    	{
    		DEBUG_USART_SendByte(pdata);  // send the data to other device by UART4.
    	}
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	int i = 0;
    /* Perform manual RX re-enabling. See NOTE 5 below. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);

    /* TESTING BREAKPOINT LOCATION #1 */

    /* A frame has been received, copy it to our local buffer. */
    if (cb_data->datalength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
        if(rx_buffer[1] != NETWORK_ID)
            return;
        for(i = 0; i < cb_data->datalength; i++)
        {
            // write the data stored in rx_buff[] into the ringBuffer.
        	ringBuffer_writeByte(&usart4_receive_buffer, rx_buffer[i]);
        }

//        DEBUG_USART_SendArray(&rx_buffer[1], cb_data->datalength-3);
    }

    /* TESTING BREAKPOINT LOCATION #2 */
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    /* Re-activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* TESTING BREAKPOINT LOCATION #3 */
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
 * 4. There is nothing to do in the loop here as frame reception and RX re-enabling is handled by the callbacks. In a less trivial real-world
 *    application the RX data callback would generally signal the reception event to some background protocol layer to further process each RX frame.
 * 5. When using double buffering, RX can be re-enabled before reading all the frame data as this is precisely the purpose of having two buffers. All
 *    the registers needed to process the received frame are also double buffered with the exception of the Accumulator CIR memory and the LDE
 *    threshold (accessed when calling dwt_readdiagnostics). In an actual application where these values might be needed for any processing or
 *    diagnostics purpose, they would have to be read before RX re-enabling is performed so that they are not corrupted by a frame being received
 *    while they are being read. Typically, in this example, any such diagnostic data access would be done at the very beginning of the rx_ok_cb
 *    function.
 * 6. A real application might get an operating system (OS) buffer for this data reading and then pass the buffer onto a queue into the next layer
      of processing task via an appropriate OS call.
 * 7. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
