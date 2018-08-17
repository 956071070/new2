/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Simple TX example code
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <stdio.h>
#include <stdlib.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "sleep.h"
#include "deca_types.h"
#include "uart.h"
#include "port.h"
#include "comm.h"
#include "timer.h"
#include "ultility.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME                "SIMPLE TX v1.2"
#define TX_BUFFER_SIZE		    100
#define EUID                    0x01        // 设备独立 ID， 不可更改

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
 dwt_config_t config = {
        1,                /* Channel number. */
        DWT_PRF_64M,      /* Pulse repetition frequency. */
        DWT_PLEN_128,     /* Preamble length. Used in TX only. */
        DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
        9,                /* TX preamble code. Used in TX only. */
        9,                /* RX preamble code. Used in RX only. */
        0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_850K,      /* Data rate. */
        DWT_PHRMODE_STD,  /* PHY header mode. */
        (128 + 1 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

dwt_txconfig_t cus_tx_config = {
        0xC9,
        0x67676767
};

uint8_t tx_buff[TX_BUFFER_SIZE] = { 0 };
ringBuff_t usart4_ring_buffer = { 0, 0, 0, {0} }; // declare of ring buffer
int frameIdx = 0;   // the counter for the frame index.

uint8_t timer_status = 0;
uint8_t channel_flag = 0x00;
uint8_t temporary_recorder_id = 0;
uint8_t various_tmp = 0;

devID_t global_id = {
    EUID,  // euid
    0x01,  // panid
    0x01,  // devid
    0      // changed
};
devID_t tmp_id = {0, 0x01, 0x01, 0};

uint8_t global_instruction_frame = FRAME_INSTRUCTION_NONE;
instance_data_t instance;
run_mode_t RUN_MODE = RUN_NONE;


static uint16_t fetch_size;
static uint8_t fetch_data[200];

/* Declaration of static functions. */
static void rx_ok_cb(const dwt_cb_data_t *rx_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DEBUG_USART_IRQHandler()
 *
 * @brief  serial（UART4）interrupt function
 *
 * input parameters
 *
 * output parameters
 *
 * @return no return value
 */
void DEBUG_USART_IRQHandler(void)
{
	static uint8_t receData;
    static uint8_t instruction_flag = 0;
    static uint8_t delimiter_flag = 0;
    static uint8_t frame_delimiter[9] = {0};

    static uint8_t frame_counter = 0;

    if(USART_GetITStatus(DEBUG_USART, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(DEBUG_USART, USART_IT_TC);
    }

    if(USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(DEBUG_USART, USART_IT_RXNE);  // clear the USART_IT_RXNE flag.
        receData = USART_ReceiveData(DEBUG_USART);                // fetch the data from UART3 receive data register,
                                                              //   and store it in the 'temp' variable.
        // DEBUG_USART_SendByte(receData);

        switch(RUN_MODE)
        {
            case CONTROLER_MODE:
            {
                frame_delimiter[0] = frame_delimiter[1];
                frame_delimiter[1] = receData;

                // if(string_cmp(frame_delimiter, "##", 2))
                if(string_cmp(&frame_delimiter[0], "##", 2))
                {
                    delimiter_flag = 1;
                    frame_counter = 2;
                    return;
                }

                if(delimiter_flag)
                {
                    frame_delimiter[frame_counter++] = receData;

                    if(frame_counter == 6 && string_cmp(&frame_delimiter[2], "FND", 3))
                    {
                        frame_counter  = 2;
                        delimiter_flag = 0;

                        if(!tmp_id.changed)
                        {
                            tmp_id.panid   = frame_delimiter[5];
                            tmp_id.changed = 1;

                            global_instruction_frame = FRAME_INSTRUCTION_FND;
                        }
                    }

                    if(frame_counter == 8 && string_cmp(&frame_delimiter[2], "SET", 3))
                    {
                        frame_counter  = 2;
                        delimiter_flag = 0;

                        if(!tmp_id.changed)
                        {

                            tmp_id.euid    = frame_delimiter[5];
                            tmp_id.panid   = frame_delimiter[6];
                            tmp_id.devid   = frame_delimiter[7];
                            tmp_id.changed = 1;

                            global_instruction_frame = FRAME_INSTRUCTION_SET;
                        }
                    }

                    if(frame_counter == 7 && string_cmp(&frame_delimiter[2], "CHGI", 4))
                    {
                        frame_counter  = 2;
                        delimiter_flag = 0;

                        tmp_id.changed = 1;
                        tmp_id.euid    = frame_delimiter[6];

                        global_instruction_frame = FRAME_INSTRUCTION_CHGM;
                        various_tmp = 1;
                    }

                    if(frame_counter == 7 && string_cmp(&frame_delimiter[2], "CHGO", 4))
                    {
                        frame_counter  = 2;
                        delimiter_flag = 0;

                        tmp_id.changed = 1;
                        tmp_id.euid    = frame_delimiter[6];

                        global_instruction_frame = FRAME_INSTRUCTION_CHGM;
                        various_tmp = 0;
                    }

                    if(frame_counter > 8) frame_counter = 2;
                }
            }
            break;

            case CONFIG_MODE:
            {
            }
            break;

            case NORMAL_MODE:
            {
                if(instance.mode == SENDER)
                {
                    // static uint8 packetDelimiter[4] = { 0 };
                    // static uint8 packetDelimiterFlag = 0x00;

                    // // if(!packetDelimiterFlag)
                    // {
                    //     packetDelimiter[0] = packetDelimiter[1];
                    //     packetDelimiter[1] = packetDelimiter[2];
                    //     packetDelimiter[2] = packetDelimiter[3];
                    //     packetDelimiter[3] = receData;

                    //     if((packetDelimiterFlag != 0x01) && string_cmp(packetDelimiter, "#@$&", 4))
                    //     {
                    //         packetDelimiterFlag = 0x01;
                    //         return;
                    //     }
                    // }

                    // if(packetDelimiterFlag == 0x01)
                    // {
                    //     // global_uwb_packet_panid = receData;
                    //     switch(receData)
                    //     {
                    //         case 0x01:
                    //         {
                    //             channel_flag = 0x00;
                    //             tmp_id.devid = 0x02;
                    //         }
                    //             break;

                    //         case 0x02:

                    //             channel_flag = 0x00;
                    //             tmp_id.devid = 0x04;
                    //             break;

                    //         case 0x03:
                    //         {
                    //             channel_flag = 0x00;
                    //             tmp_id.devid = 0x08;
                    //         }
                    //             break;

                    //         case 0x04:
                    //         {
                    //             channel_flag = 0x00;
                    //             tmp_id.devid = 0xff;
                    //         }
                    //         break;

                    //         default:
                    //             break;
                    //     }
                    //     packetDelimiterFlag = 0x00;

                    //     return;
                    // }

                    ringBuffer_writeByte(&usart4_ring_buffer, receData);
                }
            }
            break;

            default:
            break;
        }
    }
}

void BASIC_TIM_IRQHandler(void)
{
    static int timerCounter = 0;
    extern uint8 temporary_recorder_id;

    if(TIM_GetITStatus(BASIC_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(BASIC_TIM , TIM_FLAG_Update);
        timerCounter++;

        if(timerCounter == 5000)
        {
            timerCounter = 0;

            temporary_recorder_id = 0;
            timer_status = 0;
        // DEBUG_USART_SendByte(0x00);

            TIM_ClearITPendingBit(BASIC_TIM , TIM_FLAG_Update);
            TIM_Cmd(BASIC_TIM, DISABLE);
        }


    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DW1000_SendByte()
 *
 * @brief  write the data waiting to send into transmit registers of the dw1000.
 *
 * input parameters:
 * @param data             - array pointer to be sent.
 * @param data_len         - the length of the data to be send.
 *
 * output parameters
 *
 * @return no return value
 */
void DW1000_SendByte(uint8_t data[], uint8_t data_len)
{
    int ret;

    dwt_writetxdata(data_len, data, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(data_len, 0, 0);   /* Zero offset in TX buffer, no ranging. */

    ret = dwt_starttx(0); // start to trasmit the data.
    if (ret == DWT_ERROR)
    {
        uart_printf("\n> start tx error.\n");
    }

    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) // wait until the SYS_STATUS_TXFRS flag is set, which means the data have completed.
    { };
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS); // clear the SYS_STATUS_TXFRS flag to facilitate to send the next frame of data.
}

void send_find_package(uint8_t mode, uint8_t tpanid)
{
    uint8_t pkg[7] = {FRAME_TYPE, FRAME_INSTRUCTION, FRAME_INSTRUCTION_FND, tpanid, 0, 0, 0};

    DW1000_DEBUG(TIP_DEBUG, (">>> start to search panid [0x%02x].\n", tpanid));
    do
    {
        DW1000_DEBUG(TIP_DEBUG, ("# Search DEVID: %d\n", pkg[4]));

        DW1000_SendByte(pkg, 7);

        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        Delay_ms(1000);
        dwt_forcetrxoff();

    } while(mode && (pkg[4]++ < 12));

    DW1000_DEBUG(TIP_DEBUG, ("<<< search end.\n\n"));
}

void send_set_package(const uint8_t euid, const uint8_t panid, const uint8_t devid)
{
    uint8_t pkg[8] = {0};

    pkg[0] = FRAME_TYPE;
    pkg[1] = FRAME_INSTRUCTION;
    pkg[2] = FRAME_INSTRUCTION_SET;
    pkg[3] = euid;
    pkg[4] = panid;
    pkg[5] = devid;
    pkg[6] = 0;
    pkg[7] = 0;

    DW1000_SendByte(pkg, 8);
}

void send_change_mode_package(uint8_t mode, uint8_t euid)
{
    uint8_t pkg[7] = {0};

    pkg[0] = FRAME_TYPE;
    pkg[1] = FRAME_INSTRUCTION;
    pkg[2] = FRAME_INSTRUCTION_CHGM;
    pkg[3] = mode;
    pkg[4] = euid;
    pkg[5] = 0;
    pkg[6] = 0;

    DW1000_SendByte(pkg, 7);
}

/* the response of FND frame. */
void send_response_2_package(void)
{
    uint8_t pkg[10] = {0};

    pkg[0] = FRAME_TYPE;
    pkg[1] = FRAME_INSTRUCTION;
    pkg[2] = FRAME_INSTRUCTION_RESP;
    pkg[3] = FRAME_INSTRUCTION_RESP_2;
    pkg[4] = EUID;
    pkg[5] = global_id.panid;
    pkg[6] = global_id.devid;
    pkg[7] = RUN_MODE;
    pkg[8] = 0;
    pkg[9] = 0;

    // DEBUG_USART_SendArray(pkg, 7);
    // dwt_setrxaftertxdelay(0);

    DW1000_SendByte(pkg, 10);

    // dwt_writetxdata(7, pkg, 0); /* Zero offset in TX buffer. */
    // dwt_writetxfctrl(7, 0, 0);   /* Zero offset in TX buffer, no ranging. */

    // dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);              // start to trasmit the data.
}

/* the response of CHGM frame. */
void send_response_3_package(uint8_t msg)
{
    uint8_t pkg[8] = {0};

    pkg[0] = FRAME_TYPE;
    pkg[1] = FRAME_INSTRUCTION;
    pkg[2] = FRAME_INSTRUCTION_RESP;
    pkg[3] = FRAME_INSTRUCTION_RESP_3;
    pkg[4] = EUID;
    pkg[5] = msg;
    pkg[6] = 0;
    pkg[7] = 0;

    DW1000_SendByte(pkg, 8);
}

/* the response of SET frame. */
void send_response_4_package(resp_4_msg_t msg)
{
    uint8_t pkg[8] = {0};

    pkg[0] = FRAME_TYPE;
    pkg[1] = FRAME_INSTRUCTION;
    pkg[2] = FRAME_INSTRUCTION_RESP;
    pkg[3] = FRAME_INSTRUCTION_RESP_4;
    pkg[4] = EUID;
    pkg[5] = msg;
    pkg[6] = 0;
    pkg[7] = 0;

    DW1000_SendByte(pkg, 8);
}

void instance_initial(instance_data_t* instance)
{
    instance->mode = INS_MODE_NONE;
    instance->state = TA_INIT;

    instance->txFrameLength = 0;

    instance->eventIdxOut   = 0;
    instance->eventIdxIn    = 0;
}

/**
 * Application entry point.
 */
int main(void)
{
    uint8_t s1switch = 0;
	uint8_t i;
    uint8_t pdata; // Temporarily store the data fetched from the ringBuff.
    char test[200] = {0};
    uint8_t tx_enable_flag = 0;
    uint8_t rx_enable_flag = 0;
    // uint32 otpPowerValue = 0;

    // timer_status = 1;

    /* peripheral initialization */
    peripherals_init();
    basicTimer_configuration();
    Systick_Init(72);

    /* Display application name on LCD. */
    // lcd_display_str(APP_NAME);

    /* Install DW1000 IRQ handler. */
    port_set_deca_isr(dwt_isr);

    /* Reset and initialise DW1000. See NOTE 2 below.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        // lcd_display_str("INIT FAILED");
        while (1)
        { };
    }
    spi_set_rate_high();

    dwt_setlnapamode(1, 1);
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW1000. See NOTE 3 below. */
    dwt_setsmarttxpower(0);
    dwt_configuretxrf(&cus_tx_config);
    dwt_configure(&config);


    /* Register RX call-back. */
    dwt_setcallbacks(NULL, &rx_ok_cb, NULL, &rx_err_cb);

    /* Enable wanted interrupts (RX good frames and RX errors). */
    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

    instance_initial(&instance);


    /* detect the dial switch to select the work mode. */
    s1switch = getSwitchStatus();
    if(s1switch & (1<<1)) // configurator
    {
        DW1000_DEBUG(TIP_DEBUG, ("> CONTROLLER MODE.\n"));
        ringBuffer_CLS(&usart4_ring_buffer);

        // dwt_setrxtimeout(0);
        // dwt_forcetrxoff();

        RUN_MODE                 = CONTROLER_MODE;
        instance.mode            = INS_MODE_NONE;
        global_instruction_frame = FRAME_INSTRUCTION_NONE;
    }
    else if(s1switch & (1<<0)) // sender
    {
        DW1000_DEBUG(TIP_DEBUG, ("> SENDER MODE.\n"));
        ringBuffer_CLS(&usart4_ring_buffer);

        dwt_setrxtimeout(0);
        dwt_forcetrxoff();

        RUN_MODE                 = NORMAL_MODE;
        instance.mode            = SENDER;
        global_instruction_frame = FRAME_INSTRUCTION_NONE;
    }
    else // receiver
    {
        DW1000_DEBUG(TIP_DEBUG, ("> RECEIVER MODE.\n"));
        ringBuffer_CLS(&usart4_ring_buffer);

        /* duty-cycle is 1/2. */
        // dwt_setsniffmode(1, 8, 16);

        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        RUN_MODE                 = NORMAL_MODE;
        instance.mode            = RECEIVER;
        global_instruction_frame = FRAME_INSTRUCTION_NONE;
    }

    while(1)
    {
        switch(RUN_MODE)
        {
            case CONTROLER_MODE:
            {
                // DW1000_DEBUG(TIP_DEBUG, ("> In Controller mode.\n"));
                switch(global_instruction_frame)
                {
                    case FRAME_INSTRUCTION_FND:
                    {
                        DW1000_DEBUG(TIP_DEBUG, ("> Finding the device ...\n"));
                        global_instruction_frame = FRAME_INSTRUCTION_NONE;

                        DW1000_DEBUG(TIP_DEBUG, ("> Send the FND package.\n"));

                        /* turn on the receiver after sending is complete.  */
                        // dwt_setrxtimeout(20000);
                        // dwt_setrxaftertxdelay(1);

                        if(tmp_id.changed)
                        {
                            tmp_id.changed = 0;
                            send_find_package(1, tmp_id.panid);
                        }

                        // dwt_setrxtimeout(0);
                        // dwt_rxenable(DWT_START_RX_IMMEDIATE);

                        // DW1000_DEBUG(TIP_DEBUG, ("> open the receiver, wait 5 seconds.\n"));
                        // Delay_ms(500);

                        // dwt_forcetrxoff();
                        // DW1000_DEBUG(TIP_DEBUG, ("> close the receiver.\n"));
                    }
                    break;

                    case FRAME_INSTRUCTION_SET:
                    {
                        DW1000_DEBUG(TIP_DEBUG, ("> set the device.\n"));
                        global_instruction_frame = FRAME_INSTRUCTION_NONE;

                        DW1000_DEBUG(TIP_DEBUG, ("> send the SET package.\n"));
                        if(tmp_id.changed)
                        {
                            send_set_package(tmp_id.euid, tmp_id.panid, tmp_id.devid);
                            tmp_id.changed = 0;
                        }
                        dwt_setrxtimeout(0);
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);

                        DW1000_DEBUG(TIP_DEBUG, ("> open the receiver, wait 200ms.\n"));

                        Delay_ms(200);
                        dwt_forcetrxoff();

                        DW1000_DEBUG(TIP_DEBUG, ("> close the receiver.\n"));
                    }
                    break;

                    case FRAME_INSTRUCTION_CHGM:
                    {
                        global_instruction_frame = FRAME_INSTRUCTION_NONE;
                        DW1000_DEBUG(TIP_DEBUG, ("> modify the mode of the target device.\n"));

                        /* turn on the receiver after sending is complete.  */
                        // dwt_setrxtimeout(0);
                        // dwt_setrxaftertxdelay(0);

                        if(tmp_id.changed == 1)
                        {
                            if(various_tmp == 1)
                            {
                                DW1000_DEBUG(TIP_DEBUG, ("> sended the CHGM[ FRAME_INSTRUCTION_CHGM_IN ] package, and open the receiver.\n"));
                                send_change_mode_package(FRAME_INSTRUCTION_CHGM_IN, tmp_id.euid);
                            }
                            else
                            {
                                DW1000_DEBUG(TIP_DEBUG, ("> sended the CHGM[ FRAME_INSTRUCTION_CHGM_OUT ] package, and open the receiver.\n"));
                                send_change_mode_package(FRAME_INSTRUCTION_CHGM_OUT, tmp_id.euid);
                            }

                            tmp_id.changed = 0;
                            various_tmp    = 0;
                        }

                        dwt_rxenable(DWT_START_RX_IMMEDIATE);

                        Delay_ms(5000);

                        dwt_forcetrxoff();

                        DW1000_DEBUG(TIP_DEBUG, ("> close the receiver.\n"));
                    }

                    default:
                    break;
                }

                // while(ringBuffer_readByte(&usart4_ring_buffer, &pdata))
                // {
                //     DEBUG_USART_SendByte(pdata);
                // }
            }
            break;

            case CONFIG_MODE:
            {
                // uart_printf("CONFIG_MODE\n");
                switch(global_instruction_frame)
                {
                    // receiced the FND frame.
                    case FRAME_INSTRUCTION_FND:
                    {
                        DW1000_DEBUG(TIP_DEBUG, ("> received a FND package.\n"));
                        global_instruction_frame = FRAME_INSTRUCTION_NONE;

                        dwt_forcetrxoff();

                        DW1000_DEBUG(TIP_DEBUG, ("> send the RESP-2 package.\n"));
                        send_response_2_package(); // auto open the receiver

                        dwt_setrxtimeout(0);
                        dwt_rxenable(DWT_START_TX_IMMEDIATE);
                    }
                    break;

                    // received the SET frame.
                    case FRAME_INSTRUCTION_SET:
                    {

                        DW1000_DEBUG(TIP_DEBUG, ("> received a SET package.\n\tnew panid: %x\n\tnew devid: %x\n",
                                                    tmp_id.panid, tmp_id.devid));
                        global_instruction_frame = FRAME_INSTRUCTION_NONE;

                        tmp_id.changed = 0;

                        dwt_forcetrxoff();

                        if(tmp_id.panid == global_id.panid && tmp_id.devid == global_id.devid)
                        {
                            send_response_4_package(RESP4_PARAM_EQUAL_TO_THE_OLD_ID);
                            uart_printf("\n> RESP4_EQUAL_TO_THE_OLD_ID.\n");

                            DW1000_DEBUG(TIP_DEBUG, ("> the new id is equal to the old id.\n> send RESP-4[RESP4_PARAM_EQUAL_TO_THE_OLD_ID] package.\n"));
                        }
                        else
                        {
                            global_id.panid = tmp_id.panid;
                            global_id.devid = tmp_id.devid;

                            send_response_4_package(RESP4_PARAM_SETTING_SUCCESS);

                            DW1000_DEBUG(TIP_DEBUG, ("> the new id is setted successfully.\n> send the RESP-4[RESP4_PARAM_SETTING_SUCCESS] package.\n"));
                        }

                        dwt_setrxtimeout(0);
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    }
                    break;

                    case FRAME_INSTRUCTION_CHGM:
                    {
                        global_instruction_frame = FRAME_INSTRUCTION_NONE;

                        if(various_tmp == 1)
                        {
                            various_tmp = 0;

                            dwt_forcetrxoff();
                            send_response_3_package(RESP3_IN_CONFIG);

                            DW1000_DEBUG(TIP_DEBUG, ("> I'm in Config mode, no need to access configuration mode.\n"));

                            dwt_setrxtimeout(0);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);

                        }
                        else if(various_tmp == 0)
                        {
                            various_tmp = 0;
                            RUN_MODE = NORMAL_MODE;

                            dwt_forcetrxoff();
                            send_response_3_package(RESP3_DROP_OUT_CONFIG);

                            DW1000_DEBUG(TIP_DEBUG, ("> I'm in Config mode, turnning into the Normal mode.\n"));

                            dwt_setrxtimeout(0);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);

                            continue;
                        }
                        else
                        {

                        }
                    }
                    break;

                    default:
                    break;
                }
            }
            break;

            case NORMAL_MODE:
            {
                if(instance.mode == RECEIVER)
                {
                    if(global_instruction_frame == FRAME_INSTRUCTION_FND)
                    {
                        static unsigned long pre_time, tmp = 0;
                        global_instruction_frame = FRAME_INSTRUCTION_NONE;

                        dwt_forcetrxoff();

                        pre_time = portGetTickCount();
                        pre_time += portGetTickCount() % 100;

                        DW1000_DEBUG(TIP_DEBUG, ("## pre_time: %d\n", pre_time));
                        do
                        {
                            tmp = portGetTickCount();
                        } while(tmp < pre_time);

                        send_response_2_package();

                        DW1000_DEBUG(TIP_DEBUG, ("> send RESP-2 package, including the ID.\n"));

                        dwt_setrxtimeout(0);
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    }

                    if(global_instruction_frame == FRAME_INSTRUCTION_CHGM)
                    {
                        global_instruction_frame = FRAME_INSTRUCTION_NONE;

                        if(various_tmp == FRAME_INSTRUCTION_CHGM_IN)
                        {
                            various_tmp = 0;
                            RUN_MODE = CONFIG_MODE;

                            dwt_forcetrxoff();
                            send_response_3_package(RESP3_TURN_INTO_CONFIG);

                            DW1000_DEBUG(TIP_DEBUG, ("> I'm in Normal mode, turnning into the Config mode.\n"));

                            dwt_setrxtimeout(0);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);

                            continue;
                        }
                        else if(various_tmp == FRAME_INSTRUCTION_CHGM_OUT)
                        {
                            various_tmp = 0;

                            dwt_forcetrxoff();
                            send_response_3_package(RESP3_IN_DATA);

                            DW1000_DEBUG(TIP_DEBUG, ("> I'm in Normal mode, no need to access Normal mode.\n> send the RESP-3[ RESP3_IN_DATA ] package.\n"));

                            dwt_setrxtimeout(0);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }
                        else
                        {
                            various_tmp = 0;

                            DW1000_DEBUG(TIP_DEBUG, ("> You are a amazing man.\n"));
                        }

                        // dwt_setrxtimeout(0);
                        // dwt_rxenable(DWT_START_RX_IMMEDIATE);

                        continue;
                    }
                }

                if(usart4_ring_buffer.headPosition != usart4_ring_buffer.tailPosition)
                {
                    switch(instance.mode)
                    {
                        case SENDER:
                        {
                            frameIdx = 0;
                            tx_buff[frameIdx++] = FRAME_TYPE;
                            tx_buff[frameIdx++] = FRAME_DATA;
                            tx_buff[frameIdx++] = global_id.euid;  // recorder euid
                            tx_buff[frameIdx++] = tmp_id.panid;    // player panid
                            tx_buff[frameIdx++] = tmp_id.devid;    // player devid

                            while(ringBuffer_readByte(&usart4_ring_buffer, &pdata)) // fetch the data from ringbuffer and store it in the 'pdata' variable.
                            {
                                tx_buff[frameIdx++] = pdata;
                                if(frameIdx > TX_BUFFER_SIZE-2) // set the upper bound for the numbers of the data which is fetched from ringBuffer.
                                    break;
                            }
                            tx_buff[frameIdx++] = 0;             // this two bytes is used to store the crc value, it is calculated by the dw1000.
                            tx_buff[frameIdx++] = 0;
                            DW1000_SendByte(tx_buff, frameIdx);  // call the function to send data stored in tx_buff[].

                            // DEBUG_USART_SendArray(tx_buff, frameIdx);
                        }
                        break;

                        case RECEIVER:
                        {
                            // while(ringBuffer_readByte(&usart4_ring_buffer, &pdata))
                            //     DEBUG_USART_SendByte(pdata);

                            if(usart4_ring_buffer.length != 0)
                            {

                                fetch_size = usart4_ring_buffer.length;
                                if(fetch_size > 20)
                                    fetch_size = 20;

                                ringBuffer_readArray(&usart4_ring_buffer, fetch_data, fetch_size);
                                DEBUG_USART_SendArray(fetch_data, fetch_size);
                            }
                        }
                        break;

                        default:
                        break;
                    }
                }

                switch(channel_flag)
                {
                    case 0x04:
                    {
                        dwt_config_t  config_4 = {
                                4,                /* Channel number. */
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

                        dwt_configure(&config_4);
                        channel_flag = 0x00;
                    }
                    break;

                    case 0x05:
                    {
                        dwt_config_t config_5 = {
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

                        dwt_configure(&config_5);
                        channel_flag = 0x00;
                    }
                    break;

                    default:
                    break;
                }
            }
            break;

            default:
            break;
        }

        // Delay_ms(400);

        // {
        //     // int   done    = INST_NOT_DONE_YET;
        //     // uint8 message = instance_peekevent();

        //     // while(done == INST_NOT_DONE_YET)
        //     // {
        //     //     done = app_event_process(&instance, message);
        //     //     message = 0;
        //     // }
        // }

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
static void rx_ok_cb(const dwt_cb_data_t *rx_data)
{
	int i = 0;
    uint8 rxd_event = 0;

    dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);

    event_data_t dw_event;

    // dw_event.systickTimestamp = portGetTickCount();

    // if(rx_data->event == DWT_SIG_RX_OKAY)
    {
        dw_event.rxFrameLength = rx_data->datalength;
        // rxd_event = ((rx_data->fctrl[0] == 0x41) && (rx_data->fctrl[1] == 0x88)) ? DWT_SIG_RX_OKAY : DWT_SIG_RX_UNKNOWN;
        rxd_event = DWT_SIG_RX_OKAY;
    }

    dwt_readrxdata((uint8 *)&dw_event.frame[0], dw_event.rxFrameLength, 0);

    dw_event.type = 0;
    if(rxd_event == DWT_SIG_RX_OKAY)
    {
        switch(dw_event.frame[1])
        {
            case RTLS_SENDER_POLL:
            {
                if(instance.mode == SENDER)
                {
                    return;
                }

                 if(instance.mode == RECEIVER)
                {
                    uint32 resp_tx_time;
                    int ret;

                    /* Retrieve poll reception timestamp. */
                    dw_event.poll_rx_ts = get_rx_timestamp_u64();

                    /* Compute final message transmission time. See NOTE 7 below. */
                    resp_tx_time = (dw_event.poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);

                    /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
                    dw_event.resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
                }
            }
            break; // end of RTLS_SENDER_POLL

            case RTLS_RECEIVER_RESP:
            {
                if(instance.mode == RECEIVER)
                    return;
            }
            break; // end of RTLS_RECEIVER_RESP

            case RTLS_SENDER_FINAL:
            {
                if(instance.mode == SENDER)
                    return;
            }
            break; // end of RTLS_SENDER_FINAL

            case FRAME_INSTRUCTION:
            {
                switch(RUN_MODE)
                {
                    case CONTROLER_MODE:
                    {
                        if(dw_event.frame[2] != FRAME_INSTRUCTION_RESP)
                            return;
                        DW1000_DEBUG(TIP_DEBUG, ("> receive a package [code : 0x%02x]\n", dw_event.frame[3]));

                        // received the instruction frame, which code is 0x01 (RESP-2)
                        if(dw_event.frame[3] == FRAME_INSTRUCTION_RESP_2)
                        {
                            DW1000_DEBUG(TIP_DEBUG, ("> receive a RESP-2 package.\n"));
                            DW1000_DEBUG(PARAM_DEBUG, ("> device euid: 0x%02x\n\tpanid: 0x%02x\n\tdevid: 0x%02x\n\tmode: %d\n",
                                            dw_event.frame[4], dw_event.frame[5], dw_event.frame[6], dw_event.frame[7]));

                            // for(i = 0; i < dw_event.rxFrameLength; i++)
                            // {
                            //     ringBuffer_writeByte(&usart4_ring_buffer, dw_event.frame[i]);
                            // }
                            return;
                        }

                        // received the instruction frame, which code is 0x02 (RESP-3)
                        if(dw_event.frame[3] == FRAME_INSTRUCTION_RESP_3)
                        {
                            DW1000_DEBUG(TIP_DEBUG, ("> receive a RESP-3 package.\n"));
                            switch(dw_event.frame[5])
                            {
                                case RESP3_SWITCH_MODE_ERR:
                                    DW1000_DEBUG(TIP_DEBUG, ("> Device(0x%02x): RESP3_SWITCH_MODE_ERR\n",
                                                                dw_event.frame[4]));
                                    break;
                                case RESP3_TURN_INTO_CONFIG:
                                    DW1000_DEBUG(TIP_DEBUG, ("> Device(0x%02x): RESP3_TURN_INTO_CONFIG\n",
                                                                dw_event.frame[4]));
                                    break;
                                case RESP3_DROP_OUT_CONFIG:
                                    DW1000_DEBUG(TIP_DEBUG, ("> Device(0x%02x): RESP3_DROP_OUT_CONFIG\n",
                                                                dw_event.frame[4]));
                                    break;
                                case RESP3_IN_DATA:
                                    DW1000_DEBUG(TIP_DEBUG, ("> Device(0x%02x): RESP3_IN_DATA\n",
                                                                dw_event.frame[4]));
                                    break;
                                case RESP3_IN_CONFIG:
                                    DW1000_DEBUG(TIP_DEBUG, ("> Device(0x%02x): RESP3_IN_CONFIG\n",
                                                                dw_event.frame[4]));
                                    break;
                                default:
                                    break;
                            }
                            return;
                        }

                        // received the instruction frame, which code is 0x03 (RESP-4)
                        if(dw_event.frame[3] == FRAME_INSTRUCTION_RESP_4)
                        {
                            switch(dw_event.frame[5])
                            {
                                case RESP4_PARAM_SETTING_ERR:
                                    DW1000_DEBUG(TIP_DEBUG, ("> Device(0x%02x): RESP4_PARAM_SETTING_ERR\n",
                                                                dw_event.frame[4]));
                                    break;
                                case RESP4_PARAM_SETTING_SUCCESS:
                                    DW1000_DEBUG(TIP_DEBUG, ("> Device(0x%02x): RESP4_PARAM_SETTING_SUCCESS\n",
                                                                dw_event.frame[4]));
                                    break;
                                case RESP4_PARAM_EQUAL_TO_THE_OLD_ID:
                                    DW1000_DEBUG(TIP_DEBUG, ("> Device(0x%02x): RESP4_PARAM_EQUAL_TO_THE_OLD_ID\n",
                                                                dw_event.frame[4]));
                                    break;
                                default:
                                    break;
                            }
                            return;
                        }
                    }
                    break;

                    case CONFIG_MODE:
                    {
                        switch(dw_event.frame[2])
                        {
                            case FRAME_INSTRUCTION_FND:
                            {
                                if((dw_event.frame[3] != global_id.panid)
                                    || (dw_event.frame[4] != global_id.devid))
                                    return;

                                global_instruction_frame = FRAME_INSTRUCTION_FND;
                            }
                            break;

                            case FRAME_INSTRUCTION_SET:
                            {
                                if(dw_event.frame[3] != global_id.euid)
                                    return;

                                if(dw_event.rxFrameLength != 8)
                                {
                                    DW1000_DEBUG(TIP_DEBUG, ("> receive a SET frame, but which is wrong.\n"));
                                    return;
                                }

                                if(!tmp_id.changed)
                                {
                                    tmp_id.euid    = EUID;
                                    tmp_id.panid   = dw_event.frame[4];
                                    tmp_id.devid   = dw_event.frame[5];
                                    tmp_id.changed = 1;

                                    global_instruction_frame = FRAME_INSTRUCTION_SET;
                                }
                            }
                            break;

                            case FRAME_INSTRUCTION_CHGM:
                            {
                                if(dw_event.frame[4] != global_id.euid)
                                    return;

                                global_instruction_frame = FRAME_INSTRUCTION_CHGM;

                                if(dw_event.frame[3] == FRAME_INSTRUCTION_CHGM_IN)
                                {
                                    various_tmp = 1;
                                }

                                if(dw_event.frame[3] == FRAME_INSTRUCTION_CHGM_OUT)
                                {
                                    various_tmp = 0;
                                }

                                DW1000_DEBUG(TIP_DEBUG, ("> I'm in config mode, and receive a CHGM package.\n"));
                            }
                            break;

                            default:
                            break;
                        }
                    }
                    break;

                    case NORMAL_MODE:
                    {
                        if(instance.mode != RECEIVER)
                            return;

                        // if(dw_event.frame[2] != FRAME_INSTRUCTION_FND)
                        //     return;

                        // global_instruction_frame = FRAME_INSTRUCTION_FND;
                        switch(dw_event.frame[2])
                        {
                            case FRAME_INSTRUCTION_FND:
                            {
                                if((dw_event.frame[3] != global_id.panid)
                                    || (dw_event.frame[4] != global_id.devid))
                                    return;

                                global_instruction_frame = FRAME_INSTRUCTION_FND;

                                DW1000_DEBUG(TIP_DEBUG, ("> receiced a FND package.\n"));

                            }
                            break;

                            case FRAME_INSTRUCTION_CHGM:
                            {
                                if(dw_event.frame[4] != global_id.euid)
                                    return;

                                if(dw_event.frame[3] == FRAME_INSTRUCTION_CHGM_IN)
                                {
                                    global_instruction_frame = FRAME_INSTRUCTION_CHGM;
                                    various_tmp = FRAME_INSTRUCTION_CHGM_IN;

                                    DW1000_DEBUG(TIP_DEBUG, ("> received a CHGM[ FRAME_INSTRUCTION_CHGM_IN ] package.\n"));
                                }
                                else if(dw_event.frame[3] == FRAME_INSTRUCTION_CHGM_OUT)
                                {
                                    global_instruction_frame = FRAME_INSTRUCTION_CHGM;
                                    various_tmp = FRAME_INSTRUCTION_CHGM_OUT;

                                    DW1000_DEBUG(TIP_DEBUG, ("> received a CHGM[ FRAME_INSTRUCTION_CHGM_OUT ] package.\n"));
                                }
                                else
                                {
                                    DW1000_DEBUG(TIP_DEBUG, ("> received a anonymous package.\n"));
                                }

                            }
                            break;

                            default:
                            break;
                        }
                    }
                    break;

                    default:
                    break;
                }
                return;
            }
            break;

            case FRAME_DATA:
            {
                // uart_printf("receive a data frame.\n euid:%X, panid:%X, devid:%X\n",
                //                 dw_event.frame[2], dw_event.frame[3], dw_event.frame[4]);
                // if(   (dw_event.frame[2] != DEVICE_PANID)
                //    || (dw_event.frame[3] != DEVICE_DEVID)
                //   )
                //     return;

                if( (dw_event.frame[3] & global_id.panid) == 0
                 || (dw_event.frame[4] & global_id.devid) == 0
                )
                    return;

                // uart_printf("pass\n");
                // if(timer_status == 1)
                //     return;

                // if(temporary_recorder_id == 0)
                //     temporary_recorder_id = dw_event.frame[2];

                // if(temporary_recorder_id != dw_event.frame[2])
                // {
                //     timer_status = 1;
                //     TIM_ClearFlag(BASIC_TIM, TIM_FLAG_Update);
                //     TIM_Cmd(BASIC_TIM, ENABLE);
                //     return;
                // }

                {
                    int i = 0;
                    for(i = 0; i < dw_event.rxFrameLength - 7; i++)
                    {
                        ringBuffer_writeByte(&usart4_ring_buffer, dw_event.frame[5 + i]);
                        // DEBUG_USART_SendByte(dw_event.frame[5 + i]);
                    }
                }

                return;
            }
            break;

            default:
            break;
        }

        dw_event.type = 1;
        instance_putevent(dw_event, rxd_event);
        instance.rxMsgCount++;
    }
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
