
#include "comm.h"

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
 *        response message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void resp_msg_set_ts(uint8 *ts_field, const uint64_t ts)
{
    int i;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        ts_field[i] = (ts >> (i * 8)) & 0xFF;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resp_msg_get_ts()
 *
 * @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
 *        least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to get
 *         ts  timestamp value
 *
 * @return none
 */
void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_set_ts(uint8 *ts_field, uint64_t ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

int app_event_process(instance_data_t* dw_instance, int message)
{
    switch(dw_instance->state)
    {
        case TA_INIT:
        {
            if(dw_instance->mode == SENDER)
            {
                dw_instance->state = TA_TX_WAIT_SEND;
            }
            else if(dw_instance->mode == RECEIVER)
            {
                dw_instance->state = TA_RX_WAIT;
            }
        }
        break; // end of TA_INIT

        case TA_TX_WAIT:
        {
            dw_instance->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
        }
        break; // end of TA_TX_WAIT

        case TA_TX_WAIT_SEND:
        {
            dw_instance->msg_f.frame[0] = 0xC5;
            dw_instance->msg_f.frame[1] = (instance.mode == SENDER) ? RTLS_SENDER_POLL : RTLS_NO_VALID;
            dw_instance->txFrameLength = 4; // = 11 Bytes
            dwt_writetxdata(dw_instance->txFrameLength, (uint8 *)&dw_instance->msg_f, 0);
            // dwt_setrxaftertxdelay(0);
            dwt_writetxfctrl(dw_instance->txFrameLength, 0, 0);
            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

            dw_instance->state         = TA_RX_WAIT;
            dw_instance->previousState = TA_TX_WAIT_SEND;
            dw_instance->done          = INST_DONE_WAIT_FOR_NEXT_EVENT;

        }
        break; // end of TA_TX_WAIT_SEND

        case TA_RX_WAIT:
        {
            dwt_rxenable(DWT_START_RX_IMMEDIATE) ;  // turn RX on, without delay

            dw_instance->state = TA_RX_WAIT_DATA;   // let this state handle it
            dw_instance->done  = INST_DONE_WAIT_FOR_NEXT_EVENT;

            if(message == 0)
                break;
        }
        // break; // end of TA_RX_WAIT

        case TA_RX_WAIT_DATA:
        {
            switch(message)
            {
                case DWT_SIG_RX_OKAY:
                {
                    event_data_t* dw_event = instance_getevent();
                    // uint8 srcAddr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
					// uint8 dstAddr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
                    uint8 fn_code = 0;
                    uint8 *messageData;

                    // memcpy(&srcAddr[0], &dw_event->frame[7], 2);
                    // memcpy(&dstAddr[0], &dw_event->frame[5], 2);
                    fn_code = dw_event->frame[1];
                    messageData = &dw_event->frame[2];

                    switch(fn_code)
                    {
                        case RTLS_SENDER_POLL: // the receiver fetch this frame.
                        {
                            uint32 resp_tx_time;
                            int ret;

                            dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                            dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                            dw_instance->msg_f.frame[0] = 0xC5;
                            dw_instance->msg_f.frame[1] = RTLS_RECEIVER_RESP;
                            resp_msg_set_ts((uint8 *)&dw_instance->msg_f.frame[2], dw_event->poll_rx_ts);
                            resp_msg_set_ts((uint8 *)&dw_instance->msg_f.frame[6], dw_event->resp_tx_ts);

                            dwt_writetxdata(11, (uint8 *)&dw_instance->msg_f, 0);
                            dwt_writetxfctrl(11, 0, 1);
                            ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                            if (ret == DWT_ERROR)
                            {
                                break;
                            }

                            dw_instance->state         = TA_RX_WAIT;
                            dw_instance->previousState = TA_RX_WAIT_DATA;
                            dw_instance->done          = INST_DONE;

                        }
                        break;

                        case RTLS_RECEIVER_RESP: // the sender fetch this frame.
                        {
                            uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                            int32 rtd_init, rtd_resp;
                            char dist_str[30] = {0};

                            poll_tx_ts = dwt_readtxtimestamplo32();
                            resp_rx_ts = dwt_readrxtimestamplo32();

                            resp_msg_get_ts(&messageData[0], &poll_rx_ts );
                            resp_msg_get_ts(&messageData[4], &resp_tx_ts );

                            rtd_init = resp_rx_ts - poll_tx_ts;
                            rtd_resp = resp_tx_ts - poll_rx_ts;

                            tof = ((rtd_init - rtd_resp) / 2.0) * DWT_TIME_UNITS;
                            distance = tof * SPEED_OF_LIGHT;

                            sprintf(dist_str, "DIST: %3.2f cm", distance*100);

                            DEBUG_USART_SendString(dist_str);

                            dw_instance->state         = TA_TX_WAIT;
                            dw_instance->previousState = TA_RX_WAIT_DATA;
                            dw_instance->done          = INST_DONE;
                        }
                        break;

                        case RTLS_SENDER_FINAL:
                        {
                            // uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
                            // uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                            // double Ra, Rb, Da, Db;
                            // int64 tof_dtu;

                            // /* Retrieve response transmission and final reception timestamps. */
                            // resp_tx_ts  = get_tx_timestamp_u64();
                            // final_rx_ts = get_rx_timestamp_u64();

                            // /* Get timestamps embedded in the final message. */
                            // final_msg_get_ts(&instance->msg_f.messageData[1], &poll_tx_ts);
                            // final_msg_get_ts(&instance->msg_f.messageData[5], &resp_rx_ts);
                            // final_msg_get_ts(&instance->msg_f.messageData[9], &final_tx_ts);

                            // /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
                            // poll_rx_ts_32  = (uint32)poll_rx_ts;
                            // resp_tx_ts_32  = (uint32)resp_tx_ts;
                            // final_rx_ts_32 = (uint32)final_rx_ts;
                            // Ra = (double)(resp_rx_ts     - poll_tx_ts   );
                            // Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                            // Da = (double)(final_tx_ts    - resp_rx_ts   );
                            // Db = (double)(resp_tx_ts_32  - poll_rx_ts_32);
                            // tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                            // tof      = tof_dtu * DWT_TIME_UNITS;
                            // distance = tof * SPEED_OF_LIGHT;

                        }
                        break;

                        default:
                        break;
                    } // end of DWT_SIG_RX_OKAY
                }
                break;

                default:
                break;
            }
        }
        break;

        case TA_WAIT_FOR_RESP:
        {
        }
        break; // end of TA_WAIT_FOR_RESP

        default:
            break;
    }

    return dw_instance->done;
}

void instance_putevent(event_data_t newevent, uint8 type)
{
    instance.dwevent[instance.eventIdxIn] = newevent;

    instance.dwevent[instance.eventIdxIn].type = type;

    instance.eventIdxIn++;

    if(instance.eventIdxIn == MAX_EVENT_NUM)
        instance.eventIdxIn = 0;
}

uint8 instance_peekevent(void)
{
    return instance.dwevent[instance.eventIdxOut].type;
}

event_data_t event_global;
event_data_t* instance_getevent()
{
    if(instance.dwevent[instance.eventIdxOut].type == 0)
    {
        event_global.type = 0;
        return &event_global;
    }

    event_global.type             = instance.dwevent[instance.eventIdxOut].type;
    event_global.rxFrameLength    = instance.dwevent[instance.eventIdxOut].rxFrameLength;
    event_global.poll_rx_ts       = instance.dwevent[instance.eventIdxOut].poll_rx_ts;
    event_global.resp_tx_ts       = instance.dwevent[instance.eventIdxOut].resp_tx_ts;

    memcpy(&event_global.frame, &instance.dwevent[instance.eventIdxOut].frame, event_global.rxFrameLength);

    instance.eventIdxOut++;
    if(instance.eventIdxOut == MAX_EVENT_NUM)
        instance.eventIdxOut = 0;

    return &event_global;
}

uint8 string_cmp(const uint8 *string1, const uint8 *string2, int len)
{
    int i = 0;
    for(i = 0; i < len; i++)
    {
        if(string1[i] == string2[i])
            continue;
        else
            return 0;
    }
    return 1;
}

uint8_t getSwitchStatus(void)
{
    uint8_t s1switch = 0;
    s1switch = is_switch_on(TA_SW1_3) << 5
            | is_switch_on(TA_SW1_4) << 4
            | is_switch_on(TA_SW1_5) << 3
            | is_switch_on(TA_SW1_6) << 2
            | is_switch_on(TA_SW1_7) << 1
            | is_switch_on(TA_SW1_8) << 0;

    return s1switch;
}
