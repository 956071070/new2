#ifndef __COMM_H
#define __COMM_H

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "sleep.h"
#include "port.h"

#define STANDARD_FRAME_SIZE            127
#define MAX_EVENT_NUM                  6
#define SPEED_OF_LIGHT                 299702547

#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536
/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 3300
/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

#define FINAL_MSG_TS_LEN 4
#define RESP_MSG_TS_LEN 4

#define FRAME_TYPE                              (0xC5)

#define RTLS_NO_VALID                           (0x00)
#define RTLS_SENDER_POLL                        (0x01)
#define RTLS_RECEIVER_RESP                      (0x02)
#define RTLS_SENDER_FINAL                       (0x03)
#define FRAME_INSTRUCTION                       (0xFE)
#define FRAME_DATA                              (0xFF)

#define FRAME_INSTRUCTION_NONE                  (0x00)
#define FRAME_INSTRUCTION_FND                   (0x01)
#define FRAME_INSTRUCTION_SET                   (0x02)
#define FRAME_INSTRUCTION_CHGM                  (0x03)
#define FRAME_INSTRUCTION_RESP                  (0x04)

#define FRAME_INSTRUCTION_RESP_1                (0x00)
#define FRAME_INSTRUCTION_RESP_2                (0x01)
#define FRAME_INSTRUCTION_RESP_3                (0x02)
#define FRAME_INSTRUCTION_RESP_4                (0x03)

#define FRAME_INSTRUCTION_CHGM_IN                (0x01) // in
#define FRAME_INSTRUCTION_CHGM_OUT                (0x02) // out


#define INST_NOT_DONE_YET                       0
#define INST_DONE                               1
#define INST_DONE_WAIT_FOR_NEXT_EVENT           2

#define CONTROLER_MODE_FRAME_DELIMITER          "##"

typedef enum instanceModes{SENDER, RECEIVER, INS_MODE_NONE} INST_MODE;

typedef enum
{
    RUN_NONE,
    CONTROLER_MODE,
    CONFIG_MODE,
    NORMAL_MODE,
    RUN_MODE_END

} run_mode_t;

typedef enum
{
    RESP3_SWITCH_MODE_ERR,
    RESP3_TURN_INTO_CONFIG,
    RESP3_DROP_OUT_CONFIG,
    RESP3_IN_DATA,
    RESP3_IN_CONFIG,
    RESP3_END

} resp_3_msg_t;

typedef enum
{
    RESP4_PARAM_SETTING_ERR,
    RESP4_PARAM_SETTING_SUCCESS,
    RESP4_PARAM_EQUAL_TO_THE_OLD_ID,
    RESP4_END

} resp_4_msg_t;

typedef enum
{
    TA_INIT,                // 0
    TA_TX_WAIT,             // 1
    TA_TX_WAIT_SEND,        // 2
    TA_RX_WAIT,             // 3
    TA_RX_WAIT_DATA,        // 4
    TA_WAIT_FOR_RESP        // 5

} INST_STATES;

typedef enum
{
    DWT_SIG_RX_UNKNOWN,
    DWT_SIG_RX_OKAY,
    DWT_SIG_RX_TIMEOUT

} callback_event;

typedef struct {
    uint8_t euid;
    uint8_t panid;
    uint8_t devid;
    uint8_t changed;

} devID_t;

// TX power and PG delay configuration structure
typedef struct {
                uint8 PGdelay;

                //TX POWER
                //31:24     BOOST_0.125ms_PWR
                //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
                //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
                //7:0       DEFAULT_PWR-TX_DATA_PWR
                uint32 txPwr[2]; //
}tx_struct;

typedef struct
{
    // uint8 frameCtrl[2];                         // 0 - 1
    // uint8 seqNum;                               // 2
    // uint8 panID[2];                             // 3 - 4
    // uint8 destAddr[2];                          // 5 - 6
    // uint8 sourceAddr[2];                        // 7 - 8
    // uint8 messageData[STANDARD_FRAME_SIZE-12];  // frameType 9; frameData 10 - ...
    // uint8 crc[2];

    uint8 frame[STANDARD_FRAME_SIZE];

} msg_data_t;

typedef struct
{
    uint8 type;
    uint16 rxFrameLength;

    uint64_t poll_rx_ts;
    uint64_t resp_tx_ts;

    // union
    // {
    //     uint8 frame[STANDARD_FRAME_SIZE];
    //     msg_data_t rxmsg;
    // } msg_u;
    uint8 frame[STANDARD_FRAME_SIZE];

} event_data_t;

typedef struct
{
    INST_MODE    mode;
    INST_STATES  state;
    INST_STATES  previousState;

    int          done;

    msg_data_t   msg_f;
    uint16_t     txFrameLength;

    int          rxMsgCount;

    event_data_t dwevent[MAX_EVENT_NUM];
    uint8        eventIdxOut;
    uint8        eventIdxIn;

} instance_data_t;

#define DW1000_DEBUG
// #define DW1000_NOASSERT
// #define DW1000_ERROR

#define DW1000_DBG_ON                   0x80U
#define DW1000_DBG_OFF                  0x00U
#define DW1000_DBG_HALT                 0x08U

#define DW1000_DBG_TYPES_ON             DW1000_DBG_ON
// #define NORMAL_DEBUG                    DW1000_DBG_ON
#define TIP_DEBUG                       DW1000_DBG_ON
#define PARAM_DEBUG                     DW1000_DBG_ON

#ifndef DW1000_PLATFORM_DIAG
#define DW1000_PLATFORM_DIAG(x) do {uart_printf x;} while(0)
#include <stdio.h>
#include <stdlib.h>
#endif

#ifndef DW1000_PLATFORM_ASSERT
#define DW1000_PLATFORM_ASSERT(x) do {uart_printf("Assertion \"%s\" failed at line %d in %s\n", \
                                        x, __LINE__, __FILE__); fflush(NULL); abort();} while(0)
#include <stdio.h>
#include <stdlib.h>
#endif

#ifdef DW1000_DEBUG
#define DW1000_DEBUG(debug, message) do { \
                                    if( \
                                        ((debug) & DW1000_DBG_ON) && \
                                        ((debug) & DW1000_DBG_TYPES_ON)) { \
                                        DW1000_PLATFORM_DIAG(message); \
                                        if((debug) & DW1000_DBG_HALT) { \
                                            while(1); \
                                        } \
                                    } \
                                } while(0)
#else
#define DW1000_DEBUG(debug, message)
#endif

#ifndef DW1000_NOASSERT
#define DW1000_ASSERT(message, assertion) do { if(!(assertion)) { \
                DW1000_PLATFORM_ASSERT(message); }} while(0)
#else
#define DW1000_ASSERT(message, assertion)
#endif

#ifndef DW1000_ERROR
#ifndef DW1000_NOASSERT
#define DW1000_PLATFORM_ERROR(message)      DW1000_PLATFORM_ASSERT(message)
#elif defined DW1000_DEBUG
#define DW1000_PLATFORM_ERROR(message)      DW1000_PLATFORM_DIAG((message))
#else
#define DW1000_PLATFORM_ERROR(message)
#endif

#define DW1000_ERROR(message, expression, handler) do { if(!(expression)) { \
            DW1000_PLATFORM_ERROR(message); handler;}} while(0)
#endif


extern instance_data_t instance;

static uint64_t final_rx_ts;
static uint64_t final_tx_ts;

static double tof;
static double distance;

uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
 void resp_msg_set_ts(uint8 *ts_field, const uint64_t ts);
 void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);

 void final_msg_set_ts(uint8 *ts_field, uint64_t ts);

int app_event_process(instance_data_t* instance, int message);

void instance_putevent(event_data_t newevent, uint8 type);
uint8 instance_peekevent(void);
event_data_t* instance_getevent();


uint8_t getSwitchStatus(void);


#endif
