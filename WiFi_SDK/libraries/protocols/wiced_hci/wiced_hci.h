/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
#pragma once

#include "wiced_result.h"
/** @file
 *
 * HCI Control Protocol Definitions
 *
 * This file provides definitions for HCI Control Interface between an MCU
 * and hci_control application running on 20706. Please refer to the WICED Smart Ready
 * Software User Manual.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Packets exchanged over the UART between MCU and hci_control application contain 5 byte header
 * -------------------------------------------------------------------------------------------------------
 * |  Packet Type      | Command Code          |    Group Code       |        Packet Length              |
 * -------------------------------------------------------------------------------------------------------
 * |HCI_WICED_PKT(0x19)|HCI_CONTROL_COMMAND_...|HCI_CONTROL_GROUP_...|length(low byte)| length(high byte)|
 * -------------------------------------------------------------------------------------------------------
 */

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum control_group_enum {
    DEVICE = 0,
    LE,
    GATT,
    HF,
    SPP,
    AUDIO,
    HIDD,
    AVRC_TARGET,
    TEST,
    AIO,
    TIME,
    ANCS,
    ALERT,
    IAP2,
    AG,
    LN,
    BSG,
    AVRC_CONTROLLER,
    SIXLO,
    AUDIO_SINK=0x14,
    MESH = 0x16,
    ZB_GENERAL       = 0x80,
    ZB_GATEWAY       = 0x81,
    ZB_ZCL           = 0x8a,
    ZB_ZCL_GENERAL   = 0x8b,
    ZB_QT            = 0x90,
    ZB_VENDOR        = 0x9F,
    MAX_CONTROL_GROUP
} control_group_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/
#define HCI_EVENT_PKT                                       4
#define HCI_ACL_DATA_PKT                                    2
#define HCI_WICED_PKT                                       25

/*
 * Group codes
 */
#define HCI_CONTROL_GROUP_DEVICE                              0x00
#define HCI_CONTROL_GROUP_LE                                  0x01
#define HCI_CONTROL_GROUP_GATT                                0x02
#define HCI_CONTROL_GROUP_HF                                  0x03
#define HCI_CONTROL_GROUP_SPP                                 0x04
#define HCI_CONTROL_GROUP_AUDIO                               0x05
#define HCI_CONTROL_GROUP_HIDD                                0x06
#define HCI_CONTROL_GROUP_AVRC_TARGET                         0x07
#define HCI_CONTROL_GROUP_TEST                                0x08
#define HCI_CONTROL_GROUP_AIO                                 0x09
#define HCI_CONTROL_GROUP_TIME                                0x0a
#define HCI_CONTROL_GROUP_ANCS                                0x0b
#define HCI_CONTROL_GROUP_ALERT                               0x0c
#define HCI_CONTROL_GROUP_IAP2                                0x0d
#define HCI_CONTROL_GROUP_AG                                  0x0e
#define HCI_CONTROL_GROUP_LN                                  0x0f
#define HCI_CONTROL_GROUP_BSG                                 0x10
#define HCI_CONTROL_GROUP_AVRC_CONTROLLER                     0x11
#define HCI_CONTROL_GROUP_SIXLO                               0x12
#define HCI_CONTROL_GROUP_AUDIO_SINK                          0x14
#define HCI_CONTROL_GROUP_MESH                                0x16
#define HCI_CONTROL_GROUP_ZB_FIRST                            0x80
#define HCI_CONTROL_GROUP_ZB_GENERAL                          0x80
#define HCI_CONTROL_GROUP_ZB_GATEWAY                          0x81
#define HCI_CONTROL_GROUP_ZB_ZCL                              0x8a
#define HCI_CONTROL_GROUP_ZB_ZCL_GENERAL                      0x8b
#define HCI_CONTROL_GROUP_ZB_QT                               0x90
#define HCI_CONTROL_GROUP_ZB_VENDOR                           0x9F
#define HCI_CONTROL_GROUP_ZB_LAST                             0x9F
#define HCI_CONTROL_GROUP_MISC                                0xFF

#define HCI_CONTROL_GROUP(x) ((((x) >> 8)) & 0xff)


/*
 * General purpose commands
 */
#define HCI_CONTROL_COMMAND_RESET                           ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x01 )    /* Restart controller */
#define HCI_CONTROL_COMMAND_TRACE_ENABLE                    ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x02 )    /* Enable or disable WICED traces */
#define HCI_CONTROL_COMMAND_SET_LOCAL_BDA                   ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x03 )    /* Set local device addrsss */
#define HCI_CONTROL_COMMAND_SET_BAUD_RATE                   ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x04 )    /* Change UART baud rate */
#define HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA                 ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x05 )    /* Download previously saved NVRAM chunk */
#define HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA               ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x06 )    /* Delete NVRAM chunk currently stored in RAM */
#define HCI_CONTROL_COMMAND_INQUIRY                         ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x07 )    /* Start/stop inquiry */
#define HCI_CONTROL_COMMAND_SET_VISIBILITY                  ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x08 )    /* Set BR/EDR connectability and discoverability of the device */
#define HCI_CONTROL_COMMAND_SET_PAIRING_MODE                ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x09 )    /* Set Pairing Mode for the device 0 = Not pairable 1 = Pairable */
#define HCI_CONTROL_COMMAND_UNBOND                          ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0A )    /* Delete bond with specified BDADDR */
#define HCI_CONTROL_COMMAND_USER_CONFIRMATION               ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0B )    /* User Confirmation during pairing, TRUE/FALSE passed as parameter */
#define HCI_CONTROL_COMMAND_ENABLE_COEX                     ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0C )    /* Enable coex functionality */
#define HCI_CONTROL_COMMAND_DISABLE_COEX                    ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0D )    /* Disable coex functionality */
#define HCI_CONTROL_COMMAND_SET_BATTERY_LEVEL               ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0E )    /* Sets battery level in the GATT database */
#define HCI_CONTROL_COMMAND_READ_LOCAL_BDA                  ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0F )    /* Get local device addrsss */
#define HCI_CONTROL_COMMAND_BOND                            ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x10 )    /* Initiate Bonding with a peer device */
#define HCI_CONTROL_COMMAND_READ_BUFF_STATS                 ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x11 )    /* Read Buffer statistics */
#define HCI_CONTROL_COMMAND_SET_LOCAL_NAME                  ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x12 )    /* Set the local name     */

/*
 * LE Commands
 * Define commands sent to the GAP/GATT implementation on 20706
 */
#define HCI_CONTROL_LE_COMMAND_SCAN                         ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x01 )    /* start scan */
#define HCI_CONTROL_LE_COMMAND_ADVERTISE                    ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x02 )    /* start advertisements */
#define HCI_CONTROL_LE_COMMAND_CONNECT                      ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x03 )    /* connect to peer */
#define HCI_CONTROL_LE_COMMAND_CANCEL_CONNECT               ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x04 )    /* cancel connect */
#define HCI_CONTROL_LE_COMMAND_DISCONNECT                   ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x05 )    /* disconnect */
#define HCI_CONTROL_LE_RE_PAIR                              ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x06 )    /* delete keys and then re-pair */
#define HCI_CONTROL_LE_COMMAND_GET_IDENTITY_ADDRESS         ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x07 )    /* get identity address */
#define HCI_CONTROL_LE_COMMAND_SET_CHANNEL_CLASSIFICATION   ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x08 )    /* set channel classification for the available 40 channels */
#define HCI_CONTROL_LE_COMMAND_SET_CONN_PARAMS              ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x09 )    /* set connection parameters */
#define HCI_CONTROL_LE_COMMAND_SET_RAW_ADVERTISE_DATA       ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x0a )    /* set raw advertisement data */
#define HCI_CONTROL_LE_COMMAND_SECURITY_GRANT               ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x0b )    /* grant or deny access */

/*
 * GATT Commands
 * Define commands to perform various GATT procedures
 */
#define HCI_CONTROL_GATT_COMMAND_DISCOVER_SERVICES          ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x01 )    /* discover services */
#define HCI_CONTROL_GATT_COMMAND_DISCOVER_CHARACTERISTICS   ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x02 )    /* discover characteristics */
#define HCI_CONTROL_GATT_COMMAND_DISCOVER_DESCRIPTORS       ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x03 )    /* discover descriptors */
#define HCI_CONTROL_GATT_COMMAND_READ_REQUEST               ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x04 )    /* send read request */
#define HCI_CONTROL_GATT_COMMAND_READ_RESPONSE              ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x05 )    /* send read response */
#define HCI_CONTROL_GATT_COMMAND_WRITE_COMMAND              ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x06 )    /* send write command */
#define HCI_CONTROL_GATT_COMMAND_WRITE_REQUEST              ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x07 )    /* send write request */
#define HCI_CONTROL_GATT_COMMAND_WRITE_RESPONSE             ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x08 )    /* send write response */
#define HCI_CONTROL_GATT_COMMAND_NOTIFY                     ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x09 )    /* send notification */
#define HCI_CONTROL_GATT_COMMAND_INDICATE                   ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x0a )    /* send indication */
#define HCI_CONTROL_GATT_COMMAND_INDICATE_CONFIRM           ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x0b )    /* send indication confirmation */
#define HCI_CONTROL_GATT_COMMAND_REGISTER                   ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x0c )    /* register GATT callback */
#define HCI_CONTROL_GATT_COMMAND_DB_INIT                    ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x0d )    /* initialize GATT database */
/*
 * Handsfree Commands
 * Define commands sent to the HFP profile
 */
#define HCI_CONTROL_HF_COMMAND_CONNECT                      ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x01 )    /* establish connection to HF Audio Gateway */
#define HCI_CONTROL_HF_COMMAND_DISCONNECT                   ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x02 )    /* release HF connection */
#define HCI_CONTROL_HF_COMMAND_OPEN_AUDIO                   ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x03 )    /* create audio connection on existing service level connection */
#define HCI_CONTROL_HF_COMMAND_CLOSE_AUDIO                  ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x04 )    /* disconnect audio */
#define HCI_CONTROL_HF_COMMAND_AUDIO_ACCEPT_CONN            ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x05 )    /* Accept/Reject Audio connection request */
#define HCI_CONTROL_HF_COMMAND_TURN_OFF_PCM_CLK             ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x06 )    /* To turnoff PCM/I2S clock in master case for sco */

/*
 * Sub commands to send various AT Commands
 */
#define HCI_CONTROL_HF_AT_COMMAND_BASE                      ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x20 )    /* send AT command and supporting data */
#define HCI_CONTROL_HF_AT_COMMAND_SPK                       0x00    /* Update speaker volume */
#define HCI_CONTROL_HF_AT_COMMAND_MIC                       0x01    /* Update microphone volume */
#define HCI_CONTROL_HF_AT_COMMAND_A                         0x02    /* Answer incoming call */
#define HCI_CONTROL_HF_AT_COMMAND_BINP                      0x03    /* Retrieve number from voice tag */
#define HCI_CONTROL_HF_AT_COMMAND_BVRA                      0x04    /* Enable/Disable voice recognition */
#define HCI_CONTROL_HF_AT_COMMAND_BLDN                      0x05    /* Last Number redial */
#define HCI_CONTROL_HF_AT_COMMAND_CHLD                      0x06    /* Call hold command */
#define HCI_CONTROL_HF_AT_COMMAND_CHUP                      0x07    /* Call hang up command */
#define HCI_CONTROL_HF_AT_COMMAND_CIND                      0x08    /* Read Indicator Status */
#define HCI_CONTROL_HF_AT_COMMAND_CNUM                      0x09    /* Retrieve Subscriber number */
#define HCI_CONTROL_HF_AT_COMMAND_D                         0x0A    /* Place a call using a number or memory dial */
#define HCI_CONTROL_HF_AT_COMMAND_NREC                      0x0B    /* Disable Noise reduction and echo canceling in AG */
#define HCI_CONTROL_HF_AT_COMMAND_VTS                       0x0C    /* Transmit DTMF tone */
#define HCI_CONTROL_HF_AT_COMMAND_BTRH                      0x0D    /* CCAP incoming call hold */
#define HCI_CONTROL_HF_AT_COMMAND_COPS                      0x0E    /* Query operator selection */
#define HCI_CONTROL_HF_AT_COMMAND_CMEE                      0x0F    /* Enable/disable extended AG result codes */
#define HCI_CONTROL_HF_AT_COMMAND_CLCC                      0x10    /* Query list of current calls in AG */
#define HCI_CONTROL_HF_AT_COMMAND_BIA                       0x11    /* Activate/Deactivate indicators */
#define HCI_CONTROL_HF_AT_COMMAND_BIEV                      0x12    /* Send HF indicator value to peer */
#define HCI_CONTROL_HF_AT_COMMAND_UNAT                      0x13    /* Transmit AT command not in the spec  */
#define HCI_CONTROL_HF_AT_COMMAND_MAX                       0x13    /* For command validation */

/*
 * Serial Port Profile Commands
 * Define commands sent to the SPP profile
 */
#define HCI_CONTROL_SPP_COMMAND_CONNECT                     ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x01 )    /* establish connection to SPP server */
#define HCI_CONTROL_SPP_COMMAND_DISCONNECT                  ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x02 )    /* release SPP connection */
#define HCI_CONTROL_SPP_COMMAND_DATA                        ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x03 )    /* send data */

/*
* Audio Profile Commands
* Define commands sent to the Audio profile
*/
#define HCI_CONTROL_AUDIO_COMMAND_CONNECT                   ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x01 )    /* Audio connect to sink */
#define HCI_CONTROL_AUDIO_COMMAND_DISCONNECT                ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x02 )    /* Audio disconnect  */
#define HCI_CONTROL_AUDIO_START                             ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x03 )    /* start audio with speciifc sample rate/mode */
#define HCI_CONTROL_AUDIO_STOP                              ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x04 )    /* stop audio */
#define HCI_CONTROL_AUDIO_PACKET_COUNT                      ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x05 )    /* debug packet counter sent from host */
#define HCI_CONTROL_AUDIO_START_RSP                         ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x07 )    /* Response to AV_START request*/

/*
* AVRC Target Profile Commands
* Define commands sent to the AVRC profile
*/
#define HCI_CONTROL_AVRC_TARGET_COMMAND_CONNECT                     ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x01 )    /* Initiate a connection to the peer. */
#define HCI_CONTROL_AVRC_TARGET_COMMAND_DISCONNECT                  ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x02 )    /* Disconnect a connection to the peer. */
#define HCI_CONTROL_AVRC_TARGET_COMMAND_TRACK_INFO                  ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x05 )    /* Track info sent to embedded app */
#define HCI_CONTROL_AVRC_TARGET_COMMAND_PLAYER_STATUS               ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x06 )    /* Player status info sent to embedded app */
#define HCI_CONTROL_AVRC_TARGET_COMMAND_REPEAT_MODE_CHANGE          ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x07 )    /* Repeat Mode changes sent to embedded app */
#define HCI_CONTROL_AVRC_TARGET_COMMAND_SHUFFLE_MODE_CHANGE         ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x08 )    /* Shuffle Mode changes sent to embedded app */
#define HCI_CONTROL_AVRC_TARGET_COMMAND_EQUALIZER_STATUS_CHANGE     ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x09 )    /* EQ Status changes sent to embedded app */
#define HCI_CONTROL_AVRC_TARGET_COMMAND_SCAN_STATUS_CHANGE          ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x0A )    /* Scan Status changes sent to embedded app */

#define HCI_CONTROL_AVRC_TARGET_COMMAND_REGISTER_NOTIFICATION       ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x99 )    /* Register for notifications (PTS only) */

/*
* AVRC Controller Profile Commands
* Define commands sent to the AVRC profile
*/
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_CONNECT                 ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x01 )    /* Initiate a connection to the peer */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_DISCONNECT              ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x02 )    /* Disconnect from the peer */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY                    ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x03 )    /* Send play command to the player */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_STOP                    ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x04 )    /* Send stop command to the player */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE                   ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x05 )    /* Send pause command to the player */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_FAST_FORWARD      ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x06 )    /* Start fast forward on the player */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_END_FAST_FORWARD        ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x07 )    /* End fast forward on the player */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_REWIND            ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x08 )    /* Passthrough Rewind Command */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_END_REWIND              ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x09 )    /* Passthrough Rewind Command */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_NEXT_TRACK              ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0a )    /* Passthrough Next Command */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PREVIOUS_TRACK          ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0b )    /* Passthrough Prev Command */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP               ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0c )    /* Passthrough Vol Up Command */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN             ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0d )    /* Passthrough Vol Down Command */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_GET_TRACK_INFO          ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0e )    /* Get Track Metadata */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_EQUALIZER_STATUS    ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0f )    /* Turn Equalizer On/Off */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_REPEAT_MODE         ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x10 )    /* Set Repeat Mode */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_SHUFFLE_MODE        ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x11 )    /* Set Shuffle Mode */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_SCAN_STATUS         ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x12 )    /* Set Scan Mode to Off, All tracks or Group scan  */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_LEVEL            ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x13 )    /* Set Absolute Volume */
#define HCI_CONTROL_AVRC_CONTROLLER_COMMAND_GET_PLAY_STATUS         ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x14 )    /*GET Playstatus from Peer*/

/*
 * HID Device Commands
 */
#define HCI_CONTROL_HID_COMMAND_ACCEPT_PAIRING              ( ( HCI_CONTROL_GROUP_HIDD << 8 ) | 0x01 )     /* Set device discoverable/connectable to accept pairing */
#define HCI_CONTROL_HID_COMMAND_SEND_REPORT                 ( ( HCI_CONTROL_GROUP_HIDD << 8 ) | 0x02 )     /* Send HID report */
#define HCI_CONTROL_HID_COMMAND_PUSH_PAIRING_HOST_INFO      ( ( HCI_CONTROL_GROUP_HIDD << 8 ) | 0x03 )     /* Paired host address and link keys */
#define HCI_CONTROL_HID_COMMAND_CONNECT                     ( ( HCI_CONTROL_GROUP_HIDD << 8 ) | 0x04 )     /* Connect to previously paired host */

/*
* Test Commands
*/
#define HCI_CONTROL_TEST_COMMAND_LE_RECEIVER                ( ( HCI_CONTROL_GROUP_TEST << 8 ) | 0x01 )     /* Start LE Receiver Test */
#define HCI_CONTROL_TEST_COMMAND_LE_TRASMITTER              ( ( HCI_CONTROL_GROUP_TEST << 8 ) | 0x02 )     /* Start LE Tramsmitter Test */
#define HCI_CONTROL_TEST_COMMAND_LE_TEST_END                ( ( HCI_CONTROL_GROUP_TEST << 8 ) | 0x03 )     /* End LE Test */
#define HCI_CONTROL_TEST_COMMAND_CONTINUOUS_TRANSMIT        ( ( HCI_CONTROL_GROUP_TEST << 8 ) | 0x04 )     /* Start Continuous Transmit */
#define HCI_CONTROL_TEST_COMMAND_RECEIVE_ONLY               ( ( HCI_CONTROL_GROUP_TEST << 8 ) | 0x05 )     /* Start Receiver Test */
#define HCI_CONTROL_TEST_COMMAND_DISABLE_POWER_CONTROL      ( ( HCI_CONTROL_GROUP_TEST << 8 ) | 0x06 )     /* Turn off power control for the Link */
#define HCI_CONTROL_TEST_COMMAND_SET_TX_POWER               ( ( HCI_CONTROL_GROUP_TEST << 8 ) | 0x07 )     /* Set Tx power for the Link */
#define HCI_CONTROL_TEST_COMMAND_INCR_DECR_PEER_POWER       ( ( HCI_CONTROL_GROUP_TEST << 8 ) | 0x08 )     /* Request peer to incr/decr power for the Link */

/*
* Automation IO Commands
*/
#define HCI_CONTROL_AIO_COMMAND_DIGITAL_IN                  ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x01 )      /* Digital input */
#define HCI_CONTROL_AIO_COMMAND_ANALOG_IN                   ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x02 )      /* Analog input */
#define HCI_CONTROL_AIO_COMMAND_CONNECT                     ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x03 )      /* Connect to server */
#define HCI_CONTROL_AIO_COMMAND_READ                        ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x04 )      /* Read value */
#define HCI_CONTROL_AIO_COMMAND_WRITE                       ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x05 )      /* Write value */
#define HCI_CONTROL_AIO_COMMAND_WRITE_NO_RSP                ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x06 )      /* Write with no response */
#define HCI_CONTROL_AIO_COMMAND_SET_CLIENT_CONFIG           ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x07 )      /* Set client configuration */
#define HCI_CONTROL_AIO_COMMAND_SET_VALUE_TRIGGER           ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x08 )      /* Set value trigger */
#define HCI_CONTROL_AIO_COMMAND_SET_TIME_TRIGGER            ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x09 )      /* Set time trigger */
#define HCI_CONTROL_AIO_COMMAND_SET_USER_DESCRIPTION        ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x0a )      /* Set user description */
#define HCI_CONTROL_AIO_COMMAND_DISCONNECT                  ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x0b )      /* Disconnect from server */

/*
 * Define ANCS commands
 */
#define HCI_CONTROL_ANCS_COMMAND_ACTION                     ( ( HCI_CONTROL_GROUP_ANCS << 8 ) | 0x01 )      /* ANCS notification */

/*
 * IAP2 Commands
 * Define commands sent to the IAP2 implementation
 */
#define HCI_CONTROL_IAP2_COMMAND_CONNECT                     ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x01 )    /* establish connection to SPP server */
#define HCI_CONTROL_IAP2_COMMAND_DISCONNECT                  ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x02 )    /* release SPP connection */
#define HCI_CONTROL_IAP2_COMMAND_DATA                        ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x03 )    /* send data */
#define HCI_CONTROL_IAP2_COMMAND_GET_AUTH_CHIP_INFO          ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x04 )    /* send get auth chip info */

/*
 * Handsfree AG Commands
 * Define commands sent to the HF-AG profile
 */
#define HCI_CONTROL_AG_COMMAND_CONNECT                      ( ( HCI_CONTROL_GROUP_AG << 8 ) | 0x01 )    /* establish connection to HF Device */
#define HCI_CONTROL_AG_COMMAND_DISCONNECT                   ( ( HCI_CONTROL_GROUP_AG << 8 ) | 0x02 )    /* release HF connection */
#define HCI_CONTROL_AG_COMMAND_OPEN_AUDIO                   ( ( HCI_CONTROL_GROUP_AG << 8 ) | 0x03 )    /* create audio connection on existing service level connection */
#define HCI_CONTROL_AG_COMMAND_CLOSE_AUDIO                  ( ( HCI_CONTROL_GROUP_AG << 8 ) | 0x04 )    /* disconnect audio */

/*
 * Location and Navigation commands
 */
#define HCI_CONTROL_LN_COMMAND_SET_DATA_SOURCE              ( ( HCI_CONTROL_GROUP_LN << 8 ) | 0x01 )    /* Location/navigation data from host or board */
#define HCI_CONTROL_LN_COMMAND_LN_FEATURE                   ( ( HCI_CONTROL_GROUP_LN << 8 ) | 0x02 )    /* Supported features of server */
#define HCI_CONTROL_LN_COMMAND_LOCATION_SPEED               ( ( HCI_CONTROL_GROUP_LN << 8 ) | 0x03 )    /* Location and speed data */
#define HCI_CONTROL_LN_COMMAND_POSITION_QUALITY_CHANGED     ( ( HCI_CONTROL_GROUP_LN << 8 ) | 0x04 )    /* Position quality data update */
#define HCI_CONTROL_LN_COMMAND_LN_CONTROL_RSP               ( ( HCI_CONTROL_GROUP_LN << 8 ) | 0x05 )    /* Response to LN control point request */
#define HCI_CONTROL_LN_COMMAND_NAVIGATION                   ( ( HCI_CONTROL_GROUP_LN << 8 ) | 0x06 )    /* Navigation data */

/*
 * Broadcom Serial over GATT service Commands
 */
#define HCI_CONTROL_BSG_COMMAND_DATA                        ( ( HCI_CONTROL_GROUP_BSG << 8 ) | 0x03 )    /* send data */


/*
 * Miscellaneous commands
 */
#define HCI_CONTROL_MISC_COMMAND_PING                       ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x01 )    /* Ping controller */
#define HCI_CONTROL_MISC_COMMAND_GET_VERSION                ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x02 )    /* Get SDK Version */

/*
* Audio Sink Profile Commands
* Define commands sent to the Audio sink profile
*/
#define HCI_CONTROL_AUDIO_SINK_COMMAND_CONNECT              ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x01 )    /* Audio connect to source */
#define HCI_CONTROL_AUDIO_SINK_COMMAND_DISCONNECT           ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x02 )    /* Audio disconnect  */
#define HCI_CONTROL_AUDIO_SINK_COMMAND_START                ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x03 )    /* start audio with speciifc sample rate/mode */
#define HCI_CONTROL_AUDIO_SINK_COMMAND_STOP                 ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x04 )    /* stop audio */
#define HCI_CONTROL_AUDIO_SINK_COMMAND_START_RSP            ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x05 )    /* Response to A2DP start request, send start response */
#define HCI_CONTROL_AUDIO_SINK_COMMAND_CHANGE_ROUTE         ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x06 )    /* Change the audio route */


/*
 * Mesh Commands
 */

// Commands added for proxy
#define HCI_CONTROL_MESH_COMMAND_START                                      ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xf0 )    /* Application sends mesh start    */
#define HCI_CONTROL_MESH_COMMAND_CONNECT_PROXY                              ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xf1 )    /* Application sends connect proxy */
#define HCI_CONTROL_MESH_COMMAND_SEND_PROXY_DATA                            ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xf2 )    /* Application sends proxy data    */
#define HCI_CONTROL_MESH_COMMAND_PUSH_NVRAM_DATA                            ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xf3 )    /* Application pushes the nvram data to the mesh core */
#define HCI_CONTROL_MESH_COMMAND_STACK_INIT                                 ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xf4 )    /* */
#define HCI_CONTROL_MESH_COMMAND_CONNECTION_STATE                           ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xf5 )    /*  Application use this interface to send connection state , before it sends proxy data */

/*
 * Define general events that controller can send
 */
#define HCI_CONTROL_EVENT_COMMAND_STATUS                    ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x01 )    /* Command status event for the requested operation */
#define HCI_CONTROL_EVENT_WICED_TRACE                       ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x02 )    /* WICED trace packet */
#define HCI_CONTROL_EVENT_HCI_TRACE                         ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x03 )    /* Bluetooth protocol trace */
#define HCI_CONTROL_EVENT_NVRAM_DATA                        ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x04 )    /* Request to MCU to save NVRAM chunk */
#define HCI_CONTROL_EVENT_DEVICE_STARTED                    ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x05 )    /* Device completed power up initialization */
#define HCI_CONTROL_EVENT_INQUIRY_RESULT                    ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x06 )    /* Inquiry result */
#define HCI_CONTROL_EVENT_INQUIRY_COMPLETE                  ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x07 )    /* Inquiry completed event */
#define HCI_CONTROL_EVENT_PAIRING_COMPLETE                  ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x08 )    /* Pairing Completed */
#define HCI_CONTROL_EVENT_ENCRYPTION_CHANGED                ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x09 )    /* Encryption changed event */
#define HCI_CONTROL_EVENT_CONNECTED_DEVICE_NAME             ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0A )    /* Device name event */
#define HCI_CONTROL_EVENT_USER_CONFIRMATION                 ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0B )    /* User Confirmation during pairing */
#define HCI_CONTROL_EVENT_DEVICE_ERROR                      ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0C )    /* Device Error event */
#define HCI_CONTROL_EVENT_READ_LOCAL_BDA                    ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0D )    /* Local BDA Read event */
#define HCI_CONTROL_EVENT_MAX_NUM_OF_PAIRED_DEVICES_REACHED ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0E )    /* Key Buffer Pool Full */
#define HCI_CONTROL_EVENT_READ_BUFFER_STATS                 ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x0F )    /* Read Buffer statistics event */
#define HCI_CONTROL_EVENT_UPDATE_LINK_KEY                   ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x10 )    /* Update link key info in the DCT */
#define HCI_CONTROL_EVENT_REQUEST_ID_KEYS                   ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x11 )    /* Request the ID keys */
#define HCI_CONTROL_EVENT_READ_RSSI                         ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x12 )    /* BCM20706 returns the RSSI of the desired link */
#define HCI_CONTROL_EVENT_DEVICE_INIT                       ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x13 )    /* sends a Device Started event at the end of application initialization. */
#define HCI_CONTROL_EVENT_SECURITY_REQ                      ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x14 )    /* The app will respond to this event with a "wiced_bt_ble_security_grant". */
#define HCI_CONTROL_EVENT_SECURITY_FAILED                   ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x15 )    /* Security procedure/authentication failed. */
#define HCI_CONTROL_EVENT_IO_CAPABILITIES_BR_EDR_REQUEST    ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x16 )    /* IO capablities request */
#define HCI_CONTROL_EVENT_KEYPRESS_NOTIFICATION             ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x17 )    /* KeyPress notification */
#define HCI_CONTROL_EVENT_CONNECTION_STATUS                 ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x18 )    /* Connection Status */

/*
 * Define events from the HFP profile
 */
#define HCI_CONTROL_HF_EVENT_OPEN                           ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x01 )    /* HS connection opened or connection attempt failed  */
#define HCI_CONTROL_HF_EVENT_CLOSE                          ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x02 )    /* HS connection closed */
#define HCI_CONTROL_HF_EVENT_CONNECTED                      ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x03 )    /* HS Service Level Connection is UP */
#define HCI_CONTROL_HF_EVENT_AUDIO_OPEN                     ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x04 )    /* Audio connection open */
#define HCI_CONTROL_HF_EVENT_AUDIO_CLOSE                    ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x05 )    /* Audio connection closed */
#define HCI_CONTROL_HF_EVENT_AUDIO_CONN_REQ                 ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x06 )    /* Audio connection request event */

/*
 * Subcommands AT resoponses defined with AT Commands
 */
#define HCI_CONTROL_HF_AT_EVENT_BASE                        ( ( HCI_CONTROL_GROUP_HF << 8 ) | 0x20 )
#define HCI_CONTROL_HF_AT_EVENT_OK                          0x00    /* OK response received to previous AT command */
#define HCI_CONTROL_HF_AT_EVENT_ERROR                       0x01    /* ERROR response received */
#define HCI_CONTROL_HF_AT_EVENT_CMEE                        0x02    /* Extended error codes response */
#define HCI_CONTROL_HF_AT_EVENT_RING                        0x03    /* RING indicator */
#define HCI_CONTROL_HF_AT_EVENT_VGS                         0x04
#define HCI_CONTROL_HF_AT_EVENT_VGM                         0x05
#define HCI_CONTROL_HF_AT_EVENT_CCWA                        0x06
#define HCI_CONTROL_HF_AT_EVENT_CHLD                        0x07
#define HCI_CONTROL_HF_AT_EVENT_CIND                        0x08
#define HCI_CONTROL_HF_AT_EVENT_CLIP                        0x09
#define HCI_CONTROL_HF_AT_EVENT_CIEV                        0x0A
#define HCI_CONTROL_HF_AT_EVENT_BINP                        0x0B
#define HCI_CONTROL_HF_AT_EVENT_BVRA                        0x0C
#define HCI_CONTROL_HF_AT_EVENT_BSIR                        0x0D
#define HCI_CONTROL_HF_AT_EVENT_CNUM                        0x0E
#define HCI_CONTROL_HF_AT_EVENT_BTRH                        0x0F
#define HCI_CONTROL_HF_AT_EVENT_COPS                        0x10
#define HCI_CONTROL_HF_AT_EVENT_CLCC                        0x11
#define HCI_CONTROL_HF_AT_EVENT_BIND                        0x12
#define HCI_CONTROL_HF_AT_EVENT_UNAT                        0x13
#define HCI_CONTROL_HF_AT_EVENT_MAX                         0x13    /* Maximum AT event value */

/*
 * Define LE events from the BLE GATT/GAP
 */
#define HCI_CONTROL_LE_EVENT_COMMAND_STATUS                 ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x01 )    /* Command status event for the requested operation */
#define HCI_CONTROL_LE_EVENT_SCAN_STATUS                    ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x02 )    /* LE scanning state change notification */
#define HCI_CONTROL_LE_EVENT_ADVERTISEMENT_REPORT           ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x03 )    /* Advertisement report */
#define HCI_CONTROL_LE_EVENT_ADVERTISEMENT_STATE            ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x04 )    /* LE Advertisement state change notification */
#define HCI_CONTROL_LE_EVENT_CONNECTED                      ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x05 )    /* LE Connection established */
#define HCI_CONTROL_LE_EVENT_DISCONNECTED                   ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x06 )    /* Le Connection Terminated */
#define HCI_CONTROL_LE_EVENT_IDENTITY_ADDRESS               ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x07 )    /* Identity address */
#define HCI_CONTROL_LE_EVENT_PEER_MTU                       ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0x08 )    /* Peer MTU */
/*
 * Define GATT events
 */
#define HCI_CONTROL_GATT_EVENT_COMMAND_STATUS               ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x01 )    /* Command status event for the requested operation */
#define HCI_CONTROL_GATT_EVENT_DISCOVERY_COMPLETE           ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x02 )    /* Discovery requested by host completed */
#define HCI_CONTROL_GATT_EVENT_SERVICE_DISCOVERED           ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x03 )    /* Service discovered */
#define HCI_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED    ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x04 )    /* Characteristic discovered */
#define HCI_CONTROL_GATT_EVENT_DESCRIPTOR_DISCOVERED        ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x05 )    /* Characteristic descriptor discovered */
#define HCI_CONTROL_GATT_EVENT_READ_REQUEST                 ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x06 )    /* Peer sent Read Request */
#define HCI_CONTROL_GATT_EVENT_READ_RESPONSE                ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x07 )    /* Read response */
#define HCI_CONTROL_GATT_EVENT_WRITE_REQUEST                ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x08 )    /* Peer sent Write Request */
#define HCI_CONTROL_GATT_EVENT_WRITE_RESPONSE               ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x09 )    /* Write operation completed */
#define HCI_CONTROL_GATT_EVENT_INDICATION                   ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x0a )    /* indication from peer */
#define HCI_CONTROL_GATT_EVENT_NOTIFICATION                 ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x0b )    /* notification from peer */
#define HCI_CONTROL_GATT_EVENT_READ_ERROR                   ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x0c )    /* GATT Read operation error */
#define HCI_CONTROL_GATT_EVENT_WRITE_ERROR                  ( ( HCI_CONTROL_GROUP_GATT << 8 ) | 0x0d )    /* GATT Write operation error */

/*
 * Define events from the SPP profile
 */
#define HCI_CONTROL_SPP_EVENT_CONNECTED                     ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x01 )    /* SPP connection opened */
#define HCI_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND             ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x02 )    /* SDP record with SPP service not found */
#define HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED             ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x03 )    /* Connection attempt failed  */
#define HCI_CONTROL_SPP_EVENT_DISCONNECTED                  ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x04 )    /* SPP connection closed */
#define HCI_CONTROL_SPP_EVENT_TX_COMPLETE                   ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x05 )    /* Data packet has been queued for transmission */
#define HCI_CONTROL_SPP_EVENT_RX_DATA                       ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x06 )    /* SPP data received */
#define HCI_CONTROL_SPP_EVENT_COMMAND_STATUS                ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x07 )    /* Command status event for the requested operation */

/*
 * Define events from the Audio profile
 */
#define HCI_CONTROL_AUDIO_EVENT_COMMAND_COMPLETE            ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x00 )    /* Command complete event for the requested operation */
#define HCI_CONTROL_AUDIO_EVENT_COMMAND_STATUS              ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x01 )    /* Command status event for the requested operation */
#define HCI_CONTROL_AUDIO_EVENT_CONNECTED                   ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x02 )    /* Audio connection opened */
#define HCI_CONTROL_AUDIO_EVENT_SERVICE_NOT_FOUND           ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x03 )    /* SDP record with audio service not found */
#define HCI_CONTROL_AUDIO_EVENT_CONNECTION_FAILED           ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x04 )    /* Connection attempt failed  */
#define HCI_CONTROL_AUDIO_EVENT_DISCONNECTED                ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x05 )    /* Audio connection closed */
#define HCI_CONTROL_AUDIO_EVENT_REQUEST_DATA                ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x06 )    /* Request for audio pcm sample data */
#define HCI_CONTROL_AUDIO_EVENT_STARTED                     ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x07 )    /* Command for audio start succeeded */
#define HCI_CONTROL_AUDIO_EVENT_STOPPED                     ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x08 )    /* Command for audio stop completed */
#define HCI_CONTROL_AUDIO_EVENT_CODEC_CONFIGURED            ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x09 )    /* Peer codec configured event*/
#define HCI_CONTROL_AUDIO_EVENT_START_IND                   ( ( HCI_CONTROL_GROUP_AUDIO << 8 ) | 0x10 )    /* A2DP START Indication event*/

/*
 * Define events from the AVRCP profile target events
 */
#define HCI_CONTROL_AVRC_TARGET_EVENT_CONNECTED             ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x01 )    /* AVRCP Target connected */
#define HCI_CONTROL_AVRC_TARGET_EVENT_DISCONNECTED          ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x02 )    /* AVRCP Target disconnected */
#define HCI_CONTROL_AVRC_TARGET_EVENT_PLAY                  ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x03 )    /* Play command received */
#define HCI_CONTROL_AVRC_TARGET_EVENT_STOP                  ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x04 )    /* Stop command received */
#define HCI_CONTROL_AVRC_TARGET_EVENT_PAUSE                 ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x05 )    /* Pause command received */
#define HCI_CONTROL_AVRC_TARGET_EVENT_NEXT_TRACK            ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x06 )    /* Next Track command received */
#define HCI_CONTROL_AVRC_TARGET_EVENT_PREVIOUS_TRACK        ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x07 )    /* Previous track command received */
#define HCI_CONTROL_AVRC_TARGET_EVENT_BEGIN_FAST_FORWARD    ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x08 )    /* Next Track command received */
#define HCI_CONTROL_AVRC_TARGET_EVENT_END_FAST_FORWARD      ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x09 )    /* Previous track command received */
#define HCI_CONTROL_AVRC_TARGET_EVENT_BEGIN_REWIND          ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x0A )    /* Next Track command received */
#define HCI_CONTROL_AVRC_TARGET_EVENT_END_REWIND            ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x0B )    /* Previous track command received */
#define HCI_CONTROL_AVRC_TARGET_EVENT_VOLUME_LEVEL          ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x0C )    /* Volume Level changed received */
#define HCI_CONTROL_AVRC_TARGET_EVENT_REPEAT_SETTINGS       ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x0D )    /* 'Repeat' settings changed by peer, send into to MCU app */
#define HCI_CONTROL_AVRC_TARGET_EVENT_SHUFFLE_SETTINGS      ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x0E )    /* 'Shuffle' settings changed by peer, send into to MCU app */
#define HCI_CONTROL_AVRC_TARGET_EVENT_GET_PLAYER_STATUS     ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x0F )    /* Player status info requested by peer, get info from MCU app */
#define HCI_CONTROL_AVRC_TARGET_EVENT_COMMAND_STATUS        ( ( HCI_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0xFF )    /* Result status for AVRCP commands */

/*
 * Define events from the AVRCP CT profile
 */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED          ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x01 )    /* AVRCP Controller connected */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED       ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x02 )    /* AVRCP Controller disconnected */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_CURRENT_TRACK_INFO ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x03 )    /* AVRCP Controller disconnected */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS        ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x04 )    /* AVRCP Controller Play Status Change */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_POSITION      ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x05 )    /* AVRCP Controller Play Position Change */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_CHANGE       ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x06 )    /* AVRCP Controller Track Changed */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_END          ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x07 )    /* AVRCP Controller Track reached End */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_START        ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x08 )    /* AVRCP Controller Track reached Start */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_AVAILABLE  ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x09 )    /* AVRCP Controller Player setting available */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_CHANGE     ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0a )    /* AVRCP Controller Player setting changed */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAYER_CHANGE      ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0b )    /* AVRCP Controller Player changed */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS_INFO   ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0c )    /* AVRCP Controller Play status Info includes play status/song pos/song length */
#define HCI_CONTROL_AVRC_CONTROLLER_EVENT_COMMAND_STATUS     ( ( HCI_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0xFF )    /* Result status for AVRCP commands */

/*
 * Define events from the HFP profile
 */
#define HCI_CONTROL_HID_EVENT_OPENED                        ( ( HCI_CONTROL_GROUP_HIDD << 8 ) | 0x01 )    /* Both HID channels are opened */
#define HCI_CONTROL_HID_EVENT_VIRTUAL_CABLE_UNPLUGGED       ( ( HCI_CONTROL_GROUP_HIDD << 8 ) | 0x02 )    /* Host requested Virtual Cable Unplug */
#define HCI_CONTROL_HID_EVENT_DATA                          ( ( HCI_CONTROL_GROUP_HIDD << 8 ) | 0x03 )    /* Host sent report */
#define HCI_CONTROL_HID_EVENT_CLOSED                        ( ( HCI_CONTROL_GROUP_HIDD << 8 ) | 0x04 )    /* Host attempt to establish connection failed */

/*
 * Define events from the Automation IO profile
 */
#define HCI_CONTROL_AIO_EVENT_COMMAND_STATUS                ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x01 )      /* Command status */
#define HCI_CONTROL_AIO_EVENT_DIGITAL_OUT                   ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x02 )      /* Digital output */
#define HCI_CONTROL_AIO_EVENT_ANALOG_OUT                    ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x03 )      /* Analog output */
#define HCI_CONTROL_AIO_EVENT_CONNECTED                     ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x04 )      /* Connected to server */
#define HCI_CONTROL_AIO_EVENT_READ_RSP                      ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x05 )      /* Read response */
#define HCI_CONTROL_AIO_EVENT_WRITE_RSP                     ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x06 )      /* Write response */
#define HCI_CONTROL_AIO_EVENT_VALUE_IN                      ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x07 )      /* Notification/indication */
#define HCI_CONTROL_AIO_EVENT_DISCONNECTED                  ( ( HCI_CONTROL_GROUP_AIO << 8 ) | 0x08 )      /* Disconnected from server */

/*
 * Define events from the Current Time
 */
#define HCI_CONTROL_TIME_EVENT_UPDATE                       ( ( HCI_CONTROL_GROUP_TIME << 8 ) | 0x01 )      /* Time Change notification */

/*
 * Define events from the ANCS
 */
#define HCI_CONTROL_ANCS_EVENT_NOTIFICATION                 ( ( HCI_CONTROL_GROUP_ANCS << 8 ) | 0x01 )      /* ANCS notification */
#define HCI_CONTROL_ANCS_EVENT_COMMAND_STATUS               ( ( HCI_CONTROL_GROUP_ANCS << 8 ) | 0x02 )      /* Command status event for the requested operation */

/*
 * Define events from the FindMe application
 */
#define HCI_CONTROL_ALERT_EVENT_NOTIFICATION                ( ( HCI_CONTROL_GROUP_ALERT << 8 ) | 0x01 )     /* Alert Level Notification */

/*
 * Define events from the IAP2 implementation
 */
#define HCI_CONTROL_IAP2_EVENT_CONNECTED                    ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x01 )    /* IAP2 connection opened */
#define HCI_CONTROL_IAP2_EVENT_SERVICE_NOT_FOUND            ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x02 )    /* SDP record with IAP2 service not found */
#define HCI_CONTROL_IAP2_EVENT_CONNECTION_FAILED            ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x03 )    /* Connection attempt failed  */
#define HCI_CONTROL_IAP2_EVENT_DISCONNECTED                 ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x04 )    /* IAP2 connection closed */
#define HCI_CONTROL_IAP2_EVENT_TX_COMPLETE                  ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x05 )    /* Data packet has been queued for transmission */
#define HCI_CONTROL_IAP2_EVENT_RX_DATA                      ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x06 )    /* IAP2 data received */
#define HCI_CONTROL_IAP2_EVENT_AUTH_CHIP_INFO               ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x07 )    /* IAP2 auth chip info */

/*
 * Define event for Handsfree AG implementation
 */
#define HCI_CONTROL_AG_EVENT_OPEN                           ( ( HCI_CONTROL_GROUP_AG << 8 ) | 0x01 )
#define HCI_CONTROL_AG_EVENT_CLOSE                          ( ( HCI_CONTROL_GROUP_AG << 8 ) | 0x02 )
#define HCI_CONTROL_AG_EVENT_CONNECTED                      ( ( HCI_CONTROL_GROUP_AG << 8 ) | 0x03 )
#define HCI_CONTROL_AG_EVENT_AUDIO_OPEN                     ( ( HCI_CONTROL_GROUP_AG << 8 ) | 0x04 )
#define HCI_CONTROL_AG_EVENT_AUDIO_CLOSE                    ( ( HCI_CONTROL_GROUP_AG << 8 ) | 0x05 )

/*
 * Location and Navigation events
 */
#define HCI_CONTROL_LN_EVENT_GET_LOCATION_SPEED             ( ( HCI_CONTROL_GROUP_LN << 8 ) | 0x01 )    /* Get location and speed data */
#define HCI_CONTROL_LN_EVENT_LN_CONTROL                     ( ( HCI_CONTROL_GROUP_LN << 8 ) | 0x02 )    /* LN control point request from client */
#define HCI_CONTROL_LN_EVENT_GET_NAVIGATION                 ( ( HCI_CONTROL_GROUP_LN << 8 ) | 0x03 )    /* Get navigation data */

/*
 * Define events from the Broadcom Serial over GATT profile
 */
#define HCI_CONTROL_BSG_EVENT_TX_COMPLETE                   ( ( HCI_CONTROL_GROUP_BSG << 8 ) | 0x05 )    /* Data packet has been queued for transmission */
#define HCI_CONTROL_BSG_EVENT_RX_DATA                       ( ( HCI_CONTROL_GROUP_BSG << 8 ) | 0x06 )    /* BSG data received */

/*
* Define events from the Audio sink profile
*/
#define HCI_CONTROL_AUDIO_SINK_EVENT_COMMAND_COMPLETE       ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x00 )    /* Command complete event for the requested operation */
#define HCI_CONTROL_AUDIO_SINK_EVENT_COMMAND_STATUS         ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x01 )    /* Command status event for the requested operation */
#define HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTED              ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x02 )    /* Audio connection opened */
#define HCI_CONTROL_AUDIO_SINK_EVENT_SERVICE_NOT_FOUND      ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x03 )    /* SDP record with audio service not found */
#define HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTION_FAILED      ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x04 )    /* Connection attempt failed  */
#define HCI_CONTROL_AUDIO_SINK_EVENT_DISCONNECTED           ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x05 )    /* Audio connection closed */
#define HCI_CONTROL_AUDIO_SINK_EVENT_STARTED                ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x06 )    /* Command for audio start succeeded */
#define HCI_CONTROL_AUDIO_SINK_EVENT_STOPPED                ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x07 )    /* Command for audio stop completed */
#define HCI_CONTROL_AUDIO_SINK_EVENT_CODEC_CONFIGURED       ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x08 )    /* Peer codec configured event*/
#define HCI_CONTROL_AUDIO_SINK_EVENT_START_IND              ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x09 )    /* A2DP Start indication event, received A2DP Start request */
#define HCI_CONTROL_AUDIO_SINK_EVENT_AUDIO_DATA             ( ( HCI_CONTROL_GROUP_AUDIO_SINK << 8 ) | 0x0a )    /* received audio data. encoded in case of aac, decoded in case of sbc */

/*
 * Define Miscellaneous events
 */
/*
 * Mesh events
 */
// Events added for Gateway Mesh Proxy
#define HCI_CONTROL_MESH_EVENT_CORE_PROVISION_END                           ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xb0 )      /*  */
#define HCI_CONTROL_MESH_EVENT_NVRAM_WRITE                                  ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xb1 )      /*  */
#define HCI_CONTROL_MESH_EVENT_MESH_STATUS                                  ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xb2 )      /*  */
#define HCI_CONTROL_MESH_EVENT_PROVISIONING_STATUS                          ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xb3 )      /*  */
#define HCI_CONTROL_MESH_EVENT_PROXY_CONNECTION_STATUS                      ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xb4 )      /*  */
#define HCI_CONTROL_MESH_EVENT_PROXY_DATA                                   ( ( HCI_CONTROL_GROUP_MESH << 8 ) | 0xb5 )  /* Event to send proxy data to MCU application */


#define HCI_CONTROL_MISC_EVENT_PING_REPLY                   ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x01 )    /* Ping reply */
#define HCI_CONTROL_MISC_EVENT_VERSION                      ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x02 )    /* SDK Version */

/*
 * Define Scan state that is reported with the HCI_CONTROL_LE_EVENT_SCAN_STATUS
 */
#define HCI_CONTROL_SCAN_EVENT_NO_SCAN                      0
#define HCI_CONTROL_SCAN_EVENT_HIGH_SCAN                    1
#define HCI_CONTROL_SCAN_EVENT_LOW_SCAN                     2
#define HCI_CONTROL_SCAN_EVENT_HIGH_CONN                    3
#define HCI_CONTROL_SCAN_EVENT_LOW_CONN                     4

/*
 * Define status code returned in HCI_CONTROL_EVENT_COMMAND_STATUS
 */
#define HCI_CONTROL_STATUS_SUCCESS                          0
#define HCI_CONTROL_STATUS_IN_PROGRESS                      1
#define HCI_CONTROL_STATUS_ALREADY_CONNECTED                2
#define HCI_CONTROL_STATUS_NOT_CONNECTED                    3
#define HCI_CONTROL_STATUS_BAD_HANDLE                       4
#define HCI_CONTROL_STATUS_WRONG_STATE                      5
#define HCI_CONTROL_STATUS_INVALID_ARGS                     6
#define HCI_CONTROL_STATUS_FAILED                           7
#define HCI_CONTROL_STATUS_UNKNOWN_GROUP                    8
#define HCI_CONTROL_STATUS_UNKNOWN_COMMAND                  9
#define HCI_CONTROL_STATUS_CLIENT_NOT_REGISTERED            10
#define HCI_CONTROL_STATUS_OUT_OF_MEMORY                    11

/* HS open status */
#define HCI_CONTROL_HF_STATUS_SUCCESS                       0   /* Connection successfully opened */
#define HCI_CONTROL_HF_STATUS_FAIL_SDP                      1   /* Open failed due to SDP */
#define HCI_CONTROL_HF_STATUS_FAIL_RFCOMM                   2   /* Open failed due to RFCOMM */
#define HCI_CONTROL_HF_STATUS_FAIL_CONN_TOUT                3   /* Link loss occured due to connection timeout */

#ifndef BD_ADDR_LEN
#define BD_ADDR_LEN 6
#endif

#define LE_ADV_STATE_NO_DISCOVERABLE                        0
#define LE_ADV_STATE_HIGH_DISCOVERABLE                      1
#define LE_ADV_STATE_LOW_DISCOVERABLE                       2

/*
 * HID Report Channel
 */
#define HCI_CONTROL_HID_REPORT_CHANNEL_CONTROL              0
#define HCI_CONTROL_HID_REPORT_CHANNEL_INTERRUPT            1

/*
 * HID Report Type (matches BT HID Spec definitions)
 */
#define HCI_CONTROL_HID_REPORT_TYPE_OTHER                   0
#define HCI_CONTROL_HID_REPORT_TYPE_INPUT                   1
#define HCI_CONTROL_HID_REPORT_TYPE_OUTPUT                  2
#define HCI_CONTROL_HID_REPORT_TYPE_FEATURE                 3

/* Max TX packet to be sent over SPP */
#define HCI_CONTROL_SPP_MAX_TX_BUFFER                       700

/* Max GATT command packet size to be sent over uart */
#define HCI_CONTROL_GATT_COMMAND_MAX_TX_BUFFER              100

/*
 * Define status code returned in HCI_CONTROL_AIO_EVENT_COMMAND_STATUS
 */
#define HCI_CONTROL_AIO_STATUS_SUCCESS                      0   /* Command executed successfully */
#define HCI_CONTROL_AIO_STATUS_IN_PROGRESS                  1   /* Previous command in progress */
#define HCI_CONTROL_AIO_STATUS_ALREADY_CONNECTED            2   /* Already connected to server */
#define HCI_CONTROL_AIO_STATUS_NOT_CONNECTED                3   /* Not connected to server */
#define HCI_CONTROL_AIO_STATUS_CHAR_NOT_FOUND               4   /* Characteristic not found */
#define HCI_CONTROL_AIO_STATUS_DESC_NOT_FOUND               5   /* Characteristic descriptor not found */
#define HCI_CONTROL_AIO_STATUS_INVALID_ARGS                 6   /* Invalid arguments */
#define HCI_CONTROL_AIO_STATUS_FAILED                       7   /* Generic failure */

/*
 * Define AIO characteristic type
 */
#define HCI_CONTROL_AIO_CHAR_TYPE_DIGITAL                   1   /* Digital characteristic */
#define HCI_CONTROL_AIO_CHAR_TYPE_ANALOG                    2   /* Analog characteristic */
#define HCI_CONTROL_AIO_CHAR_TYPE_AGGREGATE                 3   /* Aggregate characteristic */

/*
 * Define Player Setting capabilities
 */
#define HCI_CONTROL_PLAYER_EQUALIZER_ENABLED

#define HCI_CONTROL_PLAYER_REPEAT_ENABLED
#define HCI_CONTROL_PLAYER_REPEAT_SINGLE_ENABLED
#define HCI_CONTROL_PLAYER_REPEAT_ALL_ENABLED
#define HCI_CONTROL_PLAYER_REPEAT_GROUP_ENABLED

#define HCI_CONTROL_PLAYER_SHUFFLE_ENABLED
#define HCI_CONTROL_PLAYER_SHUFFLE_ALL_ENABLED
#define HCI_CONTROL_PLAYER_SHUFFLE_GROUP_ENABLED

/*
 * IPSP/6loBTLE commands
 */
#define HCI_CONTROL_SIXLO_COMMAND_INIT                       ( ( HCI_CONTROL_GROUP_SIXLO << 8 ) | 0x01 )    /* initialize IP service */
#define HCI_CONTROL_SIXLO_COMMAND_INIT_IND                   ( ( HCI_CONTROL_GROUP_SIXLO << 8 ) | 0x02 )    /* initialization done indication */
#define HCI_CONTROL_SIXLO_COMMAND_DATA_REQ                   ( ( HCI_CONTROL_GROUP_SIXLO << 8 ) | 0x02 )    /* initialization done indication */
#define HCI_CONTROL_SIXLO_COMMAND_DATA_IND                   ( ( HCI_CONTROL_GROUP_SIXLO << 8 ) | 0x03 )    /* initialization done indication */
#define HCI_CONTROL_SIXLO_COMMAND_ND                         ( ( HCI_CONTROL_GROUP_SIXLO << 8 ) | 0x05 )    /* neighbour discovery */
#define HCI_CONTROL_SIXLO_COMMAND_RA                         ( ( HCI_CONTROL_GROUP_SIXLO << 8 ) | 0x06 )    /* router advertisement */
#define HCI_CONTROL_SIXLO_COMMAND_IPV6_PREFIX                ( ( HCI_CONTROL_GROUP_SIXLO << 8 ) | 0x05 )    /* prefix */

/*
 * IPSP/6LoBTLE Events
 */

/**
 * TODO
 */



#define MAX_IP_CONNECTIONS                                  1


typedef void (*wiced_hci_cb)(uint16_t command, uint8_t* payload, uint32_t len);

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_result_t wiced_hci_up();
wiced_result_t wiced_hci_down(void);
wiced_result_t wiced_hci_set_event_callback(control_group_t group, wiced_hci_cb evt_cb);

/**
 * Send data over the wiced_hci interface.
 *
 * @param opcode The operation code as above for commands.
 * @param data   The data to be send as per opcode
 * @param length The length of the data being sent.
 * @return WICED_SUCCESS if the operation succeeded
 *         WICED_ERROR   if the operation failed.
 */
void wiced_hci_send(uint32_t opcode, uint8_t* data, uint16_t length);
wiced_result_t wiced_hci_configure(wiced_hci_cb rx_cb);


#ifdef __cplusplus
} /* extern C */
#endif
