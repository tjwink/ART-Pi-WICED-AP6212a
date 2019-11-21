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

#include <stdio.h>
#include <string.h>

#include "command_console.h"
#include "bt_mfg_test.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void print_usage_le_transmitter_test(void);
static int execute_le_transmitter_test(uint8_t chan_number, uint8_t length, uint8_t pattern);
static void print_usage_le_receiver_test(void);
static int execute_le_receiver_test(uint8_t chan_number);
static void print_usage_radio_tx_test(void);
static void print_usage_radio_rx_test(void);
static int execute_radio_tx_test(char *bdaddr, uint8_t frequency, uint8_t modulation_type, uint8_t logical_channel, uint8_t bb_packet_type, uint32_t packet_length, uint8_t tx_power);
static int execute_radio_rx_test(char *bdaddr, uint8_t frequency, uint8_t modulation_type, uint8_t logical_channel, uint8_t bb_packet_type, uint32_t packet_length, uint8_t time_period);
static int bt_hci_reset_ex(void);
static void bt_hci_display(uint8_t *data, uint8_t len);

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/*!
 ******************************************************************************
 * Send HCI Reset command.
 *
 * @param[in] argc  Unused.
 * @param[in] argv  Unused.
 *
 * @return    Console error code indicating if the command ran correctly.
 */
int bt_hci_reset( int argc, char *argv[] )
{
    bt_hci_reset_ex();
    return ERR_CMD_OK;
}

static void bt_hci_display(uint8_t *data, uint8_t len)
{
    uint8_t index = 0;
    uint8_t value;

    printf( " \n----------------------------------\n");
    for (index = 0 ; index < len ; index++)
    {
        value = *(data+index);
        printf("0x%02x ", value);
    }
    printf( " \n----------------------------------\n");
}

static int bt_hci_reset_ex(void)
{
    uint8_t hci_reset[] = {0x01, 0x03, 0x0c, 0x00};
    uint8_t hci_reset_cmd_complete_event[] = {0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00};
    wiced_result_t result;

    printf( "send HCI_RESET!\n");
    bt_hci_display(hci_reset, sizeof(hci_reset));

    result = bt_mfgtest_console_send_hci(hci_reset, sizeof(hci_reset),
            hci_reset_cmd_complete_event, sizeof(hci_reset_cmd_complete_event));
    if(result != WICED_BT_SUCCESS)
    {
        printf( "ERROR\n");
        return ERR_UNKNOWN;
    }
    printf( "SUCCESS\n");
    return ERR_CMD_OK;
}

/*!
 ******************************************************************************
 * Download BT Firmware.
 *
 * @param[in] argc  Unused.
 * @param[in] argv  Unused.
 *
 * @return    Console error code indicating if the command ran correctly.
 */
int bt_download_firmware( int argc, char *argv[] )
{
    wiced_result_t result;
    printf( "Download BT firmware\n");

    result = bt_mfgtest_console_download_fw();
    if(result != WICED_BT_SUCCESS)
    {
        printf( "ERROR \n");
        return ERR_UNKNOWN;
    }
    printf( "SUCCESS \n");
    return ERR_CMD_OK;
} /* malloc_info_command */


int bt_le_tx_test( int argc, char* argv[] )
{
    uint8_t chan_num = 0;
    uint8_t pattern = 0;
    uint8_t length = 0;

    if (argc != 4)
    {
        print_usage_le_transmitter_test();
        return ERR_CMD_OK;
    }

    chan_num = atoi(argv[1]);
    if ((chan_num >= 0) && (chan_num <= 39))
    {
        length = atoi(argv[2]);
        if ((length > 0) && (chan_num <= 255))
        {
            pattern = atoi(argv[3]);
            if ((pattern >= 0) && (pattern < 7))
            {
                return (execute_le_transmitter_test(chan_num, length, pattern));
            }
        }
    }
    print_usage_le_transmitter_test();

    return ERR_CMD_OK;
}

int bt_le_rx_test( int argc, char* argv[] )
{
    uint8_t chan_num = 0;
    if (argc != 2)
    {
        print_usage_le_receiver_test();
        return ERR_CMD_OK;
    }

    chan_num = atoi(argv[1]);
    if ((chan_num >= 0) && (chan_num <= 39))
    {
        return (execute_le_receiver_test(chan_num));
    }
    print_usage_le_receiver_test();

    return ERR_CMD_OK;
}

int bt_le_test_end( int argc, char* argv[] )
{
    wiced_result_t result;

    uint8_t hci_le_test_end[] = {0x01, 0x1f, 0x20, 0x00};
    uint8_t hci_le_test_end_cmd_complete_event[] = {0x04, 0x0e, 0x06, 0x01, \
                                                    0x1f, 0x20, 0x00};

    printf ("Send LE test end HCI Command:\n");
    bt_hci_display(hci_le_test_end, sizeof(hci_le_test_end));
    /* write HCI */
    result = bt_mfgtest_console_send_hci(hci_le_test_end, sizeof(hci_le_test_end), \
                                hci_le_test_end_cmd_complete_event, \
                                sizeof(hci_le_test_end_cmd_complete_event));
    if(result != WICED_BT_SUCCESS)
    {
        printf( "ERROR \n");
        return ERR_UNKNOWN;
    }
    printf( "SUCCESS \n");
    return ERR_CMD_OK;
}

int bt_radio_tx_test( int argc, char* argv[] )
{
    uint32_t frequency;
    uint8_t modulation_type;
    uint8_t logical_channel;
    uint8_t bb_packet_type;
    uint32_t packet_length;
    uint8_t tx_power;
    uint8_t modulation_type_mapping[] = { 0x1, //  0x00 8-bit Pattern
                                          0x2, // 0xFF 8-bit Pattern
                                          0x3, // 0xAA 8-bit Pattern
                                          0x9, // 0xF0 8-bit Pattern
                                          0x4  // PRBS9 Pattern
                                         };

    if (argc != 8)
    {
        print_usage_radio_tx_test();
        return ERR_CMD_OK;
    }

    if(strlen(argv[1])==12)
    {
        frequency = atoi(argv[2]);
        if ((frequency == 0) || ((frequency >= 2402) && (frequency <= 2480)))
        {
            modulation_type = atoi(argv[3]);
            if ((modulation_type >= 0) && (modulation_type <= 4))
            {
                logical_channel = atoi(argv[4]);
                if ((logical_channel >= 0) && (logical_channel <= 1))
                {
                     bb_packet_type = atoi(argv[5]);
                     if ((bb_packet_type >= 3) && (bb_packet_type <= 15))
                     {
                         packet_length = atoi(argv[6]);
                         if ((packet_length >= 0) && (packet_length <= 0xffff))
                         {
                             tx_power = atoi(argv[7]);
                             if ((tx_power >= -25) && (tx_power <= 3))
                             {
                                 return execute_radio_tx_test(argv[1], frequency, modulation_type_mapping[modulation_type], logical_channel, bb_packet_type, packet_length, tx_power);
                             }
                         }
                     }
                }
            }
        }
    }
    print_usage_radio_tx_test();

    return ERR_CMD_OK;
}

int bt_radio_rx_test( int argc, char* argv[] )
{
    uint32_t frequency;
    uint8_t modulation_type;
    uint8_t logical_channel;
    uint8_t bb_packet_type;
    uint32_t packet_length;
    uint8_t test_period;
    uint8_t modulation_type_mapping[] = { 0x1, //  0x00 8-bit Pattern
                                          0x2, // 0xFF 8-bit Pattern
                                          0x3, // 0xAA 8-bit Pattern
                                          0x9, // 0xF0 8-bit Pattern
                                          0x4  // PRBS9 Pattern
                                         };

    if (argc != 8)
    {
        print_usage_radio_rx_test();
        return ERR_CMD_OK;
    }

    if(strlen(argv[1])==12)
    {
        frequency = atoi(argv[2]);
        if ((frequency >= 2402) && (frequency <= 2480))
        {
            modulation_type = atoi(argv[3]);
            if ((modulation_type >= 0) && (modulation_type <= 4))
            {
                logical_channel = atoi(argv[4]);
                if ((logical_channel >= 0) && (logical_channel <= 1))
                {
                    bb_packet_type = atoi(argv[5]);
                    if ((bb_packet_type >= 3) && (bb_packet_type <= 15))
                    {
                        packet_length = atoi(argv[6]);
                        if ((packet_length >= 0) && (packet_length <= 0xffff))
                        {
                            test_period = atoi(argv[7]);
                            if((test_period >= 0) && (test_period <= 200))
                            {
                                return execute_radio_rx_test(argv[1], frequency, modulation_type_mapping[modulation_type], logical_channel, bb_packet_type, packet_length, test_period);
                            }
                        }
                    }
                }
            }
        }
    }
    print_usage_radio_rx_test();

    return ERR_CMD_OK;
}


static void print_usage_le_transmitter_test(void)
{
    printf ("Usage: bt_le_tx_test <tx_channel> <data_length> <packet_payload>\n");
    printf ("                tx_channel = Range: 0 - 39\n");
    printf ("                data_length: (0 - 37)\n");
    printf ("                data_pattern: (0 - 9)\n");
    printf ("                    0 - Pseudo-Random bit sequence 9\n");
    printf ("                    1 Pattern of alternating bits '11110000'\n");
    printf ("                    2 Pattern of alternating bits '10101010'\n");
    printf ("                    3 Pseudo-Random bit sequence 15\n");
    printf ("                    4 Pattern of All '1' bits\n");
    printf ("                    5 Pattern of All '0' bits\n");
    printf ("                    6 Pattern of alternating bits '00001111'\n");
    printf ("                    7 Pattern of alternating bits '0101'\n");
}


static int execute_le_transmitter_test(uint8_t chan_number, uint8_t length, uint8_t pattern)
{
    wiced_result_t result;
    uint8_t hci_le_transmitter_test[] = {0x01, 0x01E, 0x20, 0x03, 0x00, 0x00, 0x00};
    uint8_t hci_le_transmitter_test_cmd_complete_event[] = {0x04, 0x0e, 0x04, 0x01, \
                                                            0x01E, 0x20, 0x00};
    hci_le_transmitter_test[4] = chan_number;
    hci_le_transmitter_test[5] = length;
    hci_le_transmitter_test[6] = pattern;

    printf( "Send LE TX test command\n");

    bt_hci_display(hci_le_transmitter_test, sizeof(hci_le_transmitter_test));
    result = bt_mfgtest_console_send_hci(hci_le_transmitter_test, sizeof(hci_le_transmitter_test),
                             hci_le_transmitter_test_cmd_complete_event, \
                             sizeof(hci_le_transmitter_test_cmd_complete_event));
    if(result != WICED_BT_SUCCESS)
    {
        printf( "ERROR\n");
        return ERR_UNKNOWN;
    }
    printf( "SUCCESS\n");
    return ERR_CMD_OK;
}

static void print_usage_le_receiver_test(void)
{
    printf ("Usage: bt_le_rx_test <rx_channel>\n");
    printf ("             rx_channel = Range: 0 - 39.\n");
}


static int execute_le_receiver_test(uint8_t chan_number)
{
    wiced_result_t result;
    uint8_t hci_le_receiver_test[] = {0x01, 0x01D, 0x20, 0x01, 0x00};
    uint8_t hci_le_receiver_test_cmd_complete_event[] = {0x04, 0x0e, 0x04, 0x01,\
                                                         0x01D, 0x20, 0x00};
    hci_le_receiver_test[4] = chan_number;

    printf( "Send LE RX test command\n");

    bt_hci_display(hci_le_receiver_test, sizeof(hci_le_receiver_test));
    result = bt_mfgtest_console_send_hci(hci_le_receiver_test, sizeof(hci_le_receiver_test),\
                                    hci_le_receiver_test_cmd_complete_event, \
                                    sizeof(hci_le_receiver_test_cmd_complete_event));
    if(result != WICED_BT_SUCCESS)
    {
        printf( "ERROR\n");
        return ERR_UNKNOWN;
    }
    printf( "SUCCESS\n");
    return ERR_CMD_OK;
}

static int execute_radio_tx_test(char *bdaddr, uint8_t frequency, uint8_t modulation_type, uint8_t logical_channel, uint8_t bb_packet_type, uint32_t packet_length, uint8_t tx_power)
{
    int params[6];
    uint8_t i;
    wiced_result_t result;

    sscanf(bdaddr, "%02x%02x%02x%02x%02x%02x", &params[0], &params[1], &params[2], \
                                               &params[3], &params[4], &params[5]);

    uint8_t hci_write_bd_addr_cmd_complete_event[] = { 0x04, 0xe, 0x04, 0x01, \
                                                       0x01, 0xFC, 0x00 };
    uint8_t hci_write_bd_addr[] = { 0x01, 0x01, 0xFC, 0x06, 0, 0, 0, 0, 0, 0 };
    for(i = 0; i < 6; i++ )
    {
        hci_write_bd_addr[i+4] = params[5-i];
    }

    bt_hci_display(hci_write_bd_addr, sizeof(hci_write_bd_addr));
    result = bt_mfgtest_console_send_hci(hci_write_bd_addr, sizeof(hci_write_bd_addr),\
                                hci_write_bd_addr_cmd_complete_event, \
                                sizeof(hci_write_bd_addr_cmd_complete_event));
    if(result != WICED_BT_SUCCESS)
    {
        printf( "ERROR\n");
        return ERR_UNKNOWN;
    }
    printf( "SUCCESS\n");

    uint8_t hci_radio_tx_test[20] = {0x01, 0x051, 0xfc, 0x10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t hci_radio_tx_test_cmd_complete_event[] = {0x04, 0x0e, 0x04, 0x01, 0x051, 0xfc, 0x00};

    for(i = 0; i < 6; i++ )
    {
        hci_radio_tx_test[i+4] = params[5-i];    //bd address
    }
    hci_radio_tx_test[10] = (frequency==0) ? 0 : 1;        //0: hopping, 1: single frequency
    hci_radio_tx_test[11] = (frequency==0) ? 0 : (frequency - 2402);  //0: hopping 0-79:channel number (0: 2402 MHz)
    hci_radio_tx_test[12] = modulation_type;               //data pattern (3: 0xAA  8-bit Pattern)
    hci_radio_tx_test[13] = logical_channel;               //logical_Channel (0:ACL EDR, 1:ACL Basic)
    hci_radio_tx_test[14] = bb_packet_type;                //modulation type (BB_Packet_Type. 3:DM1, 4: DH1 / 2-DH1)
    hci_radio_tx_test[15] = packet_length & 0xff;          //low byte of packet_length
    hci_radio_tx_test[16] = (packet_length>>8) & 0xff;     //high byte of packet_length
    hci_radio_tx_test[17] = 8;                             //power in dBm
    hci_radio_tx_test[18] = tx_power;                      //dBm
    hci_radio_tx_test[19] = 0;                             //power table index

    bt_hci_display(hci_radio_tx_test, sizeof(hci_radio_tx_test));
    result = bt_mfgtest_console_send_hci(hci_radio_tx_test, sizeof(hci_radio_tx_test),\
                                hci_radio_tx_test_cmd_complete_event, \
                                sizeof(hci_radio_tx_test_cmd_complete_event));
    if(result != WICED_BT_SUCCESS)
    {
        printf( "ERROR\n");
        return ERR_UNKNOWN;
    }
    printf( "SUCCESS\n");
    return ERR_CMD_OK;
}

static void print_usage_radio_tx_test(void)
{
    printf ("Usage: bt_radio_tx_test <bd_addr> <frequency> <modulation_type> <logical_channel> <bb_packet_type> <packet_length> <tx_power>\n");
    printf ("                bd_addr: BD_ADDR of Tx device (6 bytes)\n");
    printf ("                frequency: 0 for hopping or (2402 - 2480) transmit frequency in MHz\n");
    printf ("                    0: normal Bluetooth hopping sequence (79 channels)\n");
    printf ("                    2402 - 2480: single frequency without hopping\n");
    printf ("                modulation_type: sets the data pattern\n");
    printf ("                    0: 0x00 8-bit Pattern\n");
    printf ("                    1: 0xFF 8-bit Pattern\n");
    printf ("                    2: 0xAA 8-bit Pattern\n");
    printf ("                    3: 0xF0 8-bit Pattern\n");
    printf ("                    4: PRBS9 Pattern\n");
    printf ("                logical_channel: sets the logical channel to Basic Rate (BR) or Enhanced Data Rate (EDR) for ACL packets\n");
    printf ("                    0: EDR\n");
    printf ("                    1: BR\n");
    printf ("                bb_packet_type: baseband packet type to use\n");
    printf ("                    3: DM1\n");
    printf ("                    4: DH1 / 2-DH1\n");
    printf ("                    8: 3-DH1\n");
    printf ("                    10: DM3 / 2-DH3\n");
    printf ("                    11: DH3 / 3-DH3\n");
    printf ("                    12: EV4 / 2-EV5\n");
    printf ("                    13: EV5 / 3-EV5\n");
    printf ("                    14: DM5 / 2-DH5\n");
    printf ("                    15: DH5 / 3-DH5\n");
    printf ("                packet_length: 0 - 65535. Device will limit the length to the max for the baseband packet type\n");
    printf ("                tx_power = (-25 - +3) transmit power in dbm\n");
}


static int execute_radio_rx_test(char *bdaddr, uint8_t frequency, uint8_t modulation_type, uint8_t logical_channel, uint8_t bb_packet_type, uint32_t packet_length, uint8_t test_period)
{
    int params[6];
    uint8_t i;
    wiced_result_t result;
    uint8_t in_buffer[255];

    sscanf(bdaddr, "%02x%02x%02x%02x%02x%02x", &params[0], &params[1], &params[2], &params[3], &params[4], &params[5]);
    uint8_t hci_radio_rx_test[] = {0x01, 0x52, 0xfc, 0x0e, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t hci_radio_rx_test_cmd_complete_event[] = {0x04, 0x0e, 0x04, 0x01, 0x52, 0xfc, 0x00};

    for( i = 0; i < 6; i++ )
    {
        hci_radio_rx_test[i+4] = params[5-i];
    }
    hci_radio_rx_test[10] = 0xe8;                          //low byte of report period in ms (1sec = 1000ms, 0x03e8)
    hci_radio_rx_test[11] = 0x03;                          //high byte
    hci_radio_rx_test[12] = frequency - 2402;
    hci_radio_rx_test[13] = modulation_type;               //data pattern (3: 0xAA 8-bit Pattern)
    hci_radio_rx_test[14] = logical_channel;               //logical_Channel (0:ACL EDR, 1:ACL Basic)
    hci_radio_rx_test[15] = bb_packet_type;                //modulation type (BB_Packet_Type. 3:DM1, 4: DH1 / 2-DH1)
    hci_radio_rx_test[16] = packet_length & 0xff;          //low byte of packet_length
    hci_radio_rx_test[17] = (packet_length>>8) & 0xff;     //high byte of packet_length

    bt_hci_display(hci_radio_rx_test, sizeof(hci_radio_rx_test));
    result = bt_mfgtest_console_send_hci(hci_radio_rx_test, sizeof(hci_radio_rx_test),\
                    hci_radio_rx_test_cmd_complete_event, \
                    sizeof(hci_radio_rx_test_cmd_complete_event));
    if(result != WICED_BT_SUCCESS)
    {
        printf( "ERROR\n");
        return ERR_UNKNOWN;
    }
    printf("\nRadio RX Test is running.\n");

    /*loop and handle the Rx Test statistics report until the 'q' key pressed*/
    for ( i=0 ; i < test_period ; i++)
    {
        printf("Statistics monitoring %d sec!\n",i);
        /* read statistics report*/
        result = bt_mfgtest_console_receive_hci(in_buffer, 36);
        if (result == WICED_BT_SUCCESS)
        {
            printf("Statistics Report received:\n");
            bt_hci_display(in_buffer, 36);

            if ((in_buffer[0]==0x04 && in_buffer[1]==0xFF && in_buffer[2]==0x21 && in_buffer[3]==0x07))
            {
                printf ("  [Rx Test statistics]\n");
                printf ("    Sync_Timeout_Count:     0x%x\n",in_buffer[4]|in_buffer[5]<<8|in_buffer[6]<<16|in_buffer[7]<<24);
                printf ("    HEC_Error_Count:        0x%x\n",in_buffer[8]|in_buffer[9]<<8|in_buffer[10]<<16|in_buffer[11]<<24);
                printf ("    Total_Received_Packets: 0x%x\n",in_buffer[12]|in_buffer[13]<<8|in_buffer[14]<<16|in_buffer[15]<<24);
                printf ("    Good_Packets:           0x%x\n",in_buffer[16]|in_buffer[17]<<8|in_buffer[18]<<16|in_buffer[19]<<24);
                printf ("    CRC_Error_Packets:      0x%x\n",in_buffer[20]|in_buffer[21]<<8|in_buffer[22]<<16|in_buffer[23]<<24);
                printf ("    Total_Received_Bits:    0x%x\n",in_buffer[24]|in_buffer[25]<<8|in_buffer[26]<<16|in_buffer[27]<<24);
                printf ("    Good_Bits:              0x%x\n",in_buffer[28]|in_buffer[29]<<8|in_buffer[30]<<16|in_buffer[31]<<24);
                printf ("    Error_Bits:             0x%x\n",in_buffer[32]|in_buffer[33]<<8|in_buffer[34]<<16|in_buffer[35]<<24);
            }
        }
    }

    bt_hci_reset_ex();

    /* flush uart data */
    while(result == WICED_BT_SUCCESS)
    {
        result = bt_mfgtest_console_receive_hci(in_buffer, 1);
    }
    return ERR_CMD_OK;
}

static void print_usage_radio_rx_test(void)
{
    printf ("Usage: bt_radio_rx_test <bd_addr> <frequency> <modulation_type> <logical_channel> <bb_packet_type> <packet_length> <test_period>\n");
    printf ("                bd_addr: BD_ADDR of Tx device (6 bytes)\n");
    printf ("                frequency = (2402 - 2480) receive frequency in MHz\n");
    printf ("                modulation_type: sets the data pattern\n");
    printf ("                    0: 0x00 8-bit Pattern\n");
    printf ("                    1: 0xFF 8-bit Pattern\n");
    printf ("                    2: 0xAA 8-bit Pattern\n");
    printf ("                    3: 0xF0 8-bit Pattern\n");
    printf ("                    4: PRBS9 Pattern\n");
    printf ("                logical_channel: sets the logical channel to Basic Rate (BR) or Enhanced Data Rate (EDR) for ACL packets\n");
    printf ("                    0: EDR\n");
    printf ("                    1: BR\n");
    printf ("                bb_packet_type: baseband packet type to use\n");
    printf ("                    3: DM1\n");
    printf ("                    4: DH1 / 2-DH1\n");
    printf ("                    8: 3-DH1\n");
    printf ("                    10: DM3 / 2-DH3\n");
    printf ("                    11: DH3 / 3-DH3\n");
    printf ("                    12: EV4 / 2-EV5\n");
    printf ("                    13: EV5 / 3-EV5\n");
    printf ("                    14: DM5 / 2-DH5\n");
    printf ("                    15: DH5 / 3-DH5\n");
    printf ("                packet_length: 0 - 65535. Device will limit the length to the max for the baseband packet type\n");
    printf ("                test_period : 1 ~ 200. Total test time(sec)\n");
}
