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

/** @file
 *
 * Quicksilver Application
 *
 * This application tests Quicksilver board features.
 *
 *
 */

#include "quicksilver.h"
#include "quicksilver_dct.h"
#include <math.h>
#include <malloc.h>
#include "wiced.h"
#include "resources.h"
#include "sntp.h"
#include "command_console.h"
#include "wiced_management.h"
#include "command_console_ping.h"
#include "wiced_usb.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define PING_TIMEOUT_MS          2000
#define PING_PERIOD_MS           3000

/******************************************************
 *                    Constants
 ******************************************************/
#define MAX_LINE_LENGTH  (128)
#define MAX_HISTORY_LENGTH (10)
#define MAX_NUM_COMMAND_TABLE  (8)

#define HTS221_SLAVE_ADDR        (0x5F)
#define HTS221_WOAMI_REG         (0x0F | 0x80)
#define HTS221_CTRL_REG1         (0x20 | 0x80)
#define HTS221_TEMP_OUT_L        (0x2A | 0x80)
#define HTS221_TEMP_OUT_H        (0x2B | 0x80)
#define HTS221_T0_DEGC_X8        (0x32 | 0x80)
#define HTS221_T1_DEGC_X8        (0x33 | 0x80)
#define HTS221_T1_T0_MSB         (0x35 | 0x80)
#define HTS221_T0_OUT_L          (0x3C | 0x80)
#define HTS221_T0_OUT_H          (0x3D | 0x80)
#define HTS221_T1_OUT_L          (0x3E | 0x80)
#define HTS221_T1_OUT_H          (0x3F | 0x80)

#define HTS221_CTRL1_PD          (0x80)
#define HTS221_CTRL1_BDU         (0x02)

#define LIS2DH12_SLAVE_ADDR      (0x19)
#define LIS2DH12_WOAMI_REG       (0x0F | 0x80)
#define LIS2DH12_CTRL_REG0       (0x1E | 0x80)
#define LIS2DH12_CTRL_REG1       (0x20 | 0x80)
#define LIS2DH12_CTRL_REG2       (0x21 | 0x80)
#define LIS2DH12_CTRL_REG3       (0x22 | 0x80)
#define LIS2DH12_CTRL_REG4       (0x23 | 0x80)
#define LIS2DH12_CTRL_REG5       (0x24 | 0x80)
#define LIS2DH12_CTRL_REG6       (0x25 | 0x80)
#define LIS2DH12_STATUS_REG      (0x27 | 0x80)
#define LIS2DH12_OUT_X_L         (0x28 | 0x80)
#define LIS2DH12_OUT_X_H         (0x29 | 0x80)
#define LIS2DH12_OUT_Y_L         (0x2A | 0x80)
#define LIS2DH12_OUT_Y_H         (0x2B | 0x80)
#define LIS2DH12_OUT_Z_L         (0x2C | 0x80)
#define LIS2DH12_OUT_Z_H         (0x2D | 0x80)

#define LIS2DH12_CTRL1_ODR_400    (0x07<<4) // 400 Hz
#define LIS2DH12_CTRL1_ODR_25     (0x03<<4) // 25 Hz
#define LIS2DH12_CTRL1_ODR_10     (0x02<<4) // 10 Hz
#define LIS2DH12_CTRL1_ODR_1      (0x01<<4) // 1 Hz
#define LIS2DH12_CTRL1_ZEN        (0x01<<2)
#define LIS2DH12_CTRL1_YEN        (0x01<<1)
#define LIS2DH12_CTRL1_XEN        (0x01)

#define LIS2DH12_CTRL4_BDU        (0x80)

#define LIS2DH12_STAT_ZYXDA       (0x01<<3)

#define NUM_I2C_MESSAGE_RETRIES   (3)

#define SPI_CLOCK_SPEED_HZ        ( 1000000 )
#define SPI_BIT_WIDTH             ( 8 )
#define SPI_MODE                  ( SPI_CLOCK_FALLING_EDGE | SPI_CLOCK_IDLE_LOW | SPI_MSB_FIRST | SPI_CS_ACTIVE_LOW )

#define MCP3208_START             (0x01<<7)
#define MCP3208_SE                (0x01<<6)
#define MCP3208_DIFF              (0x00<<6)
#define MCP3208_CH0               (0x00<<3)
#define MCP3208_CH1               (0x01<<3)
#define MCP3208_CH2               (0x02<<3)
#define MCP3208_CH3               (0x03<<3)
#define MCP3208_CH4               (0x04<<3)
#define MCP3208_CH5               (0x05<<3)
#define MCP3208_CH6               (0x06<<3)
#define MCP3208_CH7               (0x07<<3)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct
{
    wiced_semaphore_t   semaphore;      /* Semaphore used for signaling scan complete */
    uint32_t            result_count;   /* Count to measure the total scan results    */
} app_scan_data_t;

typedef struct color
{
    unsigned char Red;
    unsigned char Green;
    unsigned char Blue;
} color;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t send_ping              ( int interface );
static wiced_result_t print_wifi_config_dct ( void );
wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result );
wiced_result_t temperature_init( void );
wiced_result_t accelerometer_init( void );
wiced_result_t adc_init( void );
wiced_result_t rgb_init( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static char line_buffer[MAX_LINE_LENGTH];
static char history_buffer_storage[MAX_LINE_LENGTH * MAX_HISTORY_LENGTH];

static wiced_ip_address_t  ping_target_ip;
static wiced_usb_user_config_t usb_host_config;

static wiced_i2c_device_t i2c_device_temperature =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = HTS221_SLAVE_ADDR,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

static wiced_i2c_device_t i2c_device_accelerometer =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = LIS2DH12_SLAVE_ADDR,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

#ifdef USE_SPI0_DEVICE
static const wiced_spi_device_t spi0_device =
{
        .port        = WICED_SPI_1,
        .chip_select = WICED_GPIO_22,
        .speed       = SPI_CLOCK_SPEED_HZ,
        .mode        = SPI_MODE,
        .bits        = SPI_BIT_WIDTH
};
#endif

static const wiced_spi_device_t spi1_device =
{
        .port        = WICED_SPI_2,
        .chip_select = WICED_GPIO_NONE,
        .speed       = SPI_CLOCK_SPEED_HZ,
        .mode        = SPI_MODE,
        .bits        = SPI_BIT_WIDTH
};

static const wiced_ip_setting_t static_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168,  123,  62) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255,255,255,  0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192,168,  123,  1) ),
};

#define DIAGNOSTICS_COMMANDS \
{ "config",  wifi_config,  1, NULL, NULL, "<STA name and pass key>", "adds AP settings to DCT" }, \
{ "print_config",  print_wifi,  0, NULL, NULL,"",  "prints current wifi configuration" }, \
{ "scan",  scan_wifi,  0, NULL, NULL,"",  "scans for broadcasting wifi access points" }, \
{ "ping",  ping,  1, NULL, NULL,"<ip address>",  "pings wifi configuration" }, \
{ "temp",  temperature_get,  0, NULL, NULL,"",  "get HTS221 temperature" }, \
{ "accel",  accelerometer_get,  0, NULL, NULL,"",  "get LIS2DH12 accelerometer" }, \


/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Holder function to call printing wifi configuration in dct..
 */
int print_wifi(int argc, char* argv[]) {
    print_wifi_config_dct();
    return 0;
}

/*
 * Holder function to store AP name and Pass phrase in DCT
 */
int wifi_config( int argc, char* argv[])
{
    char* ap_name = argv[1];
    char* ap_passkey = argv[2];
    platform_dct_wifi_config_t*     dct_wifi_config          = NULL;

    /* get the read lock */
    wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( *dct_wifi_config ) );

    /* store AP name and Pass phrase in DCT*/
    strcpy((char *)&dct_wifi_config->stored_ap_list[0].details.SSID.value[0],ap_name);
    dct_wifi_config->stored_ap_list[0].details.SSID.length = strlen(ap_name);
    strcpy((char *)&dct_wifi_config->stored_ap_list[0].security_key[0],ap_passkey);
    dct_wifi_config->stored_ap_list[0].security_key_length = strlen(ap_passkey);
    wiced_dct_write( (const void*) dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );

    /* release the read lock */
    wiced_dct_read_unlock( dct_wifi_config, WICED_TRUE );
    return WICED_SUCCESS;
}

/*
 * Holder function to scan for Wi-Fi Access Points (AP)
 */
int scan_wifi(int argc, char *argv[]){
        wiced_time_t    scan_start_time;
        wiced_time_t    scan_end_time;
        app_scan_data_t scan_data;

        /* Initialize the semaphore that will tell us when the scan is complete */
        wiced_rtos_init_semaphore(&scan_data.semaphore);
        scan_data.result_count = 0;
        WPRINT_APP_INFO( ( "Waiting for scan results...\n" ) );
        WPRINT_APP_INFO( ("  # Type  BSSID             RSSI  Rate Chan  Security         SSID\n" ) );
        WPRINT_APP_INFO( ("----------------------------------------------------------------------------------------------\n" ) );

        /* Start the scan */
        wiced_time_get_time(&scan_start_time);
        wiced_wifi_scan_networks(scan_result_handler, &scan_data );

        /* Wait until scan is complete */
        wiced_rtos_get_semaphore(&scan_data.semaphore, WICED_WAIT_FOREVER);
        wiced_time_get_time(&scan_end_time);

        WPRINT_APP_INFO( ("\nScan complete in %lu milliseconds\n", (unsigned long )(scan_end_time - scan_start_time) ) );

        /* Clean up */
        wiced_rtos_deinit_semaphore(&scan_data.semaphore);

        return 0;
}

/*
 * Callback function to handle scan results
 */
wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result )
{
    /* Validate the input arguments */
    wiced_assert("Bad args", malloced_scan_result != NULL);

    if ( malloced_scan_result != NULL )
    {
        app_scan_data_t* scan_data  = (app_scan_data_t*)malloced_scan_result->user_data;

        malloc_transfer_to_curr_thread( malloced_scan_result );

        if ( malloced_scan_result->status == WICED_SCAN_INCOMPLETE )
        {
            wiced_scan_result_t* record = &malloced_scan_result->ap_details;

            WPRINT_APP_INFO( ( "%3ld ", scan_data->result_count ) );
            print_scan_result(record);
            scan_data->result_count++;
        }
        else
        {
            wiced_rtos_set_semaphore( &scan_data->semaphore );
        }

        free( malloced_scan_result );
    }
    return WICED_SUCCESS;
}

/*
 * Holder function to get HTS221 temperature
 */
int temperature_get(int argc, char *argv[]){
    wiced_result_t result = WICED_SUCCESS;
    uint8_t *wbuf;
    uint8_t *rbuf;
    int16_t T0, T1, T2, T3, raw;
    uint8_t val[4];
    int32_t temperature;
    float tempC, tempF;
    wbuf = malloc(sizeof(uint8_t)*4);
    if (wbuf == NULL)
    {
        WPRINT_APP_INFO( ("Unable to allocate wbuf\n" ));
        return result;
    }

    rbuf = malloc(sizeof(uint8_t)*4);
    if (rbuf == NULL)
    {
        WPRINT_APP_INFO( ("Unable to allocate rbuf\n" ));
        free( wbuf );
        return result;
    }

    memset(wbuf, 0, (sizeof(uint8_t)*4));
    memset(rbuf, 0, (sizeof(uint8_t)*4));

    // Temperature Calibration values
    // Read 1 byte of data from address 0x32(50)
    wbuf[0] = HTS221_T0_DEGC_X8;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    T0 = rbuf[0];

    // Read 1 byte of data from address 0x33(51)
    wbuf[0] = HTS221_T1_DEGC_X8;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    T1 = rbuf[0];

    // Read 1 byte of data from address 0x35(53)
    wbuf[0] = HTS221_T1_T0_MSB;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    raw = rbuf[0];

    // Convert the temperature Calibration values to 10-bits
    T0 = ((raw & 0x03) * 256) + T0;
    T1 = ((raw & 0x0C) * 64) + T1;

    // Read 1 byte of data from address 0x3C(60)
    wbuf[0] = HTS221_T0_OUT_L;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    // Read 1 byte of data from address 0x3D(61)
    wbuf[0] = HTS221_T0_OUT_H;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[1] = rbuf[0];

    T2 = ((val[1] & 0xFF) * 256) + (val[0] & 0xFF);

    // Read 1 byte of data from address 0x3E(62)
    wbuf[0] = HTS221_T1_OUT_L;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    // Read 1 byte of data from address 0x3F(63)
    wbuf[0] = HTS221_T1_OUT_H;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[1] = rbuf[0];

    T3 = ((val[1] & 0xFF) * 256) + (val[0] & 0xFF);

    // Read 2 bytes of data; temperature msb and lsb
    wbuf[0] = HTS221_TEMP_OUT_L;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    wbuf[0] = HTS221_TEMP_OUT_H;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[1] = rbuf[0];

    temperature = ((val[1] & 0xFF) * 256) + (val[0] & 0xFF);
    if(temperature > 32767)
    {
        temperature -= 65536;
    }

    tempC = ((T1 - T0) / 8.0) * (temperature - T2) / (T3 - T2) + (T0 / 8.0);
    tempF = (tempC * 1.8 ) + 32;

    WPRINT_APP_INFO( ( "HTS221 temperature %.1f C, %.1f F\n", tempC, tempF ) );

    free( rbuf );
    free( wbuf );

    return 0;
}

/*
 * Converts raw accelerometer data to mg.
*/
void convert_accel_data(int16_t* x, int16_t* y, int16_t* z) {
    uint16_t lx, ly, lz;

    lx = *x;
    ly= *y;
    lz= *z;

    *x = (int32_t)lx*1000/(1024*16); // transform data to millig, for 2g scale axis*1000/(1024*16),
    *y = (int32_t)ly*1000/(1024*16); // for 4g scale axis*1000/(1024*8),
    *z = (int32_t)lz*1000/(1024*16); // for 8g scale axis*1000/(1024*4)

    return;
}

/*
 * Holder function to get LIS2DH12 accelerometer
 */
int accelerometer_get(int argc, char *argv[]){
    wiced_result_t result = WICED_SUCCESS;
    uint8_t val[8];
    int16_t xdata, ydata, zdata;
    float xdataf, ydataf, zdataf;
    uint8_t *wbuf;
    uint8_t *rbuf;

    wbuf = malloc(sizeof(uint8_t)*4);
    if (wbuf == NULL)
    {
        WPRINT_APP_INFO( ("Unable to allocate wbuf\n" ));
        return result;
    }

    rbuf = malloc(sizeof(uint8_t)*4);
    if (rbuf == NULL)
    {
        WPRINT_APP_INFO( ("Unable to allocate rbuf\n" ));
        free( wbuf );
        return result;
    }

    memset(wbuf, 0, (sizeof(uint8_t)*4));
    memset(rbuf, 0, (sizeof(uint8_t)*4));

    // Read status register from address 0x27
    wbuf[0] = LIS2DH12_STATUS_REG;
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    if( val[0] & LIS2DH12_STAT_ZYXDA ) {

        // Read 1 byte of data from address 0x29
        wbuf[0] = LIS2DH12_OUT_X_H;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[1] = rbuf[0];

        // Read 1 byte of data from address 0x28
        wbuf[0] = LIS2DH12_OUT_X_L;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[0] = rbuf[0];

        // Read 1 byte of data from address 0x2B
        wbuf[0] = LIS2DH12_OUT_Y_H;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[3] = rbuf[0];

        // Read 1 byte of data from address 0x2A
        wbuf[0] = LIS2DH12_OUT_Y_L;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[2] = rbuf[0];

        // Read 1 byte of data from address 0x2D
        wbuf[0] = LIS2DH12_OUT_Z_H;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[5] = rbuf[0];

        // Read 1 byte of data from address 0x2C
        wbuf[0] = LIS2DH12_OUT_Z_L;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[4] = rbuf[0];

        xdata =(val[1]<<8 | val[0]);
        ydata =(val[3]<<8 | val[2]);
        zdata =(val[5]<<8 | val[4]);

        // Convert to mg
        convert_accel_data(&xdata, &ydata, &zdata);

        // Check for negative
        if(xdata&0x800) {
            xdata = (((~xdata&0x7ff)+1) * -1);
        }
        if(ydata&0x800) {
            ydata = (((~ydata&0x7ff)+1) * -1);
        }
        if(zdata&0x800) {
            zdata = (((~zdata&0x7ff)+1) * -1);
        }

        // Convert to g and round
        xdataf = roundf(xdata * 0.001);
        ydataf = roundf(ydata * 0.001);
        zdataf = roundf(zdata * 0.001);

        WPRINT_APP_INFO(("x: %.fg\t", xdataf));
        WPRINT_APP_INFO(("y: %.fg\t", ydataf));
        WPRINT_APP_INFO(("z: %.fg\n", zdataf));

    } else {
        WPRINT_APP_INFO(("No new XYZ data\n"));
        return 0;
    }

    free( rbuf );
    free( wbuf );
    return 0;
}

static const command_t init_commands[] = {
        DIAGNOSTICS_COMMANDS
        CMD_TABLE_END
};

/*
 * Application start/main function.
 */
void application_start( void )
{
    wiced_result_t result;

    /* Turn off GRN LED; turn on RED LED */
    wiced_gpio_output_low( WICED_LED1 );
    wiced_gpio_output_high( WICED_LED2 );

    /* Initialize the device */
    wiced_init( );

    rgb_init();

    /* Initialize the device and WICED framework */
    wiced_core_init( );

    /* Bring up the Wi-Fi client network interface */
    result = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    if ( result == WICED_SUCCESS )
    {
        /* The ping target is the gateway */
        wiced_ip_get_gateway_address( WICED_STA_INTERFACE, &ping_target_ip );

        /* send ping */
        send_ping(WICED_STA_INTERFACE);
    }
    else
    {
        WPRINT_APP_INFO(("Unable to bring up Wi-Fi network connection\n"));
    }

    /* Bring up the network on the Ethernet interface */
    result = wiced_network_up( WICED_ETHERNET_INTERFACE, WICED_USE_STATIC_IP, &static_ip_settings );

    if ( result == WICED_SUCCESS )
    {
        /* The ping target is the gateway */
        wiced_ip_get_gateway_address( WICED_ETHERNET_INTERFACE, &ping_target_ip );

        /* send ping */
        send_ping(WICED_ETHERNET_INTERFACE);
    }
    else
    {
        WPRINT_APP_INFO(("Unable to bring up wired Ethernet network connection\n"));
    }

    /* Bring up the USB host port */
    usb_host_config.host_max_class = 8;
    usb_host_config.host_max_devices = 8;
    usb_host_config.host_max_ed = 80;
    usb_host_config.host_max_hcd = 2;
    usb_host_config.host_max_iso_td = 128;
    usb_host_config.host_max_td = 32;
    usb_host_config.host_thread_stack_size = 1024;

    wiced_usb_host_init(&usb_host_config);

    /* probe for temperature device */
    temperature_init();

    /* probe for accelerometer device */
    accelerometer_init();

    /* initialize SPI for ADC */
    adc_init();

    /* enable command line interface */
    WPRINT_APP_INFO( ( "\r\nType help to know more about commands ...\r\n" ) );
    command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    console_add_cmd_table( init_commands );

    /* Turn on GRN LED; turn off RED LED */
    wiced_gpio_output_high( WICED_LED1 );
    wiced_gpio_output_low( WICED_LED2 );

    /* Start automatic time synchronization and synchronize once every day. */
    sntp_start_auto_time_sync( 1 * DAYS );

}

static wiced_result_t send_ping( int interface )
{
    uint32_t elapsed_ms;
    wiced_result_t status;

    WPRINT_APP_INFO(("Ping about to be sent\n"));

    status = wiced_ping( interface, &ping_target_ip, PING_TIMEOUT_MS, &elapsed_ms );

    if ( status == WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Ping Reply : %lu ms\n", (unsigned long)elapsed_ms ));
    }
    else if ( status == WICED_TIMEOUT )
    {
        WPRINT_APP_INFO(("Ping timeout\n"));
    }
    else
    {
        WPRINT_APP_INFO(("Ping error\n"));
    }

    return WICED_SUCCESS;
}

/*
 * Print's Wi-Fi configuration in DCT
 */
static wiced_result_t print_wifi_config_dct( void )
{
    platform_dct_wifi_config_t* dct_wifi_config = NULL;

    if ( wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( *dct_wifi_config ) ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }


    WPRINT_APP_INFO( ( "\r\n----------------------------------------------------------------\r\n\r\n") );

    /* Wi-Fi Config Section */
    WPRINT_APP_INFO( ( "Wi-Fi Config Section \r\n") );
    WPRINT_APP_INFO( ( "    device_configured               : %d \r\n", dct_wifi_config->device_configured ) );
    WPRINT_APP_INFO( ( "    stored_ap_list[0]  (SSID)       : %s \r\n", dct_wifi_config->stored_ap_list[0].details.SSID.value ) );
    WPRINT_APP_INFO( ( "    stored_ap_list[0]  (Passphrase) : %s \r\n", dct_wifi_config->stored_ap_list[0].security_key ) );
    WPRINT_APP_INFO( ( "    soft_ap_settings   (SSID)       : %s \r\n", dct_wifi_config->soft_ap_settings.SSID.value ) );
    WPRINT_APP_INFO( ( "    soft_ap_settings   (Passphrase) : %s \r\n", dct_wifi_config->soft_ap_settings.security_key ) );
    WPRINT_APP_INFO( ( "    config_ap_settings (SSID)       : %s \r\n", dct_wifi_config->config_ap_settings.SSID.value ) );
    WPRINT_APP_INFO( ( "    config_ap_settings (Passphrase) : %s \r\n", dct_wifi_config->config_ap_settings.security_key ) );
    WPRINT_APP_INFO( ( "    country_code                    : %c%c%d \r\n", ((dct_wifi_config->country_code) >>  0) & 0xff,
                                                                            ((dct_wifi_config->country_code) >>  8) & 0xff,
                                                                            ((dct_wifi_config->country_code) >> 16) & 0xffff));
    WPRINT_APP_INFO( ( "    DCT mac_address                 : ") );
    print_mac_address( (wiced_mac_t*) &dct_wifi_config->mac_address );
    WPRINT_APP_INFO( ("\r\n") );

    wiced_dct_read_unlock( dct_wifi_config, WICED_FALSE );

    return WICED_SUCCESS;
}

/*
 * Initializes I2C, probes for temperature device
 */
wiced_result_t temperature_init( void )
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t *wbuf;
    uint8_t *rbuf;

    wbuf = malloc(sizeof(uint8_t)*4);
    if (wbuf == NULL)
    {
        WPRINT_APP_INFO( ("Unable to allocate wbuf\n" ));
        return result;
    }

    rbuf = malloc(sizeof(uint8_t)*4);
    if (rbuf == NULL)
    {
        WPRINT_APP_INFO( ("Unable to allocate rbuf\n" ));
        free( wbuf );
        return result;
    }

    memset(wbuf, 0, (sizeof(uint8_t)*4));
    memset(rbuf, 0, (sizeof(uint8_t)*4));

    /* Initialize I2C */
    if ( wiced_i2c_init( &i2c_device_temperature ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "I2C Initialization Failed\n" ) );
        free( rbuf );
        free( wbuf );
        return WICED_ERROR;
    }

    /* Probe I2C bus for temperature sensor */
    if( wiced_i2c_probe_device( &i2c_device_temperature, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
        WPRINT_APP_INFO( ( "Failed to connect to temperature device; addr 0x%x\n", i2c_device_temperature.address ) );
        free( rbuf );
        free( wbuf );
        return WICED_ERROR;
    }

    wbuf[0] = HTS221_WOAMI_REG;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    if( rbuf[0] != 0xbc )
    {
        WPRINT_APP_INFO( ( "Failed to read WHOAMI from temperature device; addr 0x%x\n", i2c_device_temperature.address ) );
        free( rbuf );
        free( wbuf );
        return WICED_ERROR;
    }
    WPRINT_APP_INFO( ( "HTS221 device (0x%x) at address 0x%x\n", rbuf[0], i2c_device_temperature.address ) );

    /* Power-up the device */
    wbuf[0] = HTS221_CTRL_REG1;
    wbuf[1] = (HTS221_CTRL1_PD | HTS221_CTRL1_BDU);
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2 );

    free( wbuf );
    free( rbuf );

    return WICED_SUCCESS;
}

/*
 * Initializes I2C, probes for accelerometer device
 */
wiced_result_t accelerometer_init( void )
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t *wbuf;
    uint8_t *rbuf;

    wbuf = malloc(sizeof(uint8_t)*4);
    if (wbuf == NULL)
    {
        WPRINT_APP_INFO( ("Unable to allocate wbuf\n" ));
        return result;
    }

    rbuf = malloc(sizeof(uint8_t)*4);
    if (rbuf == NULL)
    {
        WPRINT_APP_INFO( ("Unable to allocate rbuf\n" ));
        free( wbuf );
        return result;
    }

    memset(wbuf, 0, (sizeof(uint8_t)*4));
    memset(rbuf, 0, (sizeof(uint8_t)*4));

    /* Initialize I2C */
    if ( wiced_i2c_init( &i2c_device_accelerometer ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "I2C Initialization Failed\n" ) );
        free( rbuf );
        free( wbuf );
        return WICED_ERROR;
    }

    /* Probe I2C bus for accelerometer */
    if( wiced_i2c_probe_device( &i2c_device_accelerometer, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
        WPRINT_APP_INFO( ( "Failed to connect to accelerometer device; addr 0x%x\n", i2c_device_accelerometer.address ) );
        free( rbuf );
        free( wbuf );
        return WICED_ERROR;
    }

    wbuf[0] = LIS2DH12_WOAMI_REG;
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    if( rbuf[0] != 0x33 )
    {
        WPRINT_APP_INFO( ( "Failed to read WHOAMI from accelerometer device; addr 0x%x\n", i2c_device_accelerometer.address ) );
        free( rbuf );
        free( wbuf );
        return WICED_ERROR;
    }
    WPRINT_APP_INFO( ( "LIS2DH12 device (0x%x) at address 0x%x\n", rbuf[0], i2c_device_accelerometer.address ) );

    /* Power-up the device */
    wbuf[0] = LIS2DH12_CTRL_REG1;
    wbuf[1] = 0;
    wbuf[1] = (LIS2DH12_CTRL1_ODR_400 | LIS2DH12_CTRL1_ZEN | LIS2DH12_CTRL1_YEN | LIS2DH12_CTRL1_XEN);
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2 );

    /* Set normal mode */
    wbuf[0] = LIS2DH12_CTRL_REG4;
    wbuf[1] = LIS2DH12_CTRL4_BDU;
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2 );

    free( wbuf );
    free( rbuf );

    return WICED_SUCCESS;
}

/*
 * Initializes SPI for ADC
 */
wiced_result_t adc_init( void )
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_spi_message_segment_t spi_segment;
    uint16_t *wbuf;
    uint16_t *rbuf;
    uint16_t code, val;
    float vin;

    wbuf = malloc(sizeof(uint16_t)*4);
    if (wbuf == NULL)
    {
        WPRINT_APP_INFO( ("Unable to allocate wbuf\n" ));
        return result;
    }

    rbuf = malloc(sizeof(uint16_t)*4);
    if (rbuf == NULL)
    {
        WPRINT_APP_INFO( ("Unable to allocate rbuf\n" ));
        free( wbuf );
        return result;
    }

    memset(wbuf, 0, (sizeof(uint16_t)*4));
    memset(rbuf, 0, (sizeof(uint16_t)*4));

    /* Initialize SPI1 */
    if ( wiced_spi_init( &spi1_device ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "SPI1 Initialization Failed\n" ) );
        free( rbuf );
        free( wbuf );
        return WICED_ERROR;
    }

    spi_segment.tx_buffer = (void*)wbuf;
    spi_segment.rx_buffer = (void*)rbuf;
    spi_segment.length = 4;

    wbuf[0] = MCP3208_START | MCP3208_SE | MCP3208_CH7; /* single-ended, CH7) */
    result = wiced_spi_transfer( &spi1_device, &spi_segment, 1 );
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO( ( "SPI1 Transfer Failed\n" ) );
        free( rbuf );
        free( wbuf );
        return WICED_ERROR;
    }

    // get the 12b out of the return
    code = 0;
    code = (rbuf[1]&0xff)<<8;
    code |= (rbuf[1]&0xff00)>>8;
    code = (code>>2) & 0xffff;

    // reverse bit order
    val  = ((code&0x1)<<11) | ((code&0x2)<<9) | ((code&0x4)<<7) | ((code&0x8)<<5);
    val |= ((code&0x20)<<1) | ((code&0x10)<<3);
    val |= ((code&0x80)>>3) | ((code&0x40)>>1);
    val |= ((code&0x800)>>11) | ((code&0x400)>>9) | ((code&0x200)>>7) | ((code&0x100)>>5);

    vin = (3.30 / 4096) * val;
    WPRINT_APP_INFO( ( "MCP3208 CH7 %.1fV\n", vin ) );

    free( wbuf );
    free( rbuf );

    return WICED_SUCCESS;
}

void show_color( color c)
{
    unsigned int mask = 1 <<31;
    unsigned int data_array[3] = {};
    data_array[0] = 0;
    data_array[1] = 0;
    data_array[2] = 0xFFFFFFFF;

    data_array[1] = 0;
    data_array[1] = 0b111 << 29;
    data_array[1] |= 0b00010 << 24;
    data_array[1] |= c.Blue << 16;
    data_array[1] |= c.Green << 8;
    data_array[1] |= c.Red;

    for( int i = 0; i< 3; i++ )
    {
        mask = 1<<31;

        for ( int j = 0; j<32; j++)
        {
            wiced_gpio_output_low( WICED_RGB_CLOCK );

            if ( data_array[i] & mask )
            {
                wiced_gpio_output_high( WICED_RGB_DATA );
            }
            else
            {
                wiced_gpio_output_low( WICED_RGB_DATA );
            }

            wiced_gpio_output_high( WICED_RGB_CLOCK );

            mask >>= 1; // right shift by 1
        }
    }
}

/*
 * Initializes RGB LED
 */
wiced_result_t rgb_init( void )
{
    color Rainbow[8] = {
            {255,0,0},
            {255,110,0},
            {255,255,0},
            {0,255,0},
            {0,0,255},
            {0,255,255},
            {255,0,255},
            {255,255,255}
    };

    wiced_gpio_init( WICED_RGB_CLOCK, OUTPUT_PUSH_PULL );
    wiced_gpio_init( WICED_RGB_DATA, OUTPUT_PUSH_PULL );

    wiced_gpio_output_high( WICED_RGB_CLOCK );
    wiced_gpio_output_high( WICED_RGB_DATA );

    for ( int i = 0; i< 8; i++ )
    {
        show_color(Rainbow[i]);
        wiced_rtos_delay_milliseconds(500);
    }

    wiced_gpio_output_high( WICED_RGB_CLOCK );
    wiced_gpio_output_high( WICED_RGB_DATA );

    return WICED_SUCCESS;
}
