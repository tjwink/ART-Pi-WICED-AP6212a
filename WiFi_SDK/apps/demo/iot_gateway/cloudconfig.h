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

#ifdef __cplusplus
extern "C"
{
#endif

#define CLOUD_HOST_NAME_COAP                    "coap.exosite.com"
#define CLOUD_HOST_NAME_HTTP                    "m2.exosite.com"
#define CLOUD_HOST_KEY                          "8aa3ffa7f05f0c06e70989350881cb905aea9a26"
#define CLOUD_CLIENT_KEY                        "f758d171edf28953da56739ccbfe2da514a082b7"
#define CLOUD_DATAPORT_KEY_COAP                 "4699422864f5e3584c940e1cab3b563f368b9fd3"
#define CLOUD_DATAPORT_KEY_HTTP                 "7bc73f2596930358d4ed3f61e1ba54f859a5ea7d"
#define CLOUD_DATAPORT_ALIAS_COAP               "coaptest"
#define CLOUD_DATAPORT_ALIAS_HTTP               "httptest"
#define CLOUD_DATAPORT_ALIAS_CONTROL            "control"

#ifdef CLOUD_HOST_CARRIOTS
#define CLOUD_HTTP_URI                          "/streams/ HTTP/1.1"
#else
#define CLOUD_HTTP_URI                          "/api:v1/rpc/process  HTTP/1.1"
#endif

#ifdef CLOUD_HOST_CARRIOTS
#define CLOUD_API_KEY                           "a9103bfc9779cc09d7b7f96add9aa06c9dcd0604a5331806ed43e6fb5632813b"
//#define SENSOR_DATA                           "{\"temp\":\"41\", \"humd\":\"68\",\"accel\":\"123\",\"gyro\":\"132\",\"light\":\"10\"}"
#define SENSOR_DATA                             "{\"x\":1,\"y\":2,\"z\":3}"
#define SENSOR_ID                               "sensors@wwear20739.wwear20739"
#endif

#define HTTP_POST_REQ_LENGTH                    1000
#define BUFFER_LENGTH                           (500)
#define CLOUD_OBSERVE_TIMER                     10000

#ifdef __cplusplus
}
#endif
