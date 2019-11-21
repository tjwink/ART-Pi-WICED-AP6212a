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

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                    Constants
 ******************************************************/
#define CLOUD_CONFIG WICED_REST

#define WICED_REST           1
#define WICED_AWS_BROKER     2
#define WICED_BLUEMIX_BROKER 3


#if (CLOUD_CONFIG == WICED_BLUEMIX_BROKER)

/*
 *  Change the IP address to match the server address
 * <org>.messaging.internetofthings.ibmcloud.com
 */
#define MQTT_BROKER_ADDRESS                  "<org>.messaging.internetofthings.ibmcloud.com"

/*
 * MQTT client ID
 * d:<org>:<deviceType>:<device-id>
 */
#define CLIENT_ID                            "d:<org>:meshGateway:123456"

/*
 * Publish device events to the app
 * iot-2/evt/<event-id>/fmt/<format>
 */
#define WICED_TOPIC_PUBLISH                  "iot-2/evt/mesh/fmt/json"

/*
 * Subscribe for command
 * iot-2/cmd/<cmd-id>/fmt/<format-id>
 */
#define WICED_TOPIC_SUBSCRIBE                "iot-2/cmd/meshcmd/fmt/json"

#define USERNAME                             "use-token-auth"

/*device token*/
#define PASSWORD                             "12345678"

#elif (CLOUD_CONFIG == WICED_AWS_BROKER)

#define MQTT_BROKER_ADDRESS                 "data.iot.us-east-1.amazonaws.com"

#define CLIENT_ID                           "wiced_big"

#define WICED_TOPIC_PUBLISH                 "MQTT/TOPIC/RSP/MESH"

#define WICED_TOPIC_SUBSCRIBE               "$aws/things/MESH/shadow/update/delta"

#define USERNAME                             NULL

/*device token*/
#define PASSWORD                             NULL


#endif

#ifdef __cplusplus
} /*extern "C" */
#endif
