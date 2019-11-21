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

#ifndef  __ALIYUN_COMMON_H__
#define  __ALIYUN_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                    Constants
 ******************************************************/
#define XML_QUEUE                       "Queue"
#define XML_MESSAGE                     "Message"
#define XML_MESSAGES                    "Messages"
#define XML_VISIBILITY_TIMEOUT          "VisibilityTimeout"
#define XML_MAXIMUM_MESSAGE_SIZE        "MaximumMessageSize"
#define XML_MESSAGE_RETENTION_PERIOD    "MessageRetentionPeriod"
#define XML_MESSAGE_BODY                "MessageBody"
#define XML_PRIORITY                    "Priority"
#define XML_DELAY_SECONDS               "DelaySeconds"
#define XML_RECEIPT_HANDLES             "ReceiptHandles"
#define XML_RECEIPT_HANDLE              "ReceiptHandle"

#define HTTP_HEADER_AUTHORIZATION       "Authorization: "
#define HTTP_HEADER_XMNS_VERSION        "x-mns-version:"
#define HTTP_HEADER_XMNS_MARKER         "x-mns-marker:"
#define HTTP_HEADER_XMNS_RET_NUMBER     "x-mns-ret-number:"
#define HTTP_HEADER_XMNS_PREFIX         "x-mns-prefix:"

#define HTTP_HEADERS_NUM_MAX                (12)

#define DEFAULT_HTTP_VERSION                (HTTP_1_1)

#define XMNS_BUFFER_SIZE                    (128)
#define DATE_BUFFER_SIZE                    (30)
#define SIGNATURE_PLAINTEXT_BUFFER_SIZE     (384)
#define SIGNATURE_BUFFER_SIZE               (64)
#define REQUEST_BUFFER_SIZE                 (2048)
#define RESOURCE_BUFFER_SIZE                (128)
#define HOST_BUFFER_SIZE                    (128)
#define RESPONSE_BUFFER_SIZE                (2048)
#define AUTHORIZATION_BUFFER_SIZE           (128)
#define CONTENT_LENGTH_BUFFER_SIZE          (8)

#define XML_BUFFER_SIZE                     (1024)

#define SHA1_LENGTH                         (20)
#define MD5_LENGTH                          (16)

#define SECS_MS                             (1000)
#define MINS_MS                             (60*SECS_MS)
#define HOURS_MS                            (60*MINS_MS)
#define DAYS_MS                             (24*HOURS_MS)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  /* __ALIYUN_COMMON_H__    */
