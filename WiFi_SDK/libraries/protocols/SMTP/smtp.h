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

#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *            Enumerations
 ******************************************************/

/* Email encryption */
typedef enum
{
    WICED_EMAIL_NO_ENCRYPTION,  /* Email is not encrypted when transmitted over the network       */
    WICED_EMAIL_ENCRYPTION_TLS, /* Email is encrypted using TLS when transmitted over the network */
} wiced_email_encryption_t;

/******************************************************
 *             Structures
 ******************************************************/

/* Email account structure */
typedef struct
{
    /* Public */
    char*                    email_address;    /* Pointer to this account's email address  */
    char*                    user_name;        /* Pointer to this account's user name      */
    char*                    password;         /* Pointer to this account's password       */
    char*                    smtp_server;      /* Pointer to SMTP server address string    */
    uint16_t                 smtp_server_port; /* SMTP server port                         */
    wiced_email_encryption_t smtp_encryption;  /* Outgoing mail encryption                 */

    /* Private. Internal use only */
    struct wiced_email_account_internal* internal;
} wiced_email_account_t;

/* Email structure */
typedef struct
{
    /* Public */
    char*    to_addresses;     /* Recipients' email addresses separated by commas */
    char*    cc_addresses;     /* Cc email addresses separated by commas          */
    char*    bcc_addresses;    /* Bcc email addresses separated by commas         */
    char*    subject;          /* Email subject                                   */
    char*    content;          /* Email content/body                              */
    uint32_t content_length;   /* Length of email content/body                    */
    char*    signature;        /* Signature content                               */
    uint32_t signature_length; /* Signature length                                */

    /* Private. Internal use only */
    struct wiced_email_internal* internal;
} wiced_email_t;

/******************************************************
 *             Function declarations
 ******************************************************/

wiced_result_t wiced_smtp_account_init  ( wiced_email_account_t* account );
wiced_result_t wiced_smtp_account_deinit( wiced_email_account_t* account );
wiced_result_t wiced_smtp_send          ( wiced_email_account_t* account, const wiced_email_t* email );

#ifdef __cplusplus
} /* extern "C" */
#endif
