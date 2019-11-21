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

#ifndef INCLUDED_SYS_ARCH_H
#define INCLUDED_SYS_ARCH_H

//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "semphr.h"
#if defined ( IAR_TOOLCHAIN )
#include "platform_cmis.h"
#endif

#include "embos.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SYS_MBOX_NULL ((struct e_mailbox *)0)
#define SYS_SEM_NULL  ((OS_CSEMA*)0)
typedef uint32_t portTickType;

// unlike FreeRTOS EMBOS expects the mailbox buffer to be supplied externally
typedef struct e_mailbox {
    OS_MAILBOX mbox;
    void* buffer;
} e_mailbox_t;
// unlike FreeRTOS EMBOS expects the stack buffer to be supplied externally
typedef struct emos_task {
    OS_TASK task;
    OS_STACKPTR int* stack;
} emos_task_t;


#define portSTACK_TYPE  unsigned long
typedef OS_CSEMA*  /*@only@*/ sys_sem_t;
typedef e_mailbox_t*      /*@only@*/ sys_mbox_t;
typedef emos_task_t*       /*@only@*/ sys_thread_t;

uint16_t sys_rand16( void );

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* ifndef INCLUDED_SYS_ARCH_H */

