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

/** @hashtable.c
 *
 * File Description
 *
 */

#include "hashtable.h"

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

wiced_app_service_cell_t* wiced_service_array[WICED_MAXIMUM_PRIORITY_LEVELS];

/******************************************************
 *               Function Definitions
 ******************************************************/

/*******************************************************************************
**
** Function         wiced_init_hashtable
**
** Description      This function initializes the service table with service
**                  entries.
**
** Returns          void
*******************************************************************************/
void wiced_init_hashtable()
{
    int i = 0;

    for(i = 0; i < WICED_MAXIMUM_PRIORITY_LEVELS; i++)
    {
        wiced_service_array[i] = NULL;
    }

}

/*******************************************************************************
**
** Function         wiced_add_entry
**
** Description      This function adds an entry to the service table.
**
** Returns          void
*******************************************************************************/
wiced_result_t wiced_add_entry(wiced_app_service_t* service)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_app_service_cell_t* cell_ptr = NULL;
    int i;

    if(service == NULL)
    {
        wiced_assert("[wiced_add_entry] Error: Service Entry is NULL", service != NULL);
        return WICED_ERROR;
    }

    cell_ptr = wiced_service_array[service->priority];

    if(cell_ptr == NULL)
    {
        if((cell_ptr = (wiced_app_service_cell_t*)malloc(sizeof(wiced_app_service_cell_t))) == NULL) {
            wiced_assert("[wiced_add_entry] Error: Service Entry is NULL", cell_ptr != NULL);
            return WICED_FALSE;
        }

        memcpy(&cell_ptr->service, service, sizeof(wiced_app_service_t));
        cell_ptr->next = NULL;
        wiced_service_array[service->priority] = cell_ptr;
    }else
    {
        for(i = 0; cell_ptr->next != NULL; i++) // check
        {
            cell_ptr = cell_ptr->next;
        }

        if(cell_ptr == NULL)
        {
            wiced_assert("[%s:%s] Error: Service Entry is NULL", cell_ptr != NULL);
            return WICED_FALSE;
        }
        cell_ptr->next = (wiced_app_service_cell_t*)malloc(sizeof(wiced_app_service_cell_t));

        if(cell_ptr->next == NULL)
            return WICED_FALSE;

        cell_ptr = cell_ptr->next;
        memcpy(&cell_ptr->service, service, sizeof(wiced_app_service_t));
        cell_ptr->next = NULL;
    }

    return result;
}

/*******************************************************************************
**
** Function         wiced_get_entry
**
** Description      This function gets an entry from the service table.
**
** Returns          pointer to a service structure.
*******************************************************************************/
wiced_app_service_t* wiced_get_entry(int service_id)
{
    int i =0;
    wiced_app_service_cell_t* cell_ptr = NULL;

    for(i = 0; i < WICED_MAXIMUM_PRIORITY_LEVELS ; i++)
    {
        cell_ptr = wiced_service_array[i];

        if(cell_ptr == NULL) continue;

        if(cell_ptr->service.type == service_id)
        {
            return &cell_ptr->service;
        }
        else
        {
            for(;cell_ptr->next != NULL;)
            {
                cell_ptr = cell_ptr->next;
                if(cell_ptr->service.type == service_id)
                {
                    return &cell_ptr->service;
                }
            }
        }

    }

    return NULL;
}
