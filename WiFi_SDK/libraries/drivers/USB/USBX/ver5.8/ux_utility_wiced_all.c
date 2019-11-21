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

/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   Utility                                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"


/**
 *  USBX Device Dump Utility
 *
 *  This function briefly dump some useful USBX device information.
 *  Current usage is only for debug purpose, and we could extend the list for dump if necessary.
 *
 */
UINT _ux_utility_device_dump(UX_DEVICE *device)
{
	if(device == UX_NULL)
	{
        return (UX_ERROR);
	}

	printf("\n");

	printf("#UX#: Device:\n");
	printf("\t State: %lu\n", device->ux_device_state);
	printf("\t Address: %lu\n", device->ux_device_address);
	printf("\t HCD name: %s\n", device->ux_device_hcd->ux_hcd_name);
	printf("\t Class name: %s\n", device->ux_device_class->ux_host_class_name);
	printf("\t Speed: ");
	switch(device->ux_device_speed)
	{
		case UX_LOW_SPEED_DEVICE:
			printf("0 (LOW)\n");
			break;
		case UX_FULL_SPEED_DEVICE:
			printf("1 (FULL)\n");
			break;
		case UX_HIGH_SPEED_DEVICE:
			printf("2 (HIGH)\n");
			break;
		default:
			printf("%lu (Unknown)\n", device->ux_device_speed);
			break;
	}
	printf("\t Port Index: %lu\n", device->ux_device_port_location);
	printf("\t Max Power: %lu\n", device->ux_device_max_power);

    printf("\n");
    return (UX_SUCCESS);
}

/**
 *  Platform Data Cache Handle Utility
 *
 *  This function handle memory cache issue before DMA data transfer, if the data buffer used is in cache memory region!
 *  USBX uses both cache/non-cache memory sets for transfer requests.
 *  Basically USBX do control path transfer uses non-cache memory, while data path transfer uses cache memory.
 *  Whenever platform cache memory been used, it's necessary to handle cache issues for DMA data transfer.
 *
 */
UINT _ux_utility_platform_data_cache_handle(UX_TRANSFER *transfer_request)
{
    UCHAR           *data_pointer = UX_NULL;
    ULONG           requested_length = 0;
    UINT            request_direction = 0;

    if (transfer_request == UX_NULL)
    {
        return (UX_ERROR);
    }

    data_pointer        = transfer_request -> ux_transfer_request_data_pointer;
    requested_length    = transfer_request -> ux_transfer_request_requested_length;
    request_direction   = (transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION);

    /* Some transfer requests do not have data phase, ignore them for handle.  */
    if ((data_pointer == UX_NULL) || (requested_length == 0))
    {
        return (UX_SUCCESS);
    }

    /* Check if this transfer request data buffer is in cache memory region.  */
    if (platform_addr_is_uncached_region((void *)data_pointer))
    {
        /* If in non-cache memory region, just return.  */
        return (UX_SUCCESS);
    }

    /* Data buffer in cache memory region, need to do d-cache INVALIDATE/CLEAN based on transfer direction IN/OUT.  */
    if (request_direction == UX_REQUEST_IN)
    {
        /* DMA input transfer, clean & invalidate d-cache.  */
        platform_dcache_clean_and_inv_range((void *)data_pointer, requested_length);
        WPRINT_PLATFORM_DEBUG( ("#UX#: UX_REQUEST_IN, platform_dcache_inv_range, data_pointer = 0x%08X, len = %lu\n", (UINT)data_pointer, requested_length) );
    }
    else
    {
        /* DMA output transfer, clean d-cache.  */
        platform_dcache_clean_range((void *)data_pointer, requested_length);
        WPRINT_PLATFORM_DEBUG( ("#UX#: UX_REQUEST_OUT, platform_dcache_clean_range, data_pointer = 0x%08X, len = %lu\n", (UINT)data_pointer, requested_length) );
    }

    return (UX_SUCCESS);
}
