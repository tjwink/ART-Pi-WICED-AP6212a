/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library HW Function
 *
 */

#include "ugui_bus.h"
#include "ugui_bus_private.h"

#include "wiced.h"

void UG_bus_spi_init(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle)
{
    UG_BUS_ERR err_tmp = UG_BUS_NO_ERR;
    wiced_result_t result;

    result = wiced_spi_init( &bus_handle->spi_dev );
    if (result != WICED_SUCCESS)
    {
        err_tmp = UG_BUS_ERR_OPEN;
    }

    if(NULL!=(err)) *err = err_tmp;
}


void UG_bus_spi_msg_fn(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle, UG_BUS_MSG msg_type, 
                       UG_BUS_PAYLOAD_T *arg_ptr[], uint32_t arg_num, uint32_t coalesce_flag)
{
    UG_BUS_ERR err_tmp = UG_BUS_NO_ERR;
    wiced_result_t result;
    wiced_spi_message_segment_t spi_segment;

    uint32_t i = 0;

    switch(msg_type)
    {
        case UG_BUS_MSG_WRITE_SEQ:
            for(i=0; i<arg_num; i++)
            {
                spi_segment.tx_buffer = (*(arg_ptr[i])).tx_buffer;
                spi_segment.rx_buffer = (*(arg_ptr[i])).rx_buffer;
                spi_segment.length = (*(arg_ptr[i])).length;

                result = wiced_spi_transfer( &bus_handle->spi_dev, &spi_segment, 1 );
                if (result != WICED_SUCCESS)
                {
                    err_tmp = UG_BUS_ERR_IO;
                }
            }
            break;

        default:
            err_tmp = UG_BUS_NO_ERR;
    }

    if(NULL!=(err)) *err = err_tmp;
}

