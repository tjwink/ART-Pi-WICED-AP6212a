/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library HW Function
 *
 */

#include "ugui_bus.h"
#include "ugui_bus_private.h"


void UG_bus_rs232_init(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle)
{
    UG_BUS_ERR err_tmp = UG_BUS_NO_ERR;

    /* code here */

    if(NULL!=(err)) *err = err_tmp;
}


void UG_bus_rs232_msg_fn(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle, UG_BUS_MSG msg_type, 
                         UG_BUS_PAYLOAD_T *arg_ptr[], uint32_t arg_num, uint32_t coalesce_flag)
{
    UG_BUS_ERR err_tmp = UG_BUS_NO_ERR;

    /* code here */

    if(NULL!=(err)) *err = err_tmp;
}

