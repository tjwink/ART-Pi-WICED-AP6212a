/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library HW Function
 *
 */

#include "ugui_bus.h"
#include "ugui_bus_private.h"


void UG_bus_virtual_init(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle)
{
    UG_BUS_ERR err_tmp = UG_BUS_NO_ERR;

    /* code here */

    if(NULL!=(err)) *err = err_tmp;
}


void UG_bus_virtual_msg_fn(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle, UG_BUS_MSG msg_type, void *arg_ptr, uint32_t arg_len)
{
    UG_BUS_ERR err_tmp = UG_BUS_NO_ERR;

    /* code here */

    if(NULL!=(err)) *err = err_tmp;
}

