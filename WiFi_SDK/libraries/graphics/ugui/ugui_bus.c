/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library HW Function
 *
 */

#include "ugui_bus.h"
#include "ugui_bus_private.h"


/* Pointer to the current selected bus, must be STATIC */
static UG_BUS_HANDLE _ug_bus_hdl = NULL;



/******************************************************
 *               Function Definitions
 ******************************************************/


void UG_bus_init( UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle, UG_BUS_PROTO proto, void *wiced_device_bus)
{
    UG_BUS_T      bus_tmp ;
    UG_BUS_HANDLE bus_ref = NULL;
    UG_BUS_ERR    err_tmp = UG_BUS_NO_ERR;

    if(NULL==wiced_device_bus)
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_INVALID_PARAM;
        return;
    }

    if(NULL==bus_handle)
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_INVALID_PARAM;
        return;
    }

    /* only spi+i2c is supported for now, uart to come */
    if((proto != UG_BUS_PROTO_SPI) && (proto != UG_BUS_PROTO_I2C))
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_UNSUPPORTED_PROTOCOL;
        return;
    }

    /* create, init and return an handle */
    bus_ref = &bus_tmp;

    /* init bus id params */
    bus_ref->id       = 0;
    bus_ref->wmark    = UG_BUS_WMARK;
    bus_ref->type     = 0;

    /* COMPONENT_SWVER() */
    bus_ref->major    = 0;
    bus_ref->minor    = 0;

    bus_ref->protocol = proto;
    bus_ref->status   = UG_BUS_STATUS_UNKNOWN;

    switch (proto)
    {
        case UG_BUS_PROTO_SPI :
            bus_ref->spi_dev = *((wiced_spi_device_t*)wiced_device_bus);
            bus_ref->bus_msg_fn = &UG_bus_spi_msg_fn;
            break;
        case UG_BUS_PROTO_I2C :
            bus_ref->i2c_dev = *((wiced_i2c_device_t*)wiced_device_bus);
            bus_ref->bus_msg_fn = &UG_bus_i2c_msg_fn;
            break;
        case UG_BUS_PROTO_RS232 : 
            bus_ref->bus_msg_fn = &UG_bus_rs232_msg_fn;
        default :
            break;
    }

    /* open the bus and return error if needed */
    switch (proto)
    {
        case UG_BUS_PROTO_SPI :
            UG_bus_spi_init(&err_tmp, bus_ref);
            break;
        case UG_BUS_PROTO_I2C :
            UG_bus_i2c_init(&err_tmp, bus_ref);
            break;
        case UG_BUS_PROTO_RS232 :
            /* TBD */ 
        default :
            break;
    }

    /* copy on success */
    if(err_tmp==UG_BUS_NO_ERR)
    {
        bus_ref->status = UG_BUS_STATUS_READY;

        memcpy( bus_handle, bus_ref, sizeof(UG_BUS_T) );

        _ug_bus_hdl = bus_handle;
    }

    if(NULL!=(err)) *err = err_tmp;
}

void UG_bus_deinit( UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle)
{
    UG_BUS_ERR    err_tmp = UG_BUS_NO_ERR;

    if(NULL==bus_handle)
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_INVALID_PARAM;
        return;
    }

    /* open the bus and return error if needed */
    switch (bus_handle->protocol)
    {
        case UG_BUS_PROTO_SPI :
            /* not needed */
            break;
        case UG_BUS_PROTO_I2C :
            UG_bus_i2c_deinit(&err_tmp, bus_handle);
            break;
        case UG_BUS_PROTO_RS232 :
            /* TBD */ 
        default :
            break;
    }

    /* mark the bus as unavailable */
    bus_handle->status = UG_BUS_STATUS_UNKNOWN;

    if(NULL!=(err)) *err = err_tmp;
}


void UG_bus_select( UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle)
{
    UG_BUS_ERR    err_tmp = UG_BUS_NO_ERR;
    
    if(NULL!=bus_handle)
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_INVALID_PARAM;
        return;
    }

    if(UG_BUS_WMARK!=bus_handle->wmark)
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_INVALID_PARAM;
        return;
    }

    if(UG_BUS_STATUS_READY!=_ug_bus_hdl->status)
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_UNKNOWN;
        return;
    }

    _ug_bus_hdl = bus_handle;

    if(NULL!=(err)) *err = err_tmp;
}


void UG_bus_msg_fn(UG_BUS_ERR *err, UG_BUS_MSG msg_type, UG_BUS_PAYLOAD_T *arg_ptr[], uint32_t arg_len, uint32_t coalesce_flag)
{
    UG_BUS_ERR  err_tmp = UG_BUS_NO_ERR;

#if defined (DEBUG)
    if(NULL==_ug_bus_hdl)
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_UNKNOWN;
        return;
    }

    if(UG_BUS_WMARK!=_ug_bus_hdl->wmark)
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_INVALID_PARAM;
        return;
    }

    if(UG_BUS_STATUS_READY!=_ug_bus_hdl->status)
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_UNKNOWN;
        return;
    }
#endif

    if( NULL != _ug_bus_hdl->bus_msg_fn )
        _ug_bus_hdl->bus_msg_fn(&err_tmp, _ug_bus_hdl, msg_type, arg_ptr, arg_len, coalesce_flag);
    
    if(NULL!=(err)) *err = err_tmp;
}

























/******************************************************
 *               Function Definitions
 ******************************************************/

#if 0
/* These delay functions were measured with a scope on BCM943909WCD1_3.
 * They may be used for 43909 specific debugging or reference. */
#ifdef UG_I2C_USE_43909_TIMING
void UG_Delay(uint16_t number_of_milliseconds)
{
    int counter = 0;

    for (counter = 0; counter < MSEC_DELAY_COUNT_1; ++counter)
    {
        asm("nop");
    }
    if (number_of_milliseconds-- > 1)
    {
        for (counter = 0; counter < MSEC_DELAY_COUNT_FACTOR * number_of_milliseconds; ++counter)
        {
            asm("nop");
        }
    }
}

static void UG_MicroDelay(void)
{
    int counter = 0;

    for (counter = 0; counter < USEC_DELAY_COUNT_1; ++counter)
    {
        asm("nop");
    }
}

static void UG_10MicroDelay(void)
{
    int counter = 0;

    for (counter = 0; counter < USEC_DELAY_COUNT_10; ++counter)
    {
        asm("nop");
    }
}
#else
static void UG_Delay(uint16_t milliseconds)
{
    wiced_rtos_delay_milliseconds(milliseconds);
}

static void UG_MicroDelay(void)
{
    wiced_rtos_delay_microseconds(1);
}

static void UG_10MicroDelay(void)
{
    wiced_rtos_delay_microseconds(10);
}
#endif
#endif







#if 0
uint8_t ugui_com_hw_i2c_fn(void *ugui, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
    static uint8_t control = 0x13;
#ifndef UG_I2C_USE_REPEAT_START
    static uint8_t started = 0;
#endif
    wiced_result_t result;

    switch(msg)
    {
        /* Initialize I2C controller and probe device */
        case UG_COM_MSG_INIT:
            if (wiced_i2c_init(&i2c_display) != WICED_SUCCESS)
            {
                WPRINT_LIB_INFO( ("\nFailed to initialize I2C core\n") );
            }
            if (wiced_i2c_probe_device(&i2c_display, RETRIES) != WICED_TRUE)
            {
                WPRINT_LIB_INFO( ("\nI2C device not found\n") );
            }
            ugui_MicroDelay();
            break;

        /* Switch between data and command mode */
        case UG_COM_MSG_ADDRESS:
            /* Command Mode - 0x80 indicates a single command byte will be sent
             * Command Mode - 0x00 indicates a series of command bytes will be sent */
            if (arg_val == 0)
            {
#ifdef UG_I2C_USE_REPEAT_START
                control = 0x00;
                result = wiced_i2c_write(&i2c_display, WICED_I2C_START_FLAG, &control, 1);
                if (result != WICED_SUCCESS)
                {
                    WPRINT_LIB_INFO( ("Control-Byte write failed %d\n",i2c_display.address) );
                }
#else
                control = 0x80;
                started = 0;
#endif
            }
            /* Data Mode - 0x40 indicates a series of data bytes will be sent */
            else
            {
                control = 0x40;
            }
            ugui_MicroDelay();
            break;

        /* Write single command byte to device */
        case UG_COM_MSG_WRITE_BYTE:
#ifndef UG_I2C_USE_REPEAT_START
            if (!started)
            {
                wiced_i2c_write(&i2c_display, WICED_I2C_START_FLAG, &control, sizeof control);
                started = 1;
            }
            else
            {
                wiced_i2c_write(&i2c_display, 0, &control, sizeof control);
            }
#endif

            result = wiced_i2c_write(&i2c_display, 0, &arg_val, 1);
            if (result != WICED_SUCCESS)
            {
                WPRINT_LIB_INFO( ("Single-Byte write failed\n") );
            }

            ugui_MicroDelay();

            break;

        /* Write a sequence of data bytes to device */
        case UG_COM_MSG_WRITE_SEQ:
        case UG_COM_MSG_WRITE_SEQ_P:

#ifndef UG_I2C_USE_REPEAT_START
            wiced_i2c_write(&i2c_display, 0, &control, sizeof control);
#else
ziocan!
            result = wiced_i2c_write(&i2c_display, WICED_I2C_REPEATED_START_FLAG, &control, 1);
            if (result != WICED_SUCCESS)
            {
                WPRINT_LIB_INFO( ("Repeat-Start write failed\n") );
            }
#endif

            result = wiced_i2c_write(&i2c_display, WICED_I2C_STOP_FLAG, arg_ptr, arg_val);
            if (result != WICED_SUCCESS)
            {
                WPRINT_LIB_INFO( ("Multiple-Byte write failed addr=%d\n",i2c_display.address) );
            }

            ugui_MicroDelay();
            break;
    }

    return 1;
}

#endif



#if 0
void UG_bus_destroy( UG_BUS_ERR *err, UG_BUS_HANDLE *UG_bus_handle)
{
    wiced_result_t result  = WICED_SUCCESS;
    ugui_bus_t *bus_handle = NULL;

    if(NULL == (*UG_bus_handle))
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_INVALID_PARAM;
        return;
    }

    bus_handle = *((ugui_bus_t **)UG_bus_handle);

    if(UG_BUS_WMARK!=bus_handle->wmark)
    {
        if(NULL!=(err)) *err = UG_BUS_ERR_INVALID_PARAM;
        return;
    }

    switch (bus_handle->protocol)
    {
        case UG_BUS_PROTO_SPI :
            result = wiced_spi_deinit( &bus_handle->spi_dev );
            break;
        case UG_BUS_PROTO_I2C :
        case UG_BUS_PROTO_RS232 :
        default :
            break;
    }

    if(result == WICED_SUCCESS)
    {
        if(NULL!=*UG_bus_handle)
        {
            free(*UG_bus_handle);
            *UG_bus_handle = NULL;
        }
        
        if(NULL!=err)
        {
            *err = UG_BUS_NO_ERR;
        }
    }
    else
    {
        if(NULL!=err)
        {
            *err = UG_BUS_ERR_UNKNOWN;
        }
    }
}
#endif
