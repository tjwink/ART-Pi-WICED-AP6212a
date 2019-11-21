/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library HW Function
 *
 */

#include "ugui_bus.h"
#include "ugui_bus_private.h"

#define I2C_RETRIES 5


static void i2c_MicroDelay(uint16_t delay)
{
    wiced_rtos_delay_microseconds(delay);
}

void UG_bus_i2c_init(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle)
{
    UG_BUS_ERR err_tmp = UG_BUS_NO_ERR;
    wiced_result_t result;
    result = wiced_i2c_init( &bus_handle->i2c_dev );
    if (result != WICED_SUCCESS)
    {
        WPRINT_LIB_INFO( ("Failed to initialize I2C core\n") );
        err_tmp = UG_BUS_ERR_OPEN;
    }

    if(err_tmp==UG_BUS_NO_ERR)
    {
        if (wiced_i2c_probe_device(&bus_handle->i2c_dev , I2C_RETRIES) != WICED_TRUE)
        {
            WPRINT_LIB_INFO( ("I2C device not found\n") );
            err_tmp = UG_BUS_ERR_OPEN;
        }
    }
    i2c_MicroDelay(10);

    if(NULL!=(err)) *err = err_tmp;
}

void UG_bus_i2c_deinit(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle)
{
    UG_BUS_ERR err_tmp = UG_BUS_NO_ERR;
    wiced_result_t result;

    result = wiced_i2c_deinit( &bus_handle->i2c_dev );
    if (result != WICED_SUCCESS)
    {
        err_tmp = UG_BUS_ERR_OPEN;
    }

    if(NULL!=(err)) *err = err_tmp;
}

void UG_bus_i2c_msg_fn(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle, UG_BUS_MSG msg_type, 
                       UG_BUS_PAYLOAD_T *arg_ptr[], uint32_t arg_num, uint32_t coalesce_flag)
{
    UG_BUS_ERR err_tmp = UG_BUS_NO_ERR;
    wiced_result_t result = WICED_SUCCESS;

    uint16_t i2c_flags = 0;
    uint32_t i = 0;

#ifndef U8G_I2C_USE_REPEAT_START
    bus_handle->i2c_repeat_start = 0;
#endif

    /* check for transaction start/stop conditions */
    i2c_flags |= (coalesce_flag & UG_BUS_MSG_COALESCE_START) ? WICED_I2C_START_FLAG : 0;
    i2c_flags |= (coalesce_flag & UG_BUS_MSG_COALESCE_STOP)  ? WICED_I2C_STOP_FLAG : 0;

    /* check for illegal payload size */


    switch(msg_type)
    {
        case UG_BUS_MSG_WRITE_SEQ:
            for(i=0; i<arg_num; i++)
            {
                uint8_t* raw_ptr = (*(arg_ptr[i])).tx_buffer;
                uint16_t raw_bytes =  (*(arg_ptr[i])).length;
    
                if(raw_bytes > 0)
                {
#if 0
#ifndef U8G_I2C_USE_REPEAT_START
                    result = wiced_i2c_write(&bus_handle->i2c_dev,  WICED_I2C_START_FLAG, &bus_handle->i2c_control, 1);
                    if (result != WICED_SUCCESS)
                    {
                        WPRINT_LIB_INFO( ("I2C: Start write failed\n") );
                        err_tmp = UG_BUS_ERR_IO;
                    }
#else
                    result = wiced_i2c_write(&bus_handle->i2c_dev, WICED_I2C_REPEATED_START_FLAG,  &bus_handle->i2c_control, 1);
                    if (result != WICED_SUCCESS)
                    {
                        WPRINT_LIB_INFO( ("I2C: Repeat-Start write failed\n") );
                        err_tmp = UG_BUS_ERR_IO;
                    }
#endif
#endif
                    if(err_tmp==UG_BUS_NO_ERR)
                    {
                        result = wiced_i2c_write(&bus_handle->i2c_dev, i2c_flags, raw_ptr, raw_bytes);
                        if (result != WICED_SUCCESS)
                        {
                            WPRINT_LIB_INFO( ("I2C: Multiple-Byte write failed\n") );
                            err_tmp = UG_BUS_ERR_IO;
                        }
                    }

                    /* NOTE: based on I2C speed we need a different time gap on STOP FLAG */
                    /* this value needs to be tuned based on */
                    /* bus_handle->i2c_dev.speed_mode [I2C_HIGH_SPEED_MODE|I2C_STANDARD_SPEED_MODE] */
                    uint16_t delay = (i2c_flags & UG_BUS_MSG_COALESCE_STOP) ? 100 : 1;
                    i2c_MicroDelay(delay);

                }//raw_bytes>0
            }
            break;

        default:
            err_tmp = UG_BUS_NO_ERR;
    }
    if(NULL!=(err)) *err = err_tmp;
}

