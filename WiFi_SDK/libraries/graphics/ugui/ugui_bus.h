/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library Header
 *
 */

#ifndef _UGUI_BUS_H
#define _UGUI_BUS_H

#include "wiced.h"

/**
 * @brief UG_BUS_ERR: Error types returned by BUS funtion calls
 */
typedef enum{
    UG_BUS_NO_ERR = 0,
    UG_BUS_ERR_UNKNOWN,
    UG_BUS_ERR_OPEN,
    UG_BUS_ERR_BUSY,
    UG_BUS_ERR_INVALID_PARAM,
    UG_BUS_ERR_UNSUPPORTED_PROTOCOL,
    UG_BUS_ERR_OUT_OF_MEMORY,
    UG_BUS_ERR_IO,
    /**/
    UGUI_BUS_ERR_MAX
}UG_BUS_ERR;


/**
 * @brief UG_BUS_PROTO: list of protocols available for uGUI
 */
typedef enum {
    /* virtual bus for debug */
    UG_BUS_PROTO_VIRTUAL = 0,
    /* virtual empty display */
    UG_BUS_PROTO_I2C,
    UG_BUS_PROTO_SPI,
    UG_BUS_PROTO_RS232,
    UG_BUS_PROTO_RS485,
    UG_BUS_PROTO_CAN,
    /**/
    UG_BUS_PROTO_MAX
}UG_BUS_PROTO;

/**
 * @brief UG_BUS_STATUS: values for status of the bus obj
 */
typedef enum {
    UG_BUS_STATUS_UNKNOWN = 0,
    UG_BUS_STATUS_READY,
    /**/
    UG_BUS_STATUS_MAX
}UG_BUS_STATUS;

/**
 * @brief UG_BUS_MSG: message types for bus function calls
 */
typedef enum {
    UG_BUS_MSG_WRITE_SEQ = 0,
    /**/
    UG_BUS_MSG_MAX
}UG_BUS_MSG;

/**
 * @brief UG_BUS_MSG: transaction start/stop for multipe msg
 */
typedef enum {
    UG_BUS_MSG_COALESCE_NONE   = 0,
    UG_BUS_MSG_COALESCE_START  = (1<<1),
    UG_BUS_MSG_COALESCE_STOP   = (1<<2),
    UG_BUS_MSG_COALESCE_CHUNK  = (1<<3),
    /**/
    UG_BUS_MSG_COALESCE_MAX
}UG_BUS_MSG_COALESCE_FLAG;

/**
 * @brief UG_BUS_MSG: message payload for bus function calls
 */
typedef struct ugui_bus_payload_t{
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    uint32_t length;
}UG_BUS_PAYLOAD_T;

typedef struct ugui_bus_t_{
    /* COMPONENT_TAG() */
    uint32_t           id;
    uint32_t           wmark;
    uint32_t           type;

    /* COMPONENT_SWVER() */
    uint8_t            major;
    uint8_t            minor;
    char               revision[ 32 ];
    char               build[ 5 ];

    /* COMPONENT_DBG() */
    uint8_t            log_lvl;
    uint8_t            log_err;
    char               label[ 64 ];

    /* COMPONENT_ATTRIBUTES() */
    UG_BUS_PROTO       protocol;
    UG_BUS_STATUS      status;     /* internal use ONLY */
    void (*bus_msg_fn)(UG_BUS_ERR *err, struct ugui_bus_t_ *bus_handle, UG_BUS_MSG msg_type, 
                       UG_BUS_PAYLOAD_T *arg_ptr[], uint32_t arg_num, uint32_t coalesce_flag);
    wiced_spi_device_t spi_dev;
    wiced_i2c_device_t i2c_dev;

    /* internals */
    //uint8_t            i2c_control;
    uint8_t            i2c_repeat_start;
}UG_BUS_T;


typedef UG_BUS_T* UG_BUS_HANDLE;


/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * @brief initialize bus structure for a specific config
 *
 * This function will take an externally alocated bus_handle
 * and initialize his structure based on the given protocol and wiced bus
 *
 * @param[out] err the (optional) error return code, returned if not NULL 
 * @param[in,out] bus_handle the pointer to the bus obj
 * @param[in] protocol to be used for this bus obj 
 * @param[in] wiced_device_bus bus to be used for low level communication
 */
void UG_bus_init( UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle, UG_BUS_PROTO proto, void *wiced_device_bus);

/**
 * @brief de-initialize bus structure for a specific config
 *
 * This function will take an externally alocated bus_handle
 * and de-initialize his structure based on the given protocol and wiced bus
 *
 * @param[out] err the (optional) error return code, returned if not NULL 
 * @param[in,out] bus_handle the pointer to the bus obj
 */
void UG_bus_deinit( UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle);

/**
 * @brief select the bus obj to be used for next calls
 * 
 * @param[in] err the (optional) error return code, returned if not NULL 
 * @param[in] bus_handle the pointer to the bus obj
 */
void UG_bus_select( UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle);

/**
 * @brief senda message on the bus 
 * 
 * @param[in] err the (optional) error return code, returned if not NULL 
 * @param[in] msg_type code to select the bus instruction
 * @param[in] arg_ptr list of payloads to be transfered on the bus (sequentially)
 * @param[in] arg_num number of payloads in the above list
 */
void UG_bus_msg_fn(UG_BUS_ERR *err, UG_BUS_MSG msg_type, UG_BUS_PAYLOAD_T *arg_ptr[], uint32_t arg_num, uint32_t coalesce_flag);



#endif
