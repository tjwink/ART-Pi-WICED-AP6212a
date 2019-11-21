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

/**
 * @file Wiced buffer manager routines.
 */

#pragma once

#include <stdint.h>
#include "wiced_utilities.h"
#include "wiced_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#ifdef DEBUG

#define VALIDATE_POOL_HANDLE(x)                                                                     \
    if (!(x))                                                                                       \
    {                                                                                               \
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "invalid media data buffer pool handle (NULL)\n");  \
        return BUFMGR_INVALID_HANDLE;                                                               \
    }

#else /* !DEBUG */

#define VALIDATE_POOL_HANDLE(x)

#endif /* DEBUG */

/******************************************************
 *                    Constants
 ******************************************************/

#define BUFMGR_WAIT_INFINITE            WICED_WAIT_FOREVER

/* flags
 * Definition of flags in the 'flags' data member of the buffer handle. The
 * value of the 'flags' data member in the buffer handle could be a combination
 * of the following flags ORed together.
 */

#define BUFMGR_FLAG_AU_START            (1 << 0) /* Buffer starts an access unit         */
#define BUFMGR_FLAG_AU_END              (1 << 1) /* Buffer terminates an access unit     */
#define BUFMGR_FLAG_DISCONTINUITY       (1 << 2) /* Discontinuity detected in the buffer */
#define BUFMGR_FLAG_PTS_VALID           (1 << 3) /* Valid PTS associated with the buffer */
#define BUFMGR_FLAG_DTS_VALID           (1 << 4) /* Valid DTS associated with the buffer */

#define MAX_POOL_NAME_SIZE              (32)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    BUFMGR_OK                       = 0,
    BUFMGR_INVALID_HANDLE           = -1,
    BUFMGR_INVALID_PARAM            = -2,
    BUFMGR_INVALID_BUF_HANDLE       = -3,
    BUFMGR_SRCBUF_OUT_OF_RANGE      = -4,
    BUFMGR_MEM_ALLOC                = -5,
    BUFMGR_BUF_NOT_AVAIL            = -6,
    BUFMGR_BUF_HANDLE_NOT_AVAIL     = -7,
    BUFMGR_UNKNOWN_ERR              = -8
} bufmgr_status_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef void* objhandle_t;

/* Callback function to be called by the buffer pool manager when a buffer or
 * buffer handle is available.
 */

typedef void (*bufmgr_cb_t)(void* arg);

/******************************************************
 *                    Structures
 ******************************************************/

/*
 * Media data packet timestamp.
 */

typedef struct
{
    uint64_t time_val;      /* time stamp value */
    uint32_t time_scale;    /* time scale of the time stamp value */
} mtimestamp_t;

/*
 * Information for each buffer.
 */

typedef struct bufmgr_buf_info_s
{
    struct bufmgr_buf_info_s* next;

    char*    buf;
    uint32_t reference_cnt;
    void*    subclass_private;
} bufmgr_buf_info_t;

/*
 * The data buffers are not used directly. They are referenced using a buffer handle.
 *
 * When a buffer is allocated, a buffer handle will be returned which can
 * be used subsequently to access the buffer. Multiple references can be
 * made to the same buffer. However, each reference will be made using a
 * separate buffer handle, and each buffer handle will have its own context.
 * This data structure defines the context of each buffer handle. When a
 * buffer is allocated or a buffer handle is duplicated, this structure is
 * allocated from a pre-allocated pool, and then the pointer to the data
 * structure is casted to a objhandle_t type before returning to the caller.
 */

typedef struct bufmgr_buf_handle_s
{
    struct bufmgr_buf_handle_s* next;

    objhandle_t               hpool;
    uint32_t                  offset;             /* Offset to start of valid data */
    uint32_t                  size;               /* Size of valid data in buffer  */
    bufmgr_buf_info_t*        buf_info;
    uint32_t                  flags;
    mtimestamp_t              pts;
    mtimestamp_t              dts;
    void*                     subclass_private;
} bufmgr_buf_handle_t;

/*
 * Buffer pool statistics.
 */

typedef struct
{
    uint32_t num_buf_alloc;           /* Number of buffers currently allocated.                */
    uint32_t num_handle_alloc;        /* Number of buffer handles currently allocated          */
    uint32_t buf_high_mark;           /* Highest number of buffers ever allocated in the pool  */
    uint32_t handle_high_mark;        /* Highest num of buffer handles allocated in the pool   */
    uint32_t max_buf_alloc_cnt;       /* Count of number max buffer allocation reached         */
    uint32_t max_handle_alloc_cnt;    /* Count of number max buffer handle allocation reached  */
    uint32_t failed_buf_alloc_cnt;    /* Number of buffer allocation failures                  */
    uint32_t failed_handle_alloc_cnt; /* Number of buffer handle allocation failures           */
} bufmgr_stats_t;

/* Generic buffer pool data structure.
 *
 * When a buffer pool is created, this data structure shall be allocated and
 * initialized properly. The pointer to the data structure shall then be
 * cast to a objhandle_t type before returning to the caller.
 */

typedef struct
{
    char     name[MAX_POOL_NAME_SIZE];
    uint32_t poolsize;                /* Number of buffers in the pool                         */
    uint32_t handle_poolsize;         /* Number of buffer handles in the pool                  */
    uint32_t num_buf_alloc;           /* Number of buffers currently allocated                 */
    uint32_t num_handle_alloc;        /* Number of buffer handles currently allocated          */
    uint32_t prepend_bytes;           /* Number of bytes prepended at head of buffer           */
    uint32_t bufsize;                 /* Size of each buffer in the pool                       */
    uint32_t actual_bufsize;          /* Actual size of each buffer in the pool                */

    /* statistical information for the buffer pool. */
    uint32_t buf_high_mark;           /* Highest number of buffers allocated from the pool     */
    uint32_t handle_high_mark;        /* Highest num of buffer handles allocated from the pool */
    uint32_t max_buf_alloc_cnt;       /* Count of number max buffer allocation reached         */
    uint32_t max_handle_alloc_cnt;    /* Count of number max buffer handle allocation reached  */
    uint32_t failed_buf_alloc_cnt;    /* Number of buffer allocation failures                  */
    uint32_t failed_handle_alloc_cnt; /* Number of buffer handle allocation failures           */

    bufmgr_cb_t callback;             /* Callback to notify when a buffer becomes available    */
    void*       callback_arg;         /* Argument to be passed with the callback function      */

    void* subclass_private;           /* Pointer to private data                               */

    /* -------------------------------------------------------------------------+
     * |  Buffer pool interface functions
     * +-------------------------------------------------------------------------
     */

    bufmgr_status_t (*register_cb)(objhandle_t hpool, bufmgr_cb_t callback, void* cb_arg);
    bufmgr_status_t (*alloc_buf)(objhandle_t hpool, objhandle_t* hbuf, char** buf, uint32_t* size, uint32_t timeout_ms);
    bufmgr_status_t (*free_buf)(objhandle_t hpool, objhandle_t hbuf);
    bufmgr_status_t (*dup_buf)(objhandle_t  hpool, objhandle_t hbuf, objhandle_t* hnewbuf, uint32_t offset, uint32_t size, uint32_t timeout_ms);
    bufmgr_status_t (*clone_buf)(objhandle_t hpool, objhandle_t hbuf, objhandle_t* hnewbuf, uint32_t offset, uint32_t size, uint32_t timeout_ms);
    bufmgr_status_t (*copy_buf)(objhandle_t hpool, objhandle_t hbuf, char* buf, uint32_t offset, uint32_t* num_bytes);

    uint32_t        (*get_poolsize)(objhandle_t hpool);
    uint32_t        (*get_handle_poolsize)(objhandle_t hpool);
    uint32_t        (*get_bufsize)(objhandle_t hpool);
    uint32_t        (*get_actual_bufsize)(objhandle_t hpool);
    bufmgr_status_t (*get_stats)(objhandle_t hpool, bufmgr_stats_t* stats);

    bufmgr_status_t (*set_buf_flags)(objhandle_t hpool, objhandle_t hbuf, uint32_t flags);
    uint32_t        (*get_buf_flags)(objhandle_t hpool, objhandle_t hbuf);
    bufmgr_status_t (*set_buf_pts)(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* pts);
    bufmgr_status_t (*get_buf_pts)(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* pts);
    bufmgr_status_t (*set_buf_dts)(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* dts);
    bufmgr_status_t (*get_buf_dts)(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* dts);

} bufmgr_pool_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 *  Buffer pool creation function.
 *
 * @param hpool            Return handle of the created buffer pool.
 * @param name             Name of the buffer pool.
 * @param bufsize          Size of the buffer in the pool.
 * @param poolsize         Number of buffers to create for the pool.
 * @param handle_poolsize  Number of buffer handles to create for the buffer handle pool.
 * @param prepend_bytes    Number of bytes to allocate in the header of the buffer.
 * @param alignment        Buffer alignment boundary, a power of 2.
 *
 * @return @ref bufmgr_status_t
 */
bufmgr_status_t bufmgr_pool_create(objhandle_t* hpool, char* name, uint32_t bufsize, uint32_t poolsize,
                                   uint32_t handle_poolsize, uint32_t prepend_bytes, uint32_t alignment);


/**
 *  Buffer pool destroy/deallocation function.
 *
 * @param hpool     Handle for the buffer pool.
 *
 * @return @ref bufmgr_status_t
 */
bufmgr_status_t bufmgr_pool_destroy(objhandle_t hpool);


/**
 * Register a callback function to be called by the data buffer pool manager to notify
 *  the user when a buffer or buffer handle becomes available.
 *
 * @param hpool     Handle of the buffer pool.
 * @param callback  Pointer to the callback function.
 * @param cb_arg    Pointer to the argument to be passed to the callback function.
 *
 * @return @ref bufmgr_status_t
 */
static inline bufmgr_status_t bufmgr_pool_register_cb(objhandle_t hpool, bufmgr_cb_t callback, void *cb_arg)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->register_cb(hpool, callback, cb_arg));
}


/**
 *  Allocate a buffer from the buffer pool.
 *
 * This function allocate a buffer and a handle from the buffer pool.
 * The buffer handle will be initialized to point to the allocated buffer.
 *
 * @param hpool       Handle of the buffer pool.
 * @param hbuf        Address of the return buffer handle.
 * @param buf         Address of the return buffer address.
 * @param size        Address of the return buffer size.
 * @param timeout_ms  Timeout period in ms waiting for a buffer to become available.
 *
 * @return @ref bufmgr_status_t
 */
static inline bufmgr_status_t bufmgr_pool_alloc_buf(objhandle_t hpool, objhandle_t* hbuf, char** buf,
                                                    uint32_t* size, uint32_t timeout_ms)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->alloc_buf(hpool, hbuf, buf, size, timeout_ms));
}


/**
 *  Release a buffer.
 *
 * This function return a buffer to the buffer pool. It will decrement the reference count for
 * the buffer by one. If the reference count is zero, then the buffer will be returned to the pool.
 * Otherwise, only the buffer handle will be returned to the pool.
 *
 * @param hpool      Handle of the buffer pool.
 * @param hbuf       Handle of the buffer to be released.
 *
 * @return @ref bufmgr_status_t
 */
static inline bufmgr_status_t bufmgr_pool_free_buf(objhandle_t hpool, objhandle_t hbuf)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->free_buf(hpool, hbuf));
}


/**
 * Duplicate a buffer handle and re-use the same buffer as the source buffer handle.
 *
 * @param hpool      Handle of the buffer pool.
 * @param hbuf       Handle of the source buffer to duplicate.
 * @param hnewbuf    Address of the buffer handle that was duplicated.
 * @param offset     Offset from the current offset of the source buffer handle. The offset
 *                   of the buffer referenced by the new buffer handle would be offset of
 *                   the source buffer handle plus this parameter.
 * @param size       Size of the data in the buffer referenced by the new handle
 * @param timeout_ms Timeout in ms waiting for a buffer handle to become available.
 *
 * @return @ref bufmgr_status_t
 */
static inline bufmgr_status_t bufmgr_pool_dup_buf(objhandle_t hpool, objhandle_t hbuf, objhandle_t* hnewbuf,
                                                  uint32_t offset, uint32_t size, uint32_t timeout_ms)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->dup_buf(hpool, hbuf, hnewbuf, offset, size, timeout_ms));
}


/**
 * Duplicate a buffer. A new buffer and buffer handle will be allocated from the pool.
 *  The source buffer will be copied to the new buffer at starting offset of zero.
 *
 * @param hpool      Handle of the buffer pool.
 * @param hbuf       Handle of the source buffer to copy.
 * @param hnewbuf    Address of the new buffer handle.
 * @param offset     Offset from the current offset of the source buffer handle
 *                   to copy the data to the new buffer.
 * @param size       Size of the data to copy from the source to the new buffer.
 * @param timeout_ms Timeout in ms waiting for the buffer or buffer handle to become available.
 *
 * @return @ref bufmgr_status_t
 *
 * @post 'size' bytes of data is copied from (p_buf_info->p_buf + offset) of source plus
 *         offset argument to zero offset of the new buffer. Offset of the new buffer
 *         handle will be initialized to zero.
 */
static inline bufmgr_status_t bufmgr_pool_clone_buf(objhandle_t hpool, objhandle_t hbuf, objhandle_t* hnewbuf,
                                                    uint32_t offset, uint32_t size, uint32_t timeout_ms)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->clone_buf(hpool, hbuf, hnewbuf, offset, size, timeout_ms));
}


/**
 *  Copy the data from the buffer referenced by the source buffer handle
 *  to an external destination buffer.
 *
 * @param hpool      Handle of the buffer pool.
 * @param hbuf       Handle of the buffer.
 * @param dst_buf    Address of the destination buffer.
 * @param offset     Offset from the current offset of the source buffer handle to copy the data.
 * @param num_bytes  Number of bytes to copy.
 *
 * @return @ref bufmgr_status_t
 *
 * @post 'num_bytes' bytes of data is copied from
 *        (buf_info->buf + offset) of source plus
 *        offset argument to the 'dst_buf' buffer.
 */
static inline bufmgr_status_t bufmgr_pool_copy_buf(objhandle_t hpool, objhandle_t hbuf, char* dst_buf,
                                                   uint32_t offset, uint32_t* num_bytes)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->copy_buf(hpool, hbuf, dst_buf, offset, num_bytes));
}


/**
 *  Get the size of the buffer pool.
 *
 * @param hpool   Handle of the buffer pool.
 *
 * @return The number of buffers in the pool.
 */
static inline uint32_t bufmgr_pool_get_poolsize(objhandle_t hpool)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->get_poolsize(hpool));
}


/**
 *  Get the size of the buffer handle pool.
 *
 * @param hpool   Handle of the buffer pool.
 *
 * @return The number of buffer handles in the pool.
 */
static inline uint32_t bufmgr_pool_get_handle_poolsize(objhandle_t hpool)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->get_handle_poolsize(hpool));
}


/**
 *  Get the size of the buffers in the pool.
 *
 * @param hpool   Handle of the buffer pool.
 *
 * @return The size of the buffers in the pool.
 */
static inline uint32_t bufmgr_pool_get_bufsize(objhandle_t hpool)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->get_bufsize(hpool));
}


/**
 *  Get the actual size of the buffers in the pool.
 *
 * @param hpool   Handle of the buffer pool.
 *
 * @return The actual size of the buffers in the pool. This size might
 *         be larger than than the buffer size due to alignment.
 */
static inline uint32_t bufmgr_pool_get_actual_bufsize(objhandle_t hpool)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->get_actual_bufsize(hpool));
}


/**
 * Get the buffer pool statistics.
 *
 * @param hpool   Handle of the buffer pool.
 * @param stats   Address of the return buffer pool statistics.
 *
 * @return @ref bufmgr_status_t
 */
static inline bufmgr_status_t bufmgr_pool_get_stats(objhandle_t hpool, bufmgr_stats_t* stats)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->get_stats(hpool, stats));
}


/*
 * Set the flags for the buffer referenced by the buffer handle.
 *
 * @param hpool  Handle of the buffer pool.
 * @param hbuf   Handle of the buffer.
 * @param flags  Flags value.
 *
 * @return @ref bufmgr_status_t
 */
static inline bufmgr_status_t bufmgr_pool_set_buf_flags(objhandle_t hpool, objhandle_t hbuf, uint32_t flags)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->set_buf_flags(hpool, hbuf, flags));
}


/**
 * Get the flags for the buffer referenced by the buffer handle.
 *
 * @param hpool  Handle of the buffer pool.
 * @param hbuf   Handle of the buffer.
 *
 * @return The 'flags' data member of the buffer handle.
 */
static inline uint32_t bufmgr_pool_get_buf_flags(objhandle_t hpool, objhandle_t hbuf)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->get_buf_flags(hpool, hbuf));
}


/**
 * Set the PTS associated with the buffer referenced by the buffer handle.
 *
 * @param hpool Handle of the buffer pool.
 * @param hbuf  Handle of the buffer.
 * @param pts   Pointer to the PTS value.
 *
 * @return @ref bufmgr_status_t
 */
static inline bufmgr_status_t bufmgr_pool_set_buf_pts(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* pts)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->set_buf_pts(hpool, hbuf, pts));
}


/**
 * Get the PTS associated with the buffer referenced by the buffer handle.
 *
 * @param hpool Handle of the buffer pool.
 * @param hbuf  Handle of the buffer.
 * @param pts   Pointer to the return PTS value.
 *
 * @return @ref bufmgr_status_t
 */
static inline bufmgr_status_t bufmgr_pool_get_buf_pts(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* pts)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->get_buf_pts(hpool, hbuf, pts));
}


/**
 * Set the DTS associated with the buffer referenced by the buffer handle.
 *
 * @param hpool Handle of the buffer pool.
 * @param hbuf  Handle of the buffer.
 * @param dts   Pointer to the DTS value.
 *
 * @return @ref bufmgr_status_t
 */
static inline bufmgr_status_t bufmgr_pool_set_buf_dts(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* dts)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->set_buf_dts(hpool, hbuf, dts));
}


/**
 *  Get the DTS associated with the buffer referenced by the buffer handle.
 *
 * @param hpool Handle of the buffer pool.
 * @param hbuf  Handle of the buffer.
 * @param dts   Pointer to the return DTS value.
 *
 * @return @ref bufmgr_status_t
 */
static inline bufmgr_status_t bufmgr_pool_get_buf_dts(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* dts)
{
    bufmgr_pool_t* pool = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(hpool);

    return (pool->get_buf_dts(hpool, hbuf, dts));
}


/*
 * Buffer handle routines.
 */

bufmgr_status_t bufmgr_buf_free(objhandle_t hbuf);
bufmgr_status_t bufmgr_buf_dup(objhandle_t hbuf, objhandle_t* hnewbuf, uint32_t offset, uint32_t size, uint32_t timeout_ms);
bufmgr_status_t bufmgr_buf_clone(objhandle_t hbuf, objhandle_t* hnewbuf, uint32_t offset, uint32_t size, uint32_t timeout_ms);
bufmgr_status_t bufmgr_buf_copy(objhandle_t hbuf, char* dst_buf, uint32_t offset, uint32_t* num_bytes);
bufmgr_status_t bufmgr_buf_get_data(objhandle_t hbuf, char** data, uint32_t* size);
bufmgr_status_t bufmgr_buf_adjust_offset(objhandle_t hbuf, uint32_t offset);
bufmgr_status_t bufmgr_buf_set_size(objhandle_t hbuf, uint32_t size);
uint32_t        bufmgr_buf_get_size(objhandle_t hbuf);
bufmgr_status_t bufmgr_buf_get_hdr(objhandle_t hbuf, char** header);
bufmgr_status_t bufmgr_buf_get_tail(objhandle_t hbuf, char** trailer);
bufmgr_status_t bufmgr_buf_set_flags(objhandle_t hbuf, uint32_t flags);
uint32_t        bufmgr_buf_get_flags(objhandle_t hbuf);
bufmgr_status_t bufmgr_buf_set_pts(objhandle_t hbuf, mtimestamp_t* pts);
bufmgr_status_t bufmgr_buf_get_pts(objhandle_t hbuf, mtimestamp_t* pts);
bufmgr_status_t bufmgr_buf_set_dts(objhandle_t hbuf, mtimestamp_t* dts);
bufmgr_status_t bufmgr_buf_get_dts(objhandle_t hbuf, mtimestamp_t* dts);

#ifdef __cplusplus
} /* extern "C" */
#endif
