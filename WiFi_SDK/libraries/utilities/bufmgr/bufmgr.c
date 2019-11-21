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
 * @file Buffer manager routines.
 */

#include <string.h>

#include "wiced_result.h"
#include "wiced_rtos.h"
#include "wiced_log.h"
#include "bufmgr.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define ALIGN_BUF_SIZE(x, a)      (a ? (((x) + ((a) - 1)) & ~((a) - 1)) : x)

#define BUF_AVAIL(x)              ((x)->num_buf_alloc < (x)->poolsize)
#define BUF_HANDLE_AVAIL(x)       ((x)->num_handle_alloc < (x)->handle_poolsize)

#define PRIV(c)                   ((bufmgr_pool_priv_t *)(c)->subclass_private)

#ifdef DEBUG

#define VALIDATE_BUF_HANDLE(c, h)                                                                   \
    if ((!(h)) ||                                                                                   \
        ((uintptr_t)(h) < (uintptr_t)&PRIV((c))->buf_handle[0]) ||                                  \
        ((uintptr_t)(h) > (uintptr_t)&PRIV((c))->buf_handle[(c)->handle_poolsize - 1]) ||           \
        (((uintptr_t)(h) - (uintptr_t)&PRIV((c))->buf_handle[0]) % sizeof(bufmgr_buf_handle_t)))    \
    {                                                                                               \
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "invalid media data buffer handle\n");              \
        return BUFMGR_INVALID_BUF_HANDLE;                                                           \
    }

#define ASSERT_HANDLE(h)                                                                            \
    if (!(h))                                                                                       \
    {                                                                                               \
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "invalid media data buffer handle (NULL)\n");       \
        return BUFMGR_INVALID_BUF_HANDLE;                                                           \
    }

#else /* !DEBUG */

#define VALIDATE_BUF_HANDLE(x, h)
#define ASSERT_HANDLE(h)

#endif /* DEBUG */

#define BUFFER_AVAIL_EVENT (0x1)

#define WAIT_BUFFER_AVAIL_EVENT(timeout) \
    wiced_rtos_wait_for_event_flags(&priv->buf_avail_signal, BUFFER_AVAIL_EVENT, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, timeout)

#define SET_BUFFER_AVAIL_EVENT() \
    wiced_rtos_set_event_flags(&priv->buf_avail_signal, BUFFER_AVAIL_EVENT)

#define CLEAR_BUFFER_AVAIL_EVENT() \
        WAIT_BUFFER_AVAIL_EVENT(WICED_NO_WAIT)

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

/*
 * Private data structure for the buffer pool implementation.
 */

typedef struct
{
    char*                   buf_base_addr;
    bufmgr_buf_info_t*      buf_info;
    bufmgr_buf_handle_t*    buf_handle;
    bufmgr_buf_info_t*      free_buf;
    bufmgr_buf_handle_t*    free_handle;
    wiced_mutex_t           lock;
    wiced_event_flags_t     buf_avail_signal;
} bufmgr_pool_priv_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

static bufmgr_status_t register_cb(objhandle_t hpool, bufmgr_cb_t callback, void* cb_arg)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(ctx);
    ctx->callback     = callback;
    ctx->callback_arg = cb_arg;

    return BUFMGR_OK;
}


static inline void grab_buf(bufmgr_pool_t* ctx, bufmgr_pool_priv_t* priv,
                            bufmgr_buf_info_t** buf_info, bufmgr_buf_handle_t** handle)
{
    /* get a buffer from the free buffer list */
    *buf_info      = priv->free_buf;
    priv->free_buf = priv->free_buf->next;
    ctx->num_buf_alloc++;
    if (ctx->num_buf_alloc > ctx->buf_high_mark)
    {
        ctx->buf_high_mark = ctx->num_buf_alloc;
    }
    if (ctx->num_buf_alloc == ctx->poolsize)
    {
        ctx->max_buf_alloc_cnt++;
    }

    /* get a handle from the free handle list */
    *handle           = priv->free_handle;
    priv->free_handle = priv->free_handle->next;
    ctx->num_handle_alloc++;
    if (ctx->num_handle_alloc > ctx->handle_high_mark)
    {
        ctx->handle_high_mark = ctx->num_handle_alloc;
    }
    if (ctx->num_handle_alloc == ctx->handle_poolsize)
    {
        ctx->max_handle_alloc_cnt++;
    }
}


static bufmgr_status_t alloc_buf(objhandle_t hpool, objhandle_t* hbuf, char** buf, uint32_t* size, uint32_t timeout_ms)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_pool_priv_t* priv;
    bufmgr_buf_info_t* buf_info = NULL;
    bufmgr_buf_handle_t* handle = NULL;
    bufmgr_status_t rc = BUFMGR_OK;
    uint32_t events;

    UNUSED_PARAMETER(events);
    VALIDATE_POOL_HANDLE(ctx);
    priv = PRIV(ctx);

    wiced_rtos_lock_mutex(&priv->lock);
    if (timeout_ms == 0)
    {
        if (BUF_AVAIL(ctx) && BUF_HANDLE_AVAIL(ctx))
        {
            grab_buf(ctx, priv, &buf_info, &handle);
        }
        else if (!(BUF_AVAIL(ctx)))
        {
            rc = BUFMGR_BUF_NOT_AVAIL;
        }
        else if (!(BUF_HANDLE_AVAIL(ctx)))
        {
            rc = BUFMGR_BUF_HANDLE_NOT_AVAIL;
        }
    }
    else if (timeout_ms == BUFMGR_WAIT_INFINITE)
    {
        /* wait for the buffer handle and a buffer to become available */
        while (!(BUF_AVAIL(ctx)) || !(BUF_HANDLE_AVAIL(ctx)))
        {
            /* clear the flag now as there are currently no available buffer or no available buffer handle */
            CLEAR_BUFFER_AVAIL_EVENT();
            wiced_rtos_unlock_mutex(&priv->lock);
            WAIT_BUFFER_AVAIL_EVENT(timeout_ms);
            wiced_rtos_lock_mutex(&priv->lock);
        }

        /* get a buffer from the free buffer list */
        grab_buf(ctx, priv, &buf_info, &handle);
    }
    else
    {
        /* wait for the buffer handle and a buffer to become available */
        if (!(BUF_AVAIL(ctx)) || !(BUF_HANDLE_AVAIL(ctx)))
        {
            /* clear the flag now as there is no available buffer or no available buffer handle */
            CLEAR_BUFFER_AVAIL_EVENT();
            wiced_rtos_unlock_mutex(&priv->lock);
            WAIT_BUFFER_AVAIL_EVENT(timeout_ms);
            wiced_rtos_lock_mutex(&priv->lock);

            if (!(BUF_AVAIL(ctx)))
            {
                ctx->failed_buf_alloc_cnt++;
                rc = BUFMGR_BUF_NOT_AVAIL;
            }
            if (!(BUF_HANDLE_AVAIL(ctx)))
            {
                ctx->failed_handle_alloc_cnt++;
                rc = BUFMGR_BUF_HANDLE_NOT_AVAIL;
            }
            if (rc != BUFMGR_OK)
            {
                wiced_rtos_unlock_mutex(&priv->lock);
                return rc;
            }
        }

        /* get a buffer from the free buffer list */
        grab_buf(ctx, priv, &buf_info, &handle);
    }

    wiced_rtos_unlock_mutex(&priv->lock);

    if ( (rc == BUFMGR_OK) && (handle != NULL) && (buf_info != NULL) )
    {
        /* initialize the buffer handle */
        buf_info->reference_cnt = 1;
        handle->buf_info        = buf_info;
        handle->offset          = 0;
        handle->size            = ctx->bufsize;
        handle->flags           = 0;
        memset((void*)&handle->pts, 0, sizeof(mtimestamp_t));
        memset((void*)&handle->dts, 0, sizeof(mtimestamp_t));

        *hbuf = (objhandle_t)handle;
        *buf  = buf_info->buf;
        *size = ctx->bufsize;
    }

    return rc;
}


static bufmgr_status_t free_buf(objhandle_t hpool, objhandle_t hbuf)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_pool_priv_t* priv;
    bufmgr_buf_info_t* buf_info;
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;
    uint32_t num_buf_alloc;
    uint32_t num_handle_alloc;

    VALIDATE_POOL_HANDLE(ctx);
    VALIDATE_BUF_HANDLE(ctx, handle);
    priv = PRIV(ctx);

    wiced_rtos_lock_mutex(&priv->lock);

    num_handle_alloc = ctx->num_handle_alloc;
    num_buf_alloc    = ctx->num_buf_alloc;

    buf_info = handle->buf_info;
    buf_info->reference_cnt--;

    /* add the released handle to the free list */
    handle->next      = priv->free_handle;
    priv->free_handle = handle;
    ctx->num_handle_alloc--;

    /* add the released buffer to the free list */
    if (buf_info->reference_cnt == 0)
    {
        buf_info->next = priv->free_buf;
        priv->free_buf = buf_info;
        ctx->num_buf_alloc--;
    }

    /* set the flag now that there's 1 buffer or 1 buffer handle available */
    if ((num_handle_alloc == ctx->handle_poolsize) || (num_buf_alloc == ctx->poolsize))
    {
        if (ctx->callback == NULL)
        {
            SET_BUFFER_AVAIL_EVENT();
        }
    }

    wiced_rtos_unlock_mutex(&priv->lock);

    /*
     * Check to see if the buffer pool or buffer handle pool was
     * fully utilized and signal to wake up any thread waiting waiting
     * for a buffer handle or buffer to become available.
     */

    if ((num_handle_alloc == ctx->handle_poolsize) || (num_buf_alloc == ctx->poolsize))
    {
        if (ctx->callback != NULL)
        {
            ctx->callback(ctx->callback_arg);
        }
    }

    return BUFMGR_OK;
}


static bufmgr_status_t dup_buf(objhandle_t hpool, objhandle_t hbuf, objhandle_t* hnewbuf,
                               uint32_t offset, uint32_t size, uint32_t timeout_ms)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_pool_priv_t* priv;
    bufmgr_buf_handle_t* src_handle = (bufmgr_buf_handle_t*)hbuf;
    bufmgr_buf_handle_t* handle;
    uint32_t events;

    UNUSED_PARAMETER(events);
    VALIDATE_POOL_HANDLE(ctx);
    VALIDATE_BUF_HANDLE(ctx, src_handle);

    /*
     * Validate the specified input size is within the size of the
     * buffer in the pool
     */

    if (size > ctx->bufsize - (src_handle->offset + offset))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "invalid size\n");
        return BUFMGR_SRCBUF_OUT_OF_RANGE;
    }

    priv = PRIV(ctx);

    wiced_rtos_lock_mutex(&priv->lock);
    if (timeout_ms == 0)
    {
        if (BUF_HANDLE_AVAIL(ctx))
        {
            /* get a handle from the free handle list */
            handle            = priv->free_handle;
            priv->free_handle = priv->free_handle->next;
            ctx->num_handle_alloc++;
            if (ctx->num_handle_alloc > ctx->handle_high_mark)
            {
                ctx->handle_high_mark = ctx->num_handle_alloc;
            }
            if (ctx->num_handle_alloc == ctx->handle_poolsize)
            {
                ctx->max_handle_alloc_cnt++;
            }
        }
        else
        {
            wiced_rtos_unlock_mutex(&priv->lock);
            return BUFMGR_BUF_HANDLE_NOT_AVAIL;
        }
    }
    else if (timeout_ms == BUFMGR_WAIT_INFINITE)
    {
        /* wait for the buffer handle to become available */
        while (!(BUF_HANDLE_AVAIL(ctx)))
        {
            /* clear the flag now as there is no available buffer handle */
            CLEAR_BUFFER_AVAIL_EVENT();
            wiced_rtos_unlock_mutex(&priv->lock);
            WAIT_BUFFER_AVAIL_EVENT(timeout_ms);
            wiced_rtos_lock_mutex(&priv->lock);
        }

        /* get a handle from the free handle list */
        handle            = priv->free_handle;
        priv->free_handle = priv->free_handle->next;
        ctx->num_handle_alloc++;
        if (ctx->num_handle_alloc > ctx->handle_high_mark)
        {
            ctx->handle_high_mark = ctx->num_handle_alloc;
        }
        if (ctx->num_handle_alloc == ctx->handle_poolsize)
        {
            ctx->max_handle_alloc_cnt++;
        }
    }
    else
    {
        /* wait for the buffer handle to become available */
        if (!(BUF_HANDLE_AVAIL(ctx)))
        {
            /* clear the flag now as there is no available buffer handle */
            CLEAR_BUFFER_AVAIL_EVENT();
            wiced_rtos_unlock_mutex(&priv->lock);
            WAIT_BUFFER_AVAIL_EVENT(timeout_ms);
            wiced_rtos_lock_mutex(&priv->lock);
            if (!(BUF_HANDLE_AVAIL(ctx)))
            {
                ctx->failed_handle_alloc_cnt++;
                wiced_rtos_unlock_mutex(&priv->lock);
                return BUFMGR_BUF_HANDLE_NOT_AVAIL;
            }
        }

        /* get a handle from the free handle list */
        handle            = priv->free_handle;
        priv->free_handle = priv->free_handle->next;
        ctx->num_handle_alloc++;
        if (ctx->num_handle_alloc > ctx->handle_high_mark)
        {
            ctx->handle_high_mark = ctx->num_handle_alloc;
        }
        if (ctx->num_handle_alloc == ctx->handle_poolsize)
        {
            ctx->max_handle_alloc_cnt++;
        }
    }

    /* increment the reference count */
    src_handle->buf_info->reference_cnt++;
    wiced_rtos_unlock_mutex(&priv->lock);

    /* initialize the new buffer handle */
    handle->buf_info = src_handle->buf_info;
    handle->offset   = src_handle->offset + offset;
    handle->size     = size;
    handle->flags    = src_handle->flags;
    memcpy(&handle->pts, &src_handle->pts, sizeof(mtimestamp_t));
    memcpy(&handle->dts, &src_handle->dts, sizeof(mtimestamp_t));

    *hnewbuf = (objhandle_t)handle;

    return BUFMGR_OK;
}


static bufmgr_status_t clone_buf(objhandle_t hpool, objhandle_t hbuf, objhandle_t* hnewbuf,
                                 uint32_t offset, uint32_t size, uint32_t timeout_ms)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_buf_handle_t* src_handle = (bufmgr_buf_handle_t*)hbuf;
    bufmgr_buf_handle_t* new_handle;
    char* dst;
    char* src;
    uint32_t bufsize;
    bufmgr_status_t rc;

    VALIDATE_POOL_HANDLE(ctx);
    VALIDATE_BUF_HANDLE(ctx, src_handle);

    src = src_handle->buf_info->buf + src_handle->offset + offset;
    /* validate the size of the data to be copied is within the buffer size */
    if (size > ctx->bufsize - (src_handle->offset + offset))
    {
        return BUFMGR_SRCBUF_OUT_OF_RANGE;
    }

    /* allocate a new buffer */
    if ((rc = alloc_buf(hpool, hnewbuf, &dst, &bufsize, timeout_ms)) != BUFMGR_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "alloc_buf() failed (rc = 0x%08X)\n", rc);
        return rc;
    }

    new_handle = (bufmgr_buf_handle_t*)(*hnewbuf);

    memcpy(dst, src, size);
    new_handle->size  = size;
    new_handle->flags = src_handle->flags;
    memcpy(&new_handle->pts, &src_handle->pts, sizeof(mtimestamp_t));
    memcpy(&new_handle->dts, &src_handle->dts, sizeof(mtimestamp_t));

    return BUFMGR_OK;
}


static bufmgr_status_t copy_buf(objhandle_t hpool, objhandle_t hbuf, char* dst_buf,
                                uint32_t offset, uint32_t* num_bytes)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;
    char* src;

    VALIDATE_POOL_HANDLE(ctx);
    VALIDATE_BUF_HANDLE(ctx, handle);

    src = handle->buf_info->buf + handle->offset + offset;
    /* validate the size of the data to be copied is within the buffer size */
    if (*num_bytes > ctx->bufsize - (handle->offset + offset))
    {
        *num_bytes = ctx->bufsize - (handle->offset + offset);
    }
    memcpy(dst_buf, src, *num_bytes);

    return BUFMGR_OK;
}


static uint32_t get_poolsize(objhandle_t hpool)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(ctx);

    return ctx->poolsize;
}


static uint32_t get_handle_poolsize(objhandle_t hpool)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(ctx);

    return ctx->handle_poolsize;
}


static uint32_t get_bufsize(objhandle_t hpool)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(ctx);

    return ctx->bufsize;
}


static uint32_t get_actual_bufsize(objhandle_t hpool)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(ctx);

    return ctx->actual_bufsize;
}


static bufmgr_status_t get_stats(objhandle_t hpool, bufmgr_stats_t* stats)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(ctx);

    if (!stats)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "output statistic buffer is NULL\n");
        return BUFMGR_INVALID_PARAM;
    }

    stats->num_buf_alloc           = ctx->num_buf_alloc;
    stats->num_handle_alloc        = ctx->num_handle_alloc;
    stats->buf_high_mark           = ctx->buf_high_mark;
    stats->handle_high_mark        = ctx->handle_high_mark;
    stats->max_buf_alloc_cnt       = ctx->max_buf_alloc_cnt;
    stats->max_handle_alloc_cnt    = ctx->max_handle_alloc_cnt;
    stats->failed_buf_alloc_cnt    = ctx->failed_buf_alloc_cnt;
    stats->failed_handle_alloc_cnt = ctx->failed_handle_alloc_cnt;

    return BUFMGR_OK;
}


static bufmgr_status_t set_buf_flags(objhandle_t hpool, objhandle_t hbuf, uint32_t flags)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    (void)ctx;
    VALIDATE_POOL_HANDLE(ctx);
    VALIDATE_BUF_HANDLE(ctx, handle);
    handle->flags = flags;

    return BUFMGR_OK;
}


static uint32_t get_buf_flags(objhandle_t hpool, objhandle_t hbuf)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    (void)ctx;
    VALIDATE_POOL_HANDLE(ctx);
    VALIDATE_BUF_HANDLE(ctx, handle);

    return handle->flags;
}


static bufmgr_status_t set_buf_pts(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* pts)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    (void)ctx;
    VALIDATE_POOL_HANDLE(ctx);
    VALIDATE_BUF_HANDLE(ctx, handle);
    memcpy(&handle->pts, pts, sizeof(mtimestamp_t));

    return BUFMGR_OK;
}


static bufmgr_status_t get_buf_pts(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* pts)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    (void)ctx;
    VALIDATE_POOL_HANDLE(ctx);
    VALIDATE_BUF_HANDLE(ctx, handle);
    memcpy(pts, &handle->pts, sizeof(mtimestamp_t));

    return BUFMGR_OK;
}


static bufmgr_status_t set_buf_dts(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* dts)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    (void)ctx;
    VALIDATE_POOL_HANDLE(ctx);
    VALIDATE_BUF_HANDLE(ctx, handle);
    memcpy(&handle->dts, dts, sizeof(mtimestamp_t));

    return BUFMGR_OK;
}


static bufmgr_status_t get_buf_dts(objhandle_t hpool, objhandle_t hbuf, mtimestamp_t* dts)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    (void)ctx;
    VALIDATE_POOL_HANDLE(ctx);
    VALIDATE_BUF_HANDLE(ctx, handle);
    memcpy(dts, &handle->dts, sizeof(mtimestamp_t));

    return BUFMGR_OK;
}


static void bufmgr_pool_free(bufmgr_pool_t* ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    if (ctx->subclass_private != NULL)
    {
        bufmgr_pool_priv_t* priv;

        priv = (bufmgr_pool_priv_t*)ctx->subclass_private;

        if (priv->buf_handle != NULL)
        {
            free(priv->buf_handle);
        }
        if (priv->buf_info != NULL)
        {
            free(priv->buf_info);
        }
        if (priv->buf_base_addr != NULL)
        {
            free(priv->buf_base_addr);
        }
        wiced_rtos_deinit_event_flags(&priv->buf_avail_signal);
        wiced_rtos_deinit_mutex(&priv->lock);
        free(priv);
        ctx->subclass_private = NULL;
    }

    free(ctx);
}


bufmgr_status_t bufmgr_pool_create(objhandle_t* hpool, char* name, uint32_t bufsize, uint32_t poolsize,
                                   uint32_t handle_poolsize, uint32_t prepend_bytes, uint32_t alignment)
{
    bufmgr_pool_t* ctx;
    bufmgr_pool_priv_t* priv;
    char* align_buf_base_addr;
    uint32_t actual_bufsize;
    uint32_t alloc_size;
    uint16_t i;
    wiced_result_t result;

    /* Check for size, including extra suffixes we'll add later */
    if (strlen(name) > MAX_POOL_NAME_SIZE - 6)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Buffer pool name too long, must be <= %d characters\n", MAX_POOL_NAME_SIZE - 6);
        return BUFMGR_INVALID_PARAM;
    }

    /* allocate the buffer pool context */
    ctx = (bufmgr_pool_t*)calloc(1, sizeof(bufmgr_pool_t));
    if (ctx == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "allocating %s buffer pool context\n", name);
        return BUFMGR_MEM_ALLOC;
    }

    /* allocate the buffer pool private context */
    priv = (bufmgr_pool_priv_t*)calloc(1, sizeof(bufmgr_pool_priv_t));
    if (priv == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "allocating %s buffer pool private context\n", name);
        bufmgr_pool_free(ctx);
        return BUFMGR_MEM_ALLOC;
    }
    ctx->subclass_private = (void*)priv;

    /* compute actual buffer size */
    actual_bufsize      = bufsize + prepend_bytes;
    actual_bufsize      = ALIGN_BUF_SIZE(actual_bufsize, alignment);
    ctx->bufsize        = bufsize;
    ctx->actual_bufsize = actual_bufsize;
    ctx->prepend_bytes  = prepend_bytes;

    /* allocate the buffer pool memory area */
    alloc_size = actual_bufsize * poolsize + alignment;
    priv->buf_base_addr = (char *)malloc(alloc_size);
    if (priv->buf_base_addr == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "allocating %s buffer pool memory area\n", name);
        bufmgr_pool_free(ctx);
        return BUFMGR_MEM_ALLOC;
    }

    /* allocate the array of buffer info */
    alloc_size     = sizeof(bufmgr_buf_info_t) * poolsize;
    priv->buf_info = (bufmgr_buf_info_t *)calloc(1, alloc_size);
    if (priv->buf_info == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "allocating %s buffer pool\n", name);
        bufmgr_pool_free(ctx);
        return BUFMGR_MEM_ALLOC;
    }

    /* initialize and chain up the list of free buffer info */
    priv->free_buf      = priv->buf_info;
    align_buf_base_addr = (char *)ALIGN_BUF_SIZE((uintptr_t)priv->buf_base_addr, alignment);
    for (i = 0; i < poolsize; i++)
    {
        priv->free_buf[i].next = &priv->free_buf[i + 1];
        priv->free_buf[i].buf  = &align_buf_base_addr[i * actual_bufsize];
        priv->free_buf[i].buf += prepend_bytes;
    }
    priv->free_buf[poolsize - 1].next = NULL;
    ctx->poolsize             = poolsize;
    ctx->num_buf_alloc        = 0;
    ctx->failed_buf_alloc_cnt = 0;
    ctx->buf_high_mark        = 0;
    ctx->max_buf_alloc_cnt    = 0;

    /* allocate the buffer handle pool */
    alloc_size = sizeof(bufmgr_buf_handle_t) * handle_poolsize;
    priv->buf_handle = (bufmgr_buf_handle_t*)calloc(1, alloc_size);
    if (priv->buf_handle == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "allocating %s handle pool\n", name);
        bufmgr_pool_free(ctx);
        return BUFMGR_MEM_ALLOC;
    }

    /* initialize and chain up the list of free buffer handle */
    priv->free_handle = priv->buf_handle;
    for (i = 0; i < handle_poolsize; i++)
    {
        priv->free_handle[i].next  = &priv->free_handle[i + 1];
        priv->free_handle[i].hpool = (objhandle_t)ctx;
        priv->free_handle[i].size  = bufsize;
    }
    priv->free_handle[handle_poolsize - 1].next = NULL;
    ctx->handle_poolsize         = handle_poolsize;
    ctx->num_handle_alloc        = 0;
    ctx->failed_handle_alloc_cnt = 0;
    ctx->handle_high_mark        = 0;
    ctx->max_handle_alloc_cnt    = 0;

    /* create semaphore for signaling when there is buffer available */
    if ((result = wiced_rtos_init_event_flags(&priv->buf_avail_signal)) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_rtos_init_semaphore(): %d\n", (int)result);
        bufmgr_pool_free(ctx);
        return BUFMGR_UNKNOWN_ERR;
    }

    /* create lock for managing the pool */
    if ((result = wiced_rtos_init_mutex(&priv->lock)) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_rtos_init_mutex(): %d\n", (int)result);
        bufmgr_pool_free(ctx);
        return BUFMGR_UNKNOWN_ERR;
    }

    ctx->callback     = NULL;
    ctx->callback_arg = NULL;

    /* hook up the interface functions */
    ctx->register_cb         = register_cb;
    ctx->alloc_buf           = alloc_buf;
    ctx->free_buf            = free_buf;
    ctx->dup_buf             = dup_buf;
    ctx->clone_buf           = clone_buf;
    ctx->copy_buf            = copy_buf;
    ctx->get_poolsize        = get_poolsize;
    ctx->get_handle_poolsize = get_handle_poolsize;
    ctx->get_bufsize         = get_bufsize;
    ctx->get_actual_bufsize  = get_actual_bufsize;
    ctx->get_stats           = get_stats;
    ctx->set_buf_flags       = set_buf_flags;
    ctx->get_buf_flags       = get_buf_flags;
    ctx->set_buf_pts         = set_buf_pts;
    ctx->get_buf_pts         = get_buf_pts;
    ctx->set_buf_dts         = set_buf_dts;
    ctx->get_buf_dts         = get_buf_dts;

    *hpool = (objhandle_t)ctx;

    return BUFMGR_OK;
}


bufmgr_status_t bufmgr_pool_destroy(objhandle_t hpool)
{
    bufmgr_pool_t* ctx = (bufmgr_pool_t*)hpool;

    VALIDATE_POOL_HANDLE(ctx);
    bufmgr_pool_free(ctx);

    return BUFMGR_OK;
}


bufmgr_status_t bufmgr_buf_free(objhandle_t hbuf)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    return (bufmgr_pool_free_buf(handle->hpool, hbuf));
}


bufmgr_status_t bufmgr_buf_dup(objhandle_t hbuf, objhandle_t* hnewbuf, uint32_t offset,
                               uint32_t size, uint32_t timeout_ms)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    return (bufmgr_pool_dup_buf(handle->hpool, hbuf, hnewbuf, offset, size, timeout_ms));
}


bufmgr_status_t bufmgr_buf_clone(objhandle_t hbuf, objhandle_t* hnewbuf, uint32_t offset,
                                 uint32_t size, uint32_t timeout_ms)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    return (bufmgr_pool_clone_buf(handle->hpool, hbuf, hnewbuf, offset, size, timeout_ms));
}


bufmgr_status_t bufmgr_buf_copy(objhandle_t hbuf, char* dst_buf, uint32_t offset, uint32_t* num_bytes)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    return (bufmgr_pool_copy_buf(handle->hpool, hbuf, dst_buf, offset, num_bytes));
}


bufmgr_status_t bufmgr_buf_get_data(objhandle_t hbuf, char** data, uint32_t* size)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    *data = handle->buf_info->buf + handle->offset;
    *size = handle->size;

    return BUFMGR_OK;
}


bufmgr_status_t bufmgr_buf_adjust_offset(objhandle_t hbuf, uint32_t offset)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    handle->offset += offset;
    handle->size   -= offset;

    return BUFMGR_OK;
}


bufmgr_status_t bufmgr_buf_set_size(objhandle_t hbuf, uint32_t size)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    handle->size = size;

    return BUFMGR_OK;
}


uint32_t bufmgr_buf_get_size(objhandle_t hbuf)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);

    return handle->size;
}


bufmgr_status_t bufmgr_buf_get_hdr(objhandle_t hbuf, char** header)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;
    bufmgr_pool_t* ctx;

    ASSERT_HANDLE(handle);
    ctx     = (bufmgr_pool_t*)handle->hpool;
    *header = handle->buf_info->buf - ctx->prepend_bytes;

    return BUFMGR_OK;
}


bufmgr_status_t bufmgr_buf_get_tail(objhandle_t hbuf, char** trailer)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;
    bufmgr_pool_t* ctx;

    ASSERT_HANDLE(handle);
    ctx      = (bufmgr_pool_t*)handle->hpool;
    *trailer = handle->buf_info->buf + ctx->bufsize - 1;

    return BUFMGR_OK;
}


bufmgr_status_t bufmgr_buf_set_flags(objhandle_t hbuf, uint32_t flags)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    handle->flags = flags;

    return BUFMGR_OK;
}


uint32_t bufmgr_buf_get_flags(objhandle_t hbuf)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    return handle->flags;
}


bufmgr_status_t bufmgr_buf_set_pts(objhandle_t hbuf, mtimestamp_t* pts)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    memcpy(&handle->pts, pts, sizeof(mtimestamp_t));

    return BUFMGR_OK;
}


bufmgr_status_t bufmgr_buf_get_pts(objhandle_t hbuf, mtimestamp_t* pts)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    memcpy(pts, &handle->pts, sizeof(mtimestamp_t));

    return BUFMGR_OK;
}


bufmgr_status_t bufmgr_buf_set_dts(objhandle_t hbuf, mtimestamp_t* dts)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    memcpy(&handle->dts, dts, sizeof(mtimestamp_t));

    return BUFMGR_OK;
}


bufmgr_status_t bufmgr_buf_get_dts(objhandle_t hbuf, mtimestamp_t* dts)
{
    bufmgr_buf_handle_t* handle = (bufmgr_buf_handle_t*)hbuf;

    ASSERT_HANDLE(handle);
    memcpy(dts, &handle->dts, sizeof(mtimestamp_t));

    return BUFMGR_OK;
}
