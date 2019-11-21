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

/** @file
 *
 * Audio render - Audio playback library.
 *
 */

#include "wiced_result.h"
#include "wiced_rtos.h"
#include "wiced_platform.h"
#include "platform_audio.h"
#include "wiced_log.h"
#include <string.h>
#include <math.h>

#include "audio_render.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define MEDIA_TIME_CHECK    1

#define PRINT_TIME(time)    (int)(time / NUM_NSECONDS_IN_SECOND), (int)(time % NUM_NSECONDS_IN_SECOND)

#define BYTES_TO_MILLISECONDS(number_of_bytes, sample_rate, frame_size) \
    ((double)(((NUM_USECONDS_IN_SECOND / ((double)(sample_rate * frame_size))) * number_of_bytes) / (double)NUM_MSECONDS_IN_SECOND))

#define BYTES_TO_NANOSECS(number_of_bytes, sample_rate, frame_size) \
    ((double)(((double)NUM_NSECONDS_IN_SECOND / (double)(sample_rate * frame_size)) * number_of_bytes))

#define NANOSECS_TO_BYTES(duration, sample_rate, frame_size) \
    (uint32_t)((uint32_t)((double)(duration * sample_rate) / (double)NUM_NSECONDS_IN_SECOND) * (uint32_t)frame_size)

/******************************************************
 *                    Constants
 ******************************************************/

#define NUM_NSECONDS_IN_SECOND                      (1000000000LL)
#define NUM_USECONDS_IN_SECOND                      (1000000)
#define NUM_NSECONDS_IN_MSECOND                     (1000000)
#define NUM_MSECONDS_IN_SECOND                      (1000)
#define MAX_AUDIO_INIT_ATTEMPTS                     (5)
#define MAX_CONFIG_FLUSH_ATTEMPTS                   (3)
#define INTERVAL_BETWEEN_AUDIO_INIT_ATTEMPTS        (100)

#define AUDIO_RENDER_PERIOD_SIZE                    (512)
#define AUDIO_RENDER_PERIODS_TO_START               (2)

#define AUDIO_RENDER_SAMPLE_CACHE_SIZE              (8)

#define AUDIO_RENDER_THREAD_STACK_SIZE              (6 * 1024)
#define AUDIO_RENDER_THREAD_PRIORITY                (3)
#define AUDIO_RENDER_THREAD_NAME                    ("audio render thread")

#define AUDIO_RENDER_DEFAULT_DISCONTINUITY_THRESHOLD    (2 * NUM_NSECONDS_IN_SECOND)

#define AUDIO_RENDER_TAG_VALID                      (0x0C0FFEE0)
#define AUDIO_RENDER_TAG_INVALID                    (0xDEADBEEF)

#define AUDIO_RENDER_BUF_FLAG_EOS                   (1 << 0)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    AUDIO_EVENT_SHUTDOWN           = (1 << 0),
    AUDIO_EVENT_PROCESS_BUFFER     = (1 << 1),
    AUDIO_EVENT_FLUSH              = (1 << 2),
    AUDIO_EVENT_STOP               = (1 << 3),
    AUDIO_EVENT_PAUSE              = (1 << 4),
    AUDIO_EVENT_RESUME             = (1 << 5),
} AUDIO_EVENTS_T;

#define AUDIO_ALL_EVENTS       (-1)

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct buf_list_s
{
    struct buf_list_s*      next;
    audio_render_buf_t      buf;
    wiced_audio_config_t    config;
    uint8_t                 flags;
} buf_list_t;

typedef struct audio_render_s
{
    uint32_t                            tag;
    wiced_bool_t                        quit;
    volatile wiced_bool_t               flush_in_progress;
    wiced_bool_t                        clock_enable;
    wiced_bool_t                        clock_set;
    wiced_bool_t                        err_msg;
    wiced_bool_t                        release_device_on_stop;

    wiced_event_flags_t                 events;
    wiced_mutex_t                       mutex;

    platform_audio_device_id_t          device_id;
    audio_render_buf_release_cb_t       buffer_release_cb;
    audio_render_time_get_cb_t          time_get_cb;
    audio_render_event_cb_t             event_cb;
    void*                               userdata;
    wiced_audio_config_t                current_config;
    wiced_audio_config_t                buffer_config;
    wiced_audio_session_ref             sh;
    wiced_bool_t                        audio_running;
    wiced_bool_t                        audio_configured;
    wiced_bool_t                        audio_paused;
    wiced_bool_t                        signal_eos;
    uint16_t                            period_size_override;
    uint16_t                            period_size;
    uint32_t                            period_wait_time;
    uint16_t                            num_period_buffers;
    uint32_t                            bytes_buffered;
    double                              minimum_volume;
    double                              maximum_volume;
    uint8_t                             cur_volume;                 /* 0 - 100 */
    uint32_t                            latency;
    float                               speed_in_ppm;

    wiced_bool_t                        media_time_set;
    int64_t                             buffer_ns;
    int64_t                             threshold_ns;
    int64_t                             discontinuity_threshold;
    int64_t                             last_pts_pushed;
    uint8_t                             sample_cache[AUDIO_RENDER_SAMPLE_CACHE_SIZE];

    wiced_thread_t                      audio_thread;
    wiced_thread_t*                     audio_thread_ptr;
    uint8_t                             stack[AUDIO_RENDER_THREAD_STACK_SIZE];

    /*
     * Buffer management.
     */

    uint32_t                            num_buffer_nodes;
    uint32_t                            num_buffer_used;
    volatile uint32_t                   buffer_bytes_avail;
    buf_list_t*                         buffer_list;
    buf_list_t*                         buffer_free_list;
    buf_list_t*                         buffer_head;
    buf_list_t*                         buffer_tail;

    /*
     * Stats collecting.
     */

    uint64_t                            audio_frames_played;
    uint64_t                            audio_frames_dropped;
    uint64_t                            audio_frames_inserted;
    int64_t                             audio_max_early;
    int64_t                             audio_max_late;
    uint64_t                            audio_underruns;

} audio_render_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/


static inline wiced_result_t perform_device_release(audio_render_t* audio)
{
    wiced_result_t result   = WICED_SUCCESS;
    uint8_t        attempts = 0;

    wiced_audio_deinit(audio->sh);
    audio->audio_configured = WICED_FALSE;

    do
    {
        result = wiced_audio_init(audio->device_id, &audio->sh, 0);
    } while ((result != WICED_SUCCESS) && (++attempts < MAX_AUDIO_INIT_ATTEMPTS) && (wiced_rtos_delay_milliseconds(INTERVAL_BETWEEN_AUDIO_INIT_ATTEMPTS) == WICED_SUCCESS));

    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to reinitialize audio device %d (%d)\n", audio->device_id, result);
        return WICED_ERROR;
    }

    return result;
}


static inline void perform_audio_stop(audio_render_t* audio)
{
    if (audio->audio_running)
    {
        wiced_audio_stop(audio->sh);
        audio->audio_running  = WICED_FALSE;
        audio->bytes_buffered = 0;
    }
}


static inline wiced_result_t get_media_time(audio_render_t* audio, int64_t* media_time, wiced_bool_t* clock_set)
{
    wiced_result_t result;

    if (clock_set)
    {
        *clock_set = WICED_FALSE;
    }

    if (audio->time_get_cb == NULL)
    {
        wiced_8021as_time_t as_time;

        /*
         * wiced_time_read_8021as() requires a valid audio sample rate in order to get a valid audio time;
         * however, if audio time is not required, audio sample rate MUST be set to 0
         */
        as_time.audio_sample_rate = 0;
        result = wiced_time_read_8021as(&as_time);

        if (result == WICED_SUCCESS)
        {
            *media_time = ((int64_t)as_time.master_secs * NUM_NSECONDS_IN_SECOND) + (int64_t)as_time.master_nanosecs;
            if (clock_set)
            {
                /*
                 * If the master time and local time are the same, it means that we haven't
                 * received any clock messages yet so the clock hasn't been set to master time.
                 */

                if (as_time.master_secs != as_time.local_secs || as_time.master_nanosecs != as_time.local_nanosecs)
                {
                    *clock_set = WICED_TRUE;
                }
            }
        }
    }
    else
    {
        result = audio->time_get_cb(media_time, audio->userdata);
        if (result == WICED_SUCCESS && clock_set)
        {
            /*
             * Best we can do is assume that the clock has been set correctly.
             */

            *clock_set = WICED_TRUE;
        }
    }

    if (result != WICED_SUCCESS)
    {
        *media_time = 0;
    }
#if MEDIA_TIME_CHECK
    else
    {
        static int64_t last_mtime = 0;

        if (last_mtime && *media_time < last_mtime)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "TIME WENT BACKWARDS!!!  Current: %d.%09d,  last: %d.%09d\n",
                          PRINT_TIME(*media_time), PRINT_TIME(last_mtime));
        }
        last_mtime = *media_time;
    }
#endif

    return result;
}


static wiced_bool_t perform_device_configure(audio_render_t* audio, wiced_audio_config_t* config)
{
    wiced_result_t result;

    if (audio == NULL || audio->tag != AUDIO_RENDER_TAG_VALID || config == NULL)
    {
        return WICED_BADARG;
    }

    if (audio->audio_configured)
    {
        /*
         * Is the configuration the same as what we already have? If so then we're OK.
         */

        if (config->bits_per_sample == audio->current_config.bits_per_sample && config->channels == audio->current_config.channels &&
            config->frame_size == audio->current_config.frame_size && config->sample_rate == audio->current_config.sample_rate)
        {
            return WICED_SUCCESS;
        }

        /*
         * Make sure that the device is stopped. Do it ourselves instead of calling platform_audio_stop()
         * so that we can add a delay to make sure the DMA buffers are reclaimed before we deinit.
         */

        if (audio->audio_running)
        {
            wiced_audio_stop(audio->sh);
            audio->audio_running  = WICED_FALSE;
            audio->bytes_buffered = 0;
            wiced_rtos_delay_milliseconds(1);
        }

        result = perform_device_release(audio);
        if (result != WICED_SUCCESS)
        {
            return result;
        }
    }

    /*
     * Save the configuration.
     */

    audio->current_config = *config;

    /*
     * Set the period size to use with the audio driver. We want it to be a multiple of the frame size.
     */

    if (audio->period_size_override != 0)
    {
        audio->period_size = audio->period_size_override;
    }
    else
    {
        audio->period_size = AUDIO_RENDER_PERIOD_SIZE;
    }

    if (audio->period_size % audio->current_config.frame_size != 0)
    {
        audio->period_size += audio->current_config.frame_size - (audio->period_size % audio->current_config.frame_size);
    }
    audio->num_period_buffers = WICED_AUDIO_BUFFER_ROUND_DOWN(WICED_AUDIO_MAX_BUFFER_SIZE) / WICED_AUDIO_BUFFER_ARRAY_ELEMENT_SIZEOF(audio->period_size);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Using %d period buffers of size %d\n", audio->num_period_buffers, audio->period_size);

    result = wiced_audio_update_period_size(audio->sh, audio->period_size);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to update audio period size %d\n", result);
        return result;
    }

    result = wiced_audio_create_buffer(audio->sh, audio->period_size * audio->num_period_buffers, NULL, NULL);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create audio buffer %d (%d,%d)\n", result, audio->period_size, audio->num_period_buffers);
        return result;
    }

    result = wiced_audio_configure(audio->sh, &audio->current_config);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to configure audio output %d\n", result);
        return result;
    }

    /*
     * Retrieve the volume range for this device and set the current volume.
     */

    wiced_audio_get_volume_range(audio->sh, &audio->minimum_volume, &audio->maximum_volume);
    audio->cur_volume = MIN(audio->current_config.volume, 100);
    wiced_audio_set_volume(audio->sh, (double)audio->cur_volume * ((audio->maximum_volume - audio->minimum_volume) / 100.0) + audio->minimum_volume);

    /* Compute time period corresponding to period size */
    audio->period_wait_time = (uint32_t)ceil(BYTES_TO_MILLISECONDS(audio->period_size, audio->current_config.sample_rate, audio->current_config.frame_size));

    audio->audio_configured = WICED_TRUE;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio config is %d channels, %lu kHz, %d bps, frame size %u, period %lu msecs\n",
                  audio->current_config.channels, audio->current_config.sample_rate, audio->current_config.bits_per_sample,
                  audio->current_config.frame_size, audio->period_wait_time);

    return WICED_SUCCESS;
}


static void release_head_buffer(audio_render_t* audio)
{
    buf_list_t* buf_ptr;

    /*
     * Note that this routine assumes that the mutex is locked.
     */

    buf_ptr = audio->buffer_head;
    if (buf_ptr == NULL)
    {
        return;
    }

    if (buf_ptr->flags & AUDIO_RENDER_BUF_FLAG_EOS)
    {
        audio->signal_eos = WICED_TRUE;
    }

    /*
     * Remove it from the list.
     */

    audio->buffer_head = audio->buffer_head->next;
    if (audio->buffer_head == NULL)
    {
        audio->buffer_tail        = NULL;
        audio->buffer_bytes_avail = 0;
    }

    /*
     * And release the buffer.
     */

    if (audio->buffer_release_cb != NULL)
    {
        audio->buffer_release_cb(&buf_ptr->buf, audio->userdata);
    }

    /*
     * And put this node back on the free list.
     */

    buf_ptr->next           = audio->buffer_free_list;
    audio->buffer_free_list = buf_ptr;
    audio->num_buffer_used--;
}


static void flush_buffer_list(audio_render_t* audio)
{
    wiced_result_t result;
    buf_list_t* buf_ptr;
    buf_list_t* next;

    result = wiced_rtos_lock_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock audio mutex for flush %d\n", result);
        return;
    }

    /*
     * Traverse the current buffer list and release all queued buffers.
     */

    for (buf_ptr = audio->buffer_head; buf_ptr != NULL; buf_ptr = next)
    {
        next = buf_ptr->next;

        /*
         * Release this buffer.
         */

        if (audio->buffer_release_cb != NULL)
        {
            audio->buffer_release_cb(&buf_ptr->buf, audio->userdata);
        }

        /*
         * And put it back on the free list.
         */

        buf_ptr->next           = audio->buffer_free_list;
        audio->buffer_free_list = buf_ptr;
    }

    audio->buffer_head        = NULL;
    audio->buffer_tail        = NULL;
    audio->num_buffer_used    = 0;
    audio->buffer_bytes_avail = 0;

    wiced_rtos_unlock_mutex(&audio->mutex);
}


static void perform_flush(audio_render_t* audio)
{
    audio->flush_in_progress = WICED_TRUE;

    /*
     * Start by stopping the current audio playback.
     */

    perform_audio_stop(audio);

    /*
     * And now flush the buffer list.
     */

    flush_buffer_list(audio);

    audio->flush_in_progress = WICED_FALSE;
}


static int process_buffer_with_clock(audio_render_t* audio)
{
    wiced_result_t result;
    buf_list_t* buf_ptr;
    int64_t buftime;
    int64_t atime;
    int64_t media_time;
    int64_t diff;
    uint32_t bytes_to_drop;
    uint32_t bytes_to_add;
    uint32_t buffer_bytes_avail;
    uint8_t* ptr;
    uint16_t size;
    uint16_t audio_buf_size;
    int      bytes;
    uint32_t cache_size;

    result = wiced_rtos_lock_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock audio mutex %d\n", result);
        return -1;
    }

    /*
     * Grab the pointer to the head of the buffer list and the current available count.
     * The buffer push routine does not access the buffer_head pointer after it's
     * been set so it's safe to use that pointer outside of the mutex lock.
     */

    buf_ptr            = audio->buffer_head;
    buffer_bytes_avail = audio->buffer_bytes_avail;

    /*
     * Check the buffer tail for an EOS flag. At the end of a stream we can
     * end up with multiple small buffers that total less than a period of data.
     */

    if (buffer_bytes_avail < audio->period_size && audio->buffer_tail && (audio->buffer_tail->flags & AUDIO_RENDER_BUF_FLAG_EOS))
    {
        audio->signal_eos = WICED_TRUE;
    }

    wiced_rtos_unlock_mutex(&audio->mutex);

    /*
     * If we don't have enough data to fill an audio buffer than return.
     */

    if (buffer_bytes_avail < audio->period_size || buf_ptr == NULL)
    {
        return 0;
    }

    /*
     * What's already queued up for output?
     */

    wiced_audio_get_current_buffer_weight(audio->sh, &audio->latency);
    atime = (int64_t)BYTES_TO_NANOSECS(audio->latency, audio->current_config.sample_rate, audio->current_config.frame_size);

    /*
     * Get what the media time will be when the output queue is empty and
     * figure out the difference from the buffer time.
     */

    buftime = buf_ptr->buf.pts + audio->buffer_ns;
    get_media_time(audio, &media_time, NULL);
    diff = (media_time + atime) - buftime;
    bytes_to_add = 0;

    /*
     * If the difference is less than the threshold value, ignore it.
     */

    if (llabs(diff) <= audio->threshold_ns)
    {
        diff = 0;
    }

    if (diff > 0)
    {
        diff -= audio->threshold_ns;
        bytes_to_drop = NANOSECS_TO_BYTES(diff, audio->current_config.sample_rate, audio->current_config.frame_size);
        if (bytes_to_drop > buffer_bytes_avail)
        {
            /*
             * Make sure that we don't drop partial audio frames.
             */

            bytes_to_drop = (buffer_bytes_avail / audio->current_config.frame_size) * audio->current_config.frame_size;
        }

        audio->audio_frames_dropped += bytes_to_drop / audio->current_config.frame_size;

        if (diff > audio->audio_max_late)
        {
            audio->audio_max_late = diff;
        }

        /*
         * Late audio, we need to drop audio samples.
         * Grab the mutex so that we can play with the buffers.
         */

        result = wiced_rtos_lock_mutex(&audio->mutex);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock audio mutex %d\n", result);
            return -1;
        }

        audio->buffer_bytes_avail -= bytes_to_drop;
        buffer_bytes_avail        -= bytes_to_drop;

        while (bytes_to_drop > 0 && buf_ptr != NULL)
        {
            if (bytes_to_drop >= buf_ptr->buf.data_length)
            {
                /*
                 * Release the entire buffer.
                 */

                bytes_to_drop -= buf_ptr->buf.data_length;
                release_head_buffer(audio);
                buf_ptr = audio->buffer_head;
            }
            else
            {
                buf_ptr->buf.data_offset += bytes_to_drop;
                buf_ptr->buf.data_length -= bytes_to_drop;

                /*
                 * And adjust the PTS.
                 */

                buf_ptr->buf.pts += (int64_t)BYTES_TO_NANOSECS(bytes_to_drop, audio->current_config.sample_rate, audio->current_config.frame_size);
            }
        }

        wiced_rtos_unlock_mutex(&audio->mutex);

        if (buffer_bytes_avail < audio->period_size || buf_ptr == NULL)
        {
            /*
             * Not enough data left to fill a buffer.
             */

            return buffer_bytes_avail;
        }
    }
    else if (diff < 0)
    {
        /*
         * Early audio.
         */

        diff += audio->threshold_ns;
        bytes_to_add = NANOSECS_TO_BYTES((-diff), audio->current_config.sample_rate, audio->current_config.frame_size);

        audio->audio_frames_inserted += bytes_to_add / audio->current_config.frame_size;

        if (diff < audio->audio_max_early)
        {
            audio->audio_max_early = diff;
        }
    }

    /* Wait till at least one period is available for writing */
    result = wiced_audio_wait_buffer(audio->sh, audio->period_size, audio->period_wait_time + 2);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio underrun detected (%lu bytes available)\n", audio->buffer_bytes_avail);
        audio->audio_underruns++;
        /* Must do a recovery there */
        perform_audio_stop(audio);

        return buffer_bytes_avail;
    }

    /*
     * Get the audio buffer.
     */

    audio_buf_size = audio->period_size;
    result = wiced_audio_get_buffer(audio->sh, &ptr, &audio_buf_size);
    if (result != WICED_SUCCESS)
    {
        /* Potential underrun. */
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No audio buffer chunk available!\n");
        perform_audio_stop(audio);
        return buffer_bytes_avail;
    }

    /*
     * We need to fill the buffer now.
     * Grab the mutex.
     */

    result = wiced_rtos_lock_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock audio mutex %d\n", result);
        return buffer_bytes_avail;
    }

    size = 0;
    cache_size = audio->current_config.frame_size;
    while (size < audio_buf_size && audio->buffer_head != NULL)
    {
        if (bytes_to_add > 0)
        {
            uint32_t i;

            /*
             * Adding silence.
             * Remember to adjust buffer_bytes_available
             */

            bytes = MIN(audio_buf_size - size, bytes_to_add);

            /*
             * Silence concealment
             */

            if (bytes <= cache_size)
            {
                memcpy(&ptr[size], audio->sample_cache, bytes);
            }
            else
            {
                uint32_t left_over;
                uint32_t quotient;

                quotient  = bytes / cache_size;
                left_over = bytes % cache_size;

                for (i = 0; i < quotient; i++)
                {
                    memcpy(&ptr[size + (i * cache_size)], audio->sample_cache, cache_size);
                }

                if (left_over > 0)
                {
                    memcpy(&ptr[size + (i * cache_size)], audio->sample_cache, left_over);
                }
            }

            bytes_to_add -= bytes;
        }
        else
        {
            buf_ptr = audio->buffer_head;
            bytes   = MIN(audio_buf_size - size, buf_ptr->buf.data_length);
            memcpy(&ptr[size], &buf_ptr->buf.data_buf[buf_ptr->buf.data_offset], bytes);
            buf_ptr->buf.data_length  -= bytes;
            audio->buffer_bytes_avail -= bytes;

            /*
             * Caching last set of samples
             */

            if (bytes >= cache_size)
            {
                memcpy(audio->sample_cache, &buf_ptr->buf.data_buf[buf_ptr->buf.data_offset + bytes - cache_size], cache_size);
            }

            audio->audio_frames_played += bytes / audio->current_config.frame_size;

            /*
             * Are we done with this data buffer?
             */

            if (buf_ptr->buf.data_length == 0)
            {
                release_head_buffer(audio);
                buf_ptr = audio->buffer_head;
                if (buf_ptr != NULL &&
                    (buf_ptr->config.bits_per_sample != audio->current_config.bits_per_sample || buf_ptr->config.channels    != audio->current_config.channels ||
                     buf_ptr->config.frame_size      != audio->current_config.frame_size      || buf_ptr->config.sample_rate != audio->current_config.sample_rate))
                {
                    /*
                     * We have an audio configuration change. Fill any remaining space in the period buffer with silence.
                     */

                    if (size < audio_buf_size)
                    {
                        memset(&ptr[size], 0, audio_buf_size - size);
                        size  = audio_buf_size;
                        bytes = 0;
                    }
                }
            }
            else
            {
                buf_ptr->buf.data_offset += bytes;

                /*
                 * And adjust the PTS.
                 */

                buf_ptr->buf.pts += (int64_t)BYTES_TO_NANOSECS(bytes, audio->current_config.sample_rate, audio->current_config.frame_size);
            }
        }

        size               += bytes;
        buffer_bytes_avail  = audio->buffer_bytes_avail;
    }

    wiced_rtos_unlock_mutex(&audio->mutex);

    /*
     * And let the audio driver have the buffer.
     */

    result = wiced_audio_release_buffer(audio->sh, size);
    if (result == WICED_ERROR)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio buffer underrun occurred after fill!\r\n");
        audio->audio_underruns++;
        perform_audio_stop(audio);
        return buffer_bytes_avail;
    }

    audio->bytes_buffered += size;
    if (audio->audio_running == WICED_FALSE)
    {
        if (audio->bytes_buffered >= audio->period_size * AUDIO_RENDER_PERIODS_TO_START)
        {
            /* Only start when there's actual data. */
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Starting audio\n");
            result = wiced_audio_start(audio->sh);
            if (result == WICED_SUCCESS)
            {
                audio->audio_running = WICED_TRUE;
            }
            else
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Audio did not start\n");
            }
        }
    }

    return buffer_bytes_avail;
}


static int process_buffer_without_clock(audio_render_t* audio)
{
    wiced_result_t result;
    buf_list_t* buf_ptr;
    uint32_t buffer_bytes_avail;
    uint8_t* ptr;
    uint16_t size;
    uint16_t audio_buf_size;
    int bytes;
    int64_t mtime;

    result = wiced_rtos_lock_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock audio mutex %d\n", result);
        return 0;
    }

    buf_ptr            = audio->buffer_head;
    buffer_bytes_avail = audio->buffer_bytes_avail;

    /*
     * Check the buffer tail for an EOS flag. At the end of a stream we can
     * end up with multiple small buffers that total less than a period of data.
     */

    if (buffer_bytes_avail < audio->period_size && audio->buffer_tail && (audio->buffer_tail->flags & AUDIO_RENDER_BUF_FLAG_EOS))
    {
        audio->signal_eos = WICED_TRUE;
    }

    wiced_rtos_unlock_mutex(&audio->mutex);

    /*
     * If we don't have enough data to fill an audio buffer than return.
     */

    if (buffer_bytes_avail < audio->period_size || buf_ptr == NULL)
    {
        return buffer_bytes_avail;
    }

    if (audio->clock_enable == WICED_TRUE && audio->clock_set == WICED_FALSE)
    {
        /*
         * Clock hasn't been set yet. Check to see if we've received any clock messages.
         * If we have then it's time to bail out of blind push mode.
         */

        result = get_media_time(audio, &mtime, &audio->clock_set);

        if (result == WICED_SUCCESS && audio->clock_set == WICED_TRUE)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Clock has been set. Enabling clock playback.\n");
            return buffer_bytes_avail;
        }
    }

    /* Wait till at least one period is available for writing */
    result = wiced_audio_wait_buffer(audio->sh, audio->period_size, audio->period_wait_time + 2);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio underrun detected (%lu bytes available)\n", audio->buffer_bytes_avail);
        audio->audio_underruns++;

        /* Must do a recovery there */
        perform_audio_stop(audio);
    }

    /*
     * Get the audio buffer.
     */

    audio_buf_size = audio->period_size;
    result = wiced_audio_get_buffer(audio->sh, &ptr, &audio_buf_size);
    if (result != WICED_SUCCESS)
    {
        /* Potential underrun. */
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No audio buffer chunk available!\n");
        perform_audio_stop(audio);
        return buffer_bytes_avail;
    }

    /*
     * We need to fill the buffer now.
     * Grab the mutex.
     */

    result = wiced_rtos_lock_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock audio mutex %d\n", result);
        return buffer_bytes_avail;
    }

    size = 0;
    while (size < audio_buf_size && audio->buffer_head != NULL)
    {
        buf_ptr = audio->buffer_head;
        bytes   = MIN(audio_buf_size - size, buf_ptr->buf.data_length);
        memcpy(&ptr[size], &buf_ptr->buf.data_buf[buf_ptr->buf.data_offset], bytes);

        size                      += bytes;
        audio->buffer_bytes_avail -= bytes;
        buffer_bytes_avail         = audio->buffer_bytes_avail;
        buf_ptr->buf.data_length  -= bytes;

        audio->audio_frames_played += bytes / audio->current_config.frame_size;

        /*
         * Are we done with this data buffer?
         */

        if (buf_ptr->buf.data_length == 0)
        {
            release_head_buffer(audio);
            buf_ptr = audio->buffer_head;
            if (buf_ptr != NULL &&
                (buf_ptr->config.bits_per_sample != audio->current_config.bits_per_sample || buf_ptr->config.channels    != audio->current_config.channels ||
                 buf_ptr->config.frame_size      != audio->current_config.frame_size      || buf_ptr->config.sample_rate != audio->current_config.sample_rate))
            {
                /*
                 * We have an audio configuration change. Fill any remaining space in the period buffer with silence.
                 */

                if (size < audio_buf_size)
                {
                    memset(&ptr[size], 0, audio_buf_size - size);
                    size = audio_buf_size;
                }
            }
        }
        else
        {
            buf_ptr->buf.data_offset += bytes;
        }
    }

    wiced_rtos_unlock_mutex(&audio->mutex);

    /*
     * And let the audio driver have the buffer.
     */

    result = wiced_audio_release_buffer(audio->sh, size);
    if (result == WICED_ERROR)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio buffer underrun occurred after fill!\n");
        audio->audio_underruns++;
        perform_audio_stop(audio);
        return buffer_bytes_avail;
    }

    audio->bytes_buffered += size;
    if (audio->audio_running == WICED_FALSE)
    {
        if (audio->bytes_buffered >= audio->period_size * AUDIO_RENDER_PERIODS_TO_START)
        {
            /* Only start when there's actual data. */
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Starting audio\n");
            result = wiced_audio_start(audio->sh);
            if (result == WICED_SUCCESS)
            {
                audio->audio_running = WICED_TRUE;
            }
            else
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Audio did not start\n");
            }
        }
    }

    return buffer_bytes_avail;
}


static void process_audio_buffers(audio_render_t* audio)
{
    int bytes_available;
    uint32_t events;
    wiced_result_t result;

    /*
     * If the audio is paused than just return.
     */

    if (audio->audio_paused)
    {
        return;
    }

    /*
     * Start by assuming we have enough data to fill a period buffer and
     * continue processing buffers until we run out of data.
     */

    bytes_available = audio->period_size;
    while (bytes_available >= audio->period_size && !audio->signal_eos && !audio->quit)
    {
        /*
         * Buffer_head should be set if bytes_available is greater that 0. And since this is the
         * only thread that modifies buffer_head (other than setting it when it is NULL), we can
         * check it without grabbing the buffer mutex.
         */

        if (audio->buffer_head != NULL &&
            (!audio->audio_configured ||
              audio->buffer_head->config.bits_per_sample != audio->current_config.bits_per_sample || audio->buffer_head->config.channels    != audio->current_config.channels ||
              audio->buffer_head->config.frame_size      != audio->current_config.frame_size      || audio->buffer_head->config.sample_rate != audio->current_config.sample_rate))
        {
            result = perform_device_configure(audio, &audio->buffer_head->config);

            if (result != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to configure audio output\n");
                if (audio->event_cb != NULL)
                {
                    audio->event_cb(audio, audio->userdata, AUDIO_RENDER_EVENT_CONFIG_ERROR, NULL);
                }
                wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_STOP);
                break;
            }
        }

        if (audio->clock_enable == WICED_TRUE && audio->clock_set == WICED_TRUE)
        {
            bytes_available = process_buffer_with_clock(audio);
        }
        else
        {
            bytes_available = process_buffer_without_clock(audio);
        }

        /*
         * If we have a pending event notification we want to make sure that
         * we break out of this loop.
         */

        events = 0;
        result = wiced_rtos_wait_for_event_flags(&audio->events, (AUDIO_ALL_EVENTS & ~AUDIO_EVENT_PROCESS_BUFFER), &events, WICED_FALSE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
        if (result == WICED_SUCCESS && events != 0)
        {
            break;
        }
    }

    if (bytes_available < audio->period_size && audio->buffer_head != NULL)
    {
        /*
         * We don't have to lock the mutex for checking the buffer head flags since
         * all modifications to buffer_head once it's been set happen in the context
         * of this thread.
         */

        if (audio->buffer_head->flags & AUDIO_RENDER_BUF_FLAG_EOS)
        {
            audio->signal_eos = WICED_TRUE;
        }
    }

    if (audio->signal_eos && !audio->quit)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Processing signal_eos: %d (%lu)\n", bytes_available, audio->buffer_bytes_avail);
        if (audio->buffer_bytes_avail < audio->period_size)
        {
            uint32_t     wait_time_min;
            uint32_t     wait_time_max;
            wiced_time_t tick_start;
            wiced_time_t tick_stop;

            /*
             * Since there's no more data available we'll assume that playback
             * is stopping. We need to flush the remaining buffers since we can't
             * play less than a period worth of data. Be nice and let the audio
             * drain out of the audio driver buffers before shutting down. Otherwise
             * we could potentially cut off up to .1 seconds of audio.
             */

            wait_time_min = audio->period_wait_time;
            wait_time_max = (wait_time_min * (audio->num_period_buffers + 2));

            wiced_time_get_time(&tick_start);
            do
            {
                wiced_audio_get_current_buffer_weight(audio->sh, &audio->latency);
                wiced_rtos_delay_milliseconds(wait_time_min);
                wiced_time_get_time(&tick_stop);
            } while (!audio->quit && (audio->latency > 0) && ((tick_stop - tick_start) <= wait_time_max) );

            /*
             * It's possible buffers for a new stream started coming in while we were waiting.
             * Check for that before shutting down.
             */

            if (audio->buffer_bytes_avail < audio->period_size)
            {
                if (audio->release_device_on_stop)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Release audio device\n");
                    perform_audio_stop(audio);
                    perform_flush(audio);
                    perform_device_release(audio);
                }
                else
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Flush buffer list\n");
                    flush_buffer_list(audio);
                }
            }
            else
            {
                if (audio->buffer_head && (audio->buffer_head->flags & AUDIO_RENDER_BUF_FLAG_EOS) && audio->buffer_head->next)
                {
                    /*
                     * Clear the EOS flag so we don't send the notification twice.
                     */

                    audio->buffer_head->flags &= ~(AUDIO_RENDER_BUF_FLAG_EOS);
                }
            }
        }

        /*
         * Tell the app about EOS.
         */

        if (audio->event_cb != NULL)
        {
            audio->event_cb(audio, audio->userdata, AUDIO_RENDER_EVENT_EOS, NULL);
        }
        audio->signal_eos = WICED_FALSE;
    }

    /*
     * If there's more than a periods worth of audio data available, make sure
     * that we'll have a process buffer event in the queue.
     */

    if (bytes_available >= audio->period_size && !audio->quit)
    {
        wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_PROCESS_BUFFER);
    }
}


static void audio_render_thread(uint32_t context)
{
    audio_render_t* audio = (audio_render_t* )context;
    wiced_result_t result;
    uint32_t events;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio render thread begin\n");

    while (audio->quit == WICED_FALSE)
    {
        events = 0;
        result = wiced_rtos_wait_for_event_flags(&audio->events, AUDIO_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (result != WICED_SUCCESS)
        {
            continue;
        }

        if (events & AUDIO_EVENT_SHUTDOWN)
        {
            break;
        }

        if (events & AUDIO_EVENT_FLUSH)
        {
            perform_flush(audio);
        }

        if (events & AUDIO_EVENT_STOP)
        {
            perform_audio_stop(audio);
            perform_flush(audio);
            if (audio->release_device_on_stop)
            {
                perform_device_release(audio);
            }
            audio->audio_paused = WICED_FALSE;
            audio->signal_eos   = WICED_FALSE;
            audio->current_config.sample_rate = 0;
            audio->buffer_config.sample_rate  = 0;
        }

        if (events & AUDIO_EVENT_PAUSE)
        {
            audio->audio_paused = WICED_TRUE;
        }

        if (events & AUDIO_EVENT_RESUME)
        {
            audio->audio_paused = WICED_FALSE;
            process_audio_buffers(audio);
        }

        if (events & AUDIO_EVENT_PROCESS_BUFFER)
        {
            process_audio_buffers(audio);
        }
    }

    if (audio->sh != NULL)
    {
        wiced_audio_stop(audio->sh);
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio render thread end\n");

    WICED_END_OF_CURRENT_THREAD();
}


static void audio_render_shutdown(audio_render_ref audio)
{
    if (audio == NULL || audio->tag != AUDIO_RENDER_TAG_VALID)
    {
        return;
    }

    audio->tag = AUDIO_RENDER_TAG_INVALID;

    /*
     * Shutdown the main thread.
     */

    if (audio->audio_thread_ptr != NULL)
    {
        audio->quit = WICED_TRUE;

        wiced_rtos_thread_force_awake(&audio->audio_thread);
        wiced_rtos_thread_join(&audio->audio_thread);
        wiced_rtos_delete_thread(&audio->audio_thread);

        audio->audio_thread_ptr = NULL;
    }

    if (audio->sh != NULL)
    {
        wiced_audio_deinit(audio->sh);
    }

    /*
     * Free the buffer list.
     */

    if (audio->buffer_list != NULL)
    {
        /*
         * Release any in-use buffers before freeing the list.
         */

        flush_buffer_list(audio);
        free(audio->buffer_list);
        audio->buffer_list      = NULL;
        audio->buffer_free_list = NULL;
    }

    wiced_rtos_deinit_event_flags(&audio->events);
    wiced_rtos_deinit_mutex(&audio->mutex);

    free(audio);
}


audio_render_ref audio_render_init(audio_render_params_t* params)
{
    audio_render_ref audio;
    wiced_result_t result;
    buf_list_t* buf_ptr;
    int i;
    uint8_t attempts = 0;

    if (params == NULL)
    {
        return NULL;
    }

    /*
     * Allocate our main structure.
     */

    audio = (audio_render_t* )calloc_named("audio_render", 1, sizeof(audio_render_t));
    if (audio == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Audio allocation failure\n");
        return NULL;
    }
    audio->tag = AUDIO_RENDER_TAG_VALID;

    /*
     * Store the audio configuration.
     */

    audio->clock_enable           = (params->clock_enable != 0) ? WICED_TRUE : WICED_FALSE;
    audio->buffer_ns              = params->buffer_ms * NUM_NSECONDS_IN_MSECOND;
    audio->threshold_ns           = params->threshold_ms * NUM_NSECONDS_IN_MSECOND;
    audio->buffer_release_cb      = params->buf_release_cb;
    audio->time_get_cb            = params->time_get_cb;
    audio->event_cb               = params->event_cb;
    audio->period_size_override   = params->period_size;
    audio->release_device_on_stop = params->release_device_on_stop;

    /*
     * Set the default audio discontinuity threshold.
     */

    audio->discontinuity_threshold = AUDIO_RENDER_DEFAULT_DISCONTINUITY_THRESHOLD;

    /*
     * Initialize our mutex.
     */

    result = wiced_rtos_init_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create audio mutex %d\n", result);
        goto error;
    }

    /*
     * Create our event flags.
     */

    result = wiced_rtos_init_event_flags(&audio->events);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create audio event flags %d\n", result);
        goto error;
    }

    /*
     * Setup the buffer list.
     */

    audio->num_buffer_nodes = params->buffer_nodes;
    audio->buffer_list      = (buf_list_t *)calloc_named("audio_buf_list", audio->num_buffer_nodes, sizeof(buf_list_t));
    if (audio->buffer_list == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create audio buffer list\n");
        goto error;
    }

    buf_ptr = audio->buffer_list;
    for (i = 0; i < audio->num_buffer_nodes - 1; ++i)
    {
        buf_ptr[i].next = &buf_ptr[i + 1];
    }

    audio->buffer_free_list = audio->buffer_list;

    audio->userdata = params->userdata;
    audio->quit     = WICED_FALSE;

    /*
     * Initialize the audio output device.
     */

    audio->device_id = params->device_id;
    do
    {
        result = wiced_audio_init(params->device_id, &audio->sh, 0);
    } while ((result != WICED_SUCCESS) && (++attempts < MAX_AUDIO_INIT_ATTEMPTS ) && (wiced_rtos_delay_milliseconds(INTERVAL_BETWEEN_AUDIO_INIT_ATTEMPTS)== WICED_SUCCESS));

    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to initialize audio device %d (%d)\n", audio->device_id, result);
        goto error;
    }

    if (attempts > 0)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio device took %d attempts to initialize\n", attempts + 1);
    }

    /*
     * Create the main render thtread.
     */

    result = wiced_rtos_create_thread_with_stack(&audio->audio_thread, AUDIO_RENDER_THREAD_PRIORITY, AUDIO_RENDER_THREAD_NAME,
                                                 audio_render_thread, audio->stack, AUDIO_RENDER_THREAD_STACK_SIZE, audio);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create audio render thread %d\n", result);
        goto error;
    }

    audio->audio_thread_ptr = &audio->audio_thread;

    return audio;

error:
    audio_render_shutdown(audio);
    return NULL;
}


wiced_result_t audio_render_deinit(audio_render_ref audio)
{
    if (audio == NULL || audio->tag != AUDIO_RENDER_TAG_VALID)
    {
        return WICED_BADARG;
    }

    audio_render_shutdown(audio);

    return WICED_SUCCESS;
}


wiced_result_t audio_render_configure(audio_render_ref audio, wiced_audio_config_t* config)
{
    wiced_result_t result = WICED_SUCCESS;

    if (audio == NULL || audio->tag != AUDIO_RENDER_TAG_VALID || config == NULL)
    {
        return WICED_BADARG;
    }

    /*
     * Just note the new audio configuration. The main thread will perform the audio
     * (re)configuration when it encounters a buffer with the new format.
     */

    audio->buffer_config = *config;

    return result;
}


wiced_result_t audio_render_push(audio_render_ref audio, audio_render_buf_t *buf)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_result_t time_result;
    buf_list_t* buf_ptr;
    int64_t mtime;
    int count;

    (void)mtime;        /* Avoid compiler warning. */

    if (audio == NULL || audio->tag != AUDIO_RENDER_TAG_VALID)
    {
        return WICED_BADARG;
    }

    if (audio->buffer_config.sample_rate == 0)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Audio format not configured, buffer discarded\n");
        return WICED_ERROR;
    }

#if 0
    if (audio->media_time_set && audio->clock_enable == WICED_TRUE && buf != NULL)
    {
        /*
         * Check for a clock discontinuity.
         * Note that this may need to eventually be moved into the context of the main audio render thread.
         */

        if ((int64_t)buf->pts < audio->last_pts_pushed - audio->discontinuity_threshold ||
            (int64_t)buf->pts > audio->last_pts_pushed + audio->discontinuity_threshold)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Audio discontinuity detected. cur: %d:%09d  last: %d:%09d\r\n",
                          PRINT_TIME(buf->pts), PRINT_TIME(audio->last_pts_pushed));
            perform_flush(audio);
            audio->media_time_set = WICED_FALSE;
        }
    }
#endif

    if (audio->flush_in_progress)
    {
        /*
         * A flush is in progress. Give it time to finish.
         */

        count = 0;
        while (audio->flush_in_progress && count++ < 5)
        {
            wiced_rtos_delay_milliseconds(1);
        }

        if (audio->flush_in_progress)
        {
            /*
             * Not a fatal error since the main part of the flush operation will
             * have the buffer mutex locked. Shutting down the audio at the beginning
             * of the flush doesn't doesn't though.
             */

            wiced_log_msg(WLF_AUDIO, WICED_LOG_WARNING, "Adding audio during flush\n");
        }
    }

    /*
     * Grab the mutex.
     */

    result = wiced_rtos_lock_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock audio mutex %d\n", result);
        return WICED_ERROR;
    }

    if (buf == NULL && audio->buffer_tail != NULL)
    {
        /*
         * A NULL buffer signals EOS. Just tag the last buffer in the list with
         * the EOS flag.
         */

        audio->buffer_tail->flags = AUDIO_RENDER_BUF_FLAG_EOS;
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Setting EOS flag on last buffer (%u)\n", audio->buffer_bytes_avail);
    }
    else if (audio->buffer_free_list != NULL)
    {
        /*
         * Grab a node off of the free list.
         */

        buf_ptr = audio->buffer_free_list;
        audio->buffer_free_list = audio->buffer_free_list->next;
        audio->num_buffer_used++;

        if (buf != NULL)
        {
            audio->last_pts_pushed = (int64_t)buf->pts;
            buf_ptr->buf   = *buf;
            buf_ptr->flags = 0;
        }
        else
        {
            /*
             * A NULL buffer pointer signals EOS.
             */

            memset(&buf_ptr->buf, 0, sizeof(buf_ptr->buf));
            buf_ptr->flags = AUDIO_RENDER_BUF_FLAG_EOS;
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Setting EOS flag on empty buffer\n");
        }

        buf_ptr->config = audio->buffer_config;
        buf_ptr->next   = NULL;

        audio->buffer_bytes_avail += buf_ptr->buf.data_length;

        /*
         * And add it to the list of buffers to be processed.
         */

        if (audio->buffer_head == NULL)
        {
            audio->buffer_head = buf_ptr;
            audio->buffer_tail = buf_ptr;
        }
        else
        {
            audio->buffer_tail->next = buf_ptr;
            audio->buffer_tail       = buf_ptr;
        }

        if (audio->media_time_set == WICED_FALSE && audio->clock_enable == WICED_TRUE && buf != NULL)
        {
            audio->media_time_set = WICED_TRUE;

            /*
             * Verify that the 802.1AS clock API works correctly.
             */

            time_result = get_media_time(audio, &mtime, &audio->clock_set);

            if (time_result == WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "media time - net is %d:%09d, PTS %d:%09d, (PTS-net) %ld usecs (clock_set: %d)\n",
                              PRINT_TIME(mtime), PRINT_TIME(buf->pts), (int32_t)((buf->pts - mtime) / 1000LL), audio->clock_set);
            }
            else
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to retrieve base media time - disabling clock\n");
                audio->clock_enable = WICED_FALSE;
            }
        }

        audio->err_msg = WICED_FALSE;
    }
    else
    {
        if (buf != NULL)
        {
            if (audio->err_msg == WICED_FALSE)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No free buffers in audio list (%lu used out of %lu)  bytes_queued %lu\n",
                              audio->num_buffer_used, audio->num_buffer_nodes, audio->buffer_bytes_avail);
                audio->err_msg = WICED_TRUE;
            }
        }
        else
        {
            /*
             * The free list is empty but there was no buffer tail above to tag for EOS...strange.
             */

            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No buffers in free list but buffer tail is NULL\n");
        }

        result = WICED_ERROR;
    }

    /*
     * Don't forget to unlock the mutex before we return.
     */

    wiced_rtos_unlock_mutex(&audio->mutex);

    /*
     * Tell the main loop that there is a buffer to process.
     */

    wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_PROCESS_BUFFER);

    return result;
}


wiced_result_t audio_render_command(audio_render_ref audio, AUDIO_RENDER_CMD_T cmd, void* arg)
{
    wiced_result_t result = WICED_SUCCESS;
    audio_render_queue_weight_t* weight;
    audio_render_stats_t* stats;
    uint32_t volume;
    double volume_in_db;
    float speed_in_ppm;
    wiced_audio_device_ioctl_data_t effect_mode;

    if (audio == NULL || audio->tag != AUDIO_RENDER_TAG_VALID)
    {
        return WICED_BADARG;
    }

    switch (cmd)
    {
        case AUDIO_RENDER_CMD_STOP:
            wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_STOP);
            break;

        case AUDIO_RENDER_CMD_FLUSH:
            wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_FLUSH);
            break;

        case AUDIO_RENDER_CMD_PAUSE:
            wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_PAUSE);
            break;

        case AUDIO_RENDER_CMD_RESUME:
            wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_RESUME);
            break;

        case AUDIO_RENDER_CMD_SET_VOLUME:
            volume = MIN((uint32_t)arg, 100);
            audio->cur_volume = MAX(volume, 0);

            if (audio->sh != NULL)
            {
                volume_in_db = (double)audio->cur_volume * ((audio->maximum_volume - audio->minimum_volume) / 100.0) + audio->minimum_volume;
                return wiced_audio_set_volume(audio->sh, volume_in_db);
            }
            break;

        case AUDIO_RENDER_CMD_SET_SPEED:
            /* Arg is pointer to float speed adjustment in PPM  */
            if (arg == NULL)
            {
                return WICED_BADARG;
            }
            speed_in_ppm = *((float*)arg);
            result = wiced_audio_set_pll_fractional_divider(audio->sh, speed_in_ppm);
            if (result == WICED_SUCCESS)
            {
                audio->speed_in_ppm = speed_in_ppm;
            }
            break;

        case AUDIO_RENDER_CMD_GET_QUEUE_WEIGHT:
            /* Arg is pointer to audio_render_queue_weight_t    */
            if (arg == NULL)
            {
                return WICED_BADARG;
            }
            weight = (audio_render_queue_weight_t*)arg;
            weight->input_queue_weight = audio->buffer_bytes_avail;
            weight->output_buffer_size = audio->period_size * audio->num_period_buffers;
            wiced_audio_get_current_buffer_weight(audio->sh, &weight->output_buffer_weight);
            break;

        case AUDIO_RENDER_CMD_GET_STATS:
            /* Arg is pointer to audio_render_stats_t           */
            if (arg == NULL)
            {
                return WICED_BADARG;
            }
            stats = (audio_render_stats_t*)arg;
            stats->audio_frames_played              = audio->audio_frames_played;
            stats->audio_frames_dropped             = audio->audio_frames_dropped;
            stats->audio_frames_inserted            = audio->audio_frames_inserted;
            stats->audio_max_early                  = audio->audio_max_early;
            stats->audio_max_late                   = audio->audio_max_late;
            stats->audio_underruns                  = audio->audio_underruns;
            break;

        case AUDIO_RENDER_CMD_SET_EFFECT:
            effect_mode.dsp_effect_mode = (uint8_t)((uint32_t)arg);
            result = wiced_audio_device_ioctl(audio->sh, WICED_AUDIO_IOCTL_SET_DSP_EFFECT, &effect_mode);
            break;

        case AUDIO_RENDER_CMD_SET_BUFFER_MS:
            /* Arg is straight uint32_t msec buffering value */
            audio->buffer_ns = ((uint32_t)arg) * (int64_t)NUM_NSECONDS_IN_MSECOND;
            break;

        case AUDIO_RENDER_CMD_SET_THRESHOLD_NS:
            /* Arg is pointer to int64_t nanosecond threshold window value */
            if (arg == NULL)
            {
                return WICED_BADARG;
            }
            audio->threshold_ns = *((int64_t*)arg);
            break;

        default:
            return WICED_BADARG;
    }

    return result;
}
