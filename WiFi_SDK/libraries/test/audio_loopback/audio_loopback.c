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
 * Audio test Application
 *
 * This program tests the board's audio functionality.
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Plug in speaker or headphones into the DAC plug
 *   Quick Start Guide
 *
 *   After download, the app initializes audio on platform,
 *   validates if tx or rx, and performs loop iterations
 *
 */

#include "wiced.h"
#include "wiced_audio.h"
#include "platform_audio.h"
#include "audio_loopback.h"
#include "wiced_log.h"

#include <math.h>
#include "kiss_fftr.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define BYTES_TO_MILLISECONDS(number_of_bytes)     (((MICROSECONDS_PER_SECOND/config.sample_rate) * number_of_bytes)/MILLISECONDS_PER_SECOND)

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef LOOPBACK_WORKER_THREAD_STACK_SIZE
#define LOOPBACK_WORKER_THREAD_STACK_SIZE  (1024)
#endif
#define LOOPBACK_WORKER_THREAD_QUEUE_SIZE  (4)

#ifndef TEST_TX_AUDIO_DEVICE
#define TEST_TX_AUDIO_DEVICE         PLATFORM_DEFAULT_AUDIO_OUTPUT
#endif  /* TEST_TX_AUDIO_DEVICE */                                  /* To specify, change to a valid device for the platform
                                                                     *  See: platforms/<platform>/platform.h
                                                                     *       platforms/<platform>/platform_audio.c
                                                                     *       WICED/platform/include/platform_audio.h
                                                                     *  example: for BCM943909WCD1_3, specify
                                                                     *      AUDIO_DEVICE_ID_AK4954_DAC_LINE
                                                                     *  or
                                                                     *      AUDIO_DEVICE_ID_WM8533_DAC_LINE
                                                                     */

#ifndef TEST_RX_AUDIO_DEVICE
#define TEST_RX_AUDIO_DEVICE         PLATFORM_DEFAULT_AUDIO_INPUT
#endif /* TEST_RX_AUDIO_DEVICE */                                   /* To specify, change to a valid device for the platform
                                                                     *  See: platforms/<platform>/platform.h
                                                                     *       platforms/<platform>/platform_audio.c
                                                                     *       WICED/platform/include/platform_audio.h
                                                                     *  example: for BCM943909WCD1_3, specify
                                                                     *      AUDIO_DEVICE_ID_AK4954_ADC_LINE
                                                                     *  or
                                                                     *      AUDIO_DEVICE_ID_SPDIF
                                                                     */

#define PERIOD_SIZE                 WICED_AUDIO_DEVICE_PERIOD_SIZE
#define BUFFER_SIZE                 WICED_AUDIO_BUFFER_ARRAY_DIM_SIZEOF(WICED_AUDIO_DEVICE_PERIODS, PERIOD_SIZE)

#define EXTRA_MILLIS                (10)
#define TX_START_THRESHOLD_PERIODS  (3)
#define TX_START_THRESHOLD          (TX_START_THRESHOLD_PERIODS*PERIOD_SIZE)

#ifndef SAMPLE_FREQUENCY_IN_HZ
#define SAMPLE_FREQUENCY_IN_HZ      (44100)
#endif /* SAMPLE_FREQUENCY_IN_HZ */

#define MICROSECONDS_PER_SECOND     (1000*1000)
#define MILLISECONDS_PER_SECOND     (1000)
#define BITS_PER_BYTE               (8)

/* The test signal frequency has to be in the range of 100 to 22000Hz with 100Hz steps*/
#define SINE_WAVE_FREQUENCY_IN_HZ   (2000)
#define SINE_WAVE_VOLUME_PERCENTAGE (0.8F)

#define MAX_DATA_SIZE_IN_BYTES      (1024*2)

#define FFT_FREQ_TOLERANCE_PERCENT  (10)
#define TARGET_FRQ_MIN              (SINE_WAVE_FREQUENCY_IN_HZ - (SINE_WAVE_FREQUENCY_IN_HZ/FFT_FREQ_TOLERANCE_PERCENT))
#define TARGET_FRQ_MAX              (SINE_WAVE_FREQUENCY_IN_HZ + (SINE_WAVE_FREQUENCY_IN_HZ/FFT_FREQ_TOLERANCE_PERCENT))

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

static wiced_result_t initialize_library( void );
static wiced_result_t initialize_audio_device( const platform_audio_device_id_t device_id, wiced_audio_config_t* config, uint8_t* buffer, size_t buffer_length, wiced_audio_session_ref* session );
static wiced_result_t loopback_thread( void* unused );
static wiced_result_t loop_iteration( void );
static wiced_result_t get_audio_data( uint8_t* buffer, uint16_t buffer_length );

static uint16_t initialize_data( float volume, uint tone_frequency_in_hz, const wiced_audio_config_t* config, uint8_t* buffer, uint16_t buffer_length_in_bytes );
static wiced_result_t copy_data( uint8_t* buffer, uint16_t buffer_length );
#ifdef FFT_VALIDATION
static wiced_result_t data_fft(uint16_t sample_rate, const kiss_fft_scalar* in, uint16_t cnt, kiss_fft_cpx* out, int32_t* result_hz);
#endif
static wiced_result_t validate_data( uint8_t* buffer, uint16_t buffer_length );
static wiced_result_t validate_rx_data( uint16_t available_bytes );
static void print_buffer(const char *title, uint8_t* buffer, uint16_t buffer_length );

/******************************************************
 *               Variables Definitions
 ******************************************************/

static wiced_bool_t          initialized;
static wiced_bool_t          running;
static wiced_bool_t          stop;
static wiced_semaphore_t     sem;
static wiced_worker_thread_t worker_thread;
static wiced_result_t        loop_result;

static audio_loopback_config_t lib_config =
{
    .enable_sinewave = WICED_FALSE,
    .enable_data_validation = WICED_FALSE,
    .use_fft = WICED_FALSE,
    .use_hamming_window = WICED_TRUE,
    .tx_audio_device = TEST_TX_AUDIO_DEVICE,
    .rx_audio_device = TEST_RX_AUDIO_DEVICE,
};

/*RX and TX buffers to be allocated dynamically */
static uint8_t                  *tx_buffer;
static uint8_t                  *rx_buffer;

static wiced_audio_session_ref  tx_session;
static wiced_audio_session_ref  rx_session;
static int                      is_tx_started;
static int                      last_pos;
static int                      iterations;
static int                      iter_num;

static wiced_audio_config_t config =
{
    .sample_rate        = SAMPLE_FREQUENCY_IN_HZ,
    .channels           = 2,
    .bits_per_sample    = 16,
    .frame_size         = 4,
};

static int16_t  data[MAX_DATA_SIZE_IN_BYTES/4]; /* to hold 441 samples @44.1kHz */
static int      number_of_data_samples;

#ifdef FFT_VALIDATION

#define FFT_DATA_SIZE (PERIOD_SIZE/2) /* just one channel, 2 bytes per sample */
static kiss_fft_scalar fft_input_data[FFT_DATA_SIZE];
static kiss_fft_cpx fft_result[FFT_DATA_SIZE/2 + 1];
static kiss_fftr_cfg fft_cfg;
static uint8_t *fft_buffer;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t wiced_audio_loopback_config(audio_loopback_config_t *config)
{
    if (config)
    {
        memcpy(&lib_config, config, sizeof(audio_loopback_config_t));
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_ERROR;
    }
}

wiced_result_t wiced_audio_loopback_run( uint32_t iter )
{
    wiced_result_t result;

    if ( iter < (1 + TX_START_THRESHOLD / PERIOD_SIZE) &&
         lib_config.enable_data_validation )
    {
        // With validation on, we need at least (TX_START_THRESHOLD / PERIOD_SIZE)
        // periods to fill the buffer and start TX, then at least one period for RX
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Please specify minimum %d iterations\n",
                      (1 + TX_START_THRESHOLD / PERIOD_SIZE));
        return WICED_ERROR;
    }

    if ( initialize_library() != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "library initialization failed\n");
        return WICED_ERROR;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "library initialization success\n");

    result = wiced_rtos_get_semaphore( &sem, WICED_WAIT_FOREVER );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_rtos_get_semaphore failed\n");
        wiced_assert("wiced_rtos_get_semaphore loopback start",
                     result == WICED_SUCCESS);
        return WICED_ERROR;
    }

    if ( running == WICED_TRUE )
    {
        wiced_rtos_set_semaphore( &sem );
        wiced_log_msg(WLF_AUDIO, WICED_LOG_WARNING, "already running\n");
        return WICED_SUCCESS;
    }

    stop = WICED_FALSE;
    running = WICED_TRUE;
    iter_num = iter;

#ifdef FFT_VALIDATION
    if (lib_config.use_fft == WICED_TRUE)
    {
        if ((fft_cfg = kiss_fftr_alloc(FFT_DATA_SIZE, 0/*is_inverse_fft*/, NULL, NULL)) == NULL)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "FFT lib init error\n");
            return WICED_ERROR;
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "FFT lib init success\n");
        }
    }
#endif

    /* Schedule the loopback worker thread to run. */
    result = wiced_rtos_send_asynchronous_event( &worker_thread,
                                                 loopback_thread,
                                                 (void*)iter );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_rtos_send_asynchronous_event failed\n");
        wiced_assert("wiced_rtos_send_asynchronous_event loopback start",
                     result == WICED_SUCCESS);
        running = WICED_FALSE;
        result = WICED_ERROR;
        goto exit;
    }

    if ( iter_num > 0 )
    {
        /* Wait until all iteration have finished */
        while ( running == WICED_TRUE )
        {
            wiced_rtos_delay_milliseconds( 50 );
        }
        result = loop_result;
    }

exit:

#ifdef FFT_VALIDATION
    if (fft_cfg)
    {
        free(fft_cfg);
    }
#endif
    wiced_rtos_set_semaphore(&sem);

    return result;
}

wiced_result_t wiced_audio_loopback_start( void )
{
    return wiced_audio_loopback_run( 0 );
}

wiced_result_t wiced_audio_loopback_stop( void )
{
    wiced_result_t result;

    if ( initialized == WICED_FALSE )
    {
        /* There is nothing to stop. */
        return WICED_SUCCESS;
    }

    result = wiced_rtos_get_semaphore( &sem, WICED_WAIT_FOREVER );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_rtos_get_semaphore failed\n");
        wiced_assert("wiced_rtos_get_semaphore loopback stop",
                     result == WICED_SUCCESS);
        return WICED_ERROR;
    }

    if ( running == WICED_FALSE )
    {
        wiced_rtos_set_semaphore( &sem );
        return WICED_SUCCESS;
    }

    /* Signal the loopback thread to stop and wait for it. */
    stop = WICED_TRUE;
    while ( running == WICED_TRUE )
    {
        wiced_rtos_delay_milliseconds( 50 );
    }

    wiced_rtos_set_semaphore( &sem );

#ifdef FFT_VALIDATION
    if (fft_cfg)
    {
        free(fft_cfg);
    }
#endif

    return WICED_SUCCESS;
}

static wiced_result_t initialize_library( void )
{
    wiced_result_t result;

    if ( initialized == WICED_TRUE )
    {
        return WICED_SUCCESS;
    }

    result = wiced_rtos_init_semaphore( &sem );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_rtos_init_semaphore failed\n");
        wiced_assert("wiced_rtos_init_semaphore", result == WICED_SUCCESS);
        return WICED_ERROR;
    }

    wiced_rtos_set_semaphore( &sem );

    /* Create worker thread for audio loopback. */
    result = wiced_rtos_create_worker_thread( &worker_thread,
                                              WICED_DEFAULT_WORKER_PRIORITY,
                                              LOOPBACK_WORKER_THREAD_STACK_SIZE,
                                              LOOPBACK_WORKER_THREAD_QUEUE_SIZE );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_rtos_create_worker_thread failed\n");
        wiced_assert("wiced_rtos_create_worker_thread", result == WICED_SUCCESS);
        wiced_rtos_deinit_semaphore( &sem );
        return result;
    }

    if (lib_config.enable_sinewave)
    {
        /* Initialize sample data. */
        number_of_data_samples = initialize_data( SINE_WAVE_VOLUME_PERCENTAGE,
                                                  SINE_WAVE_FREQUENCY_IN_HZ,
                                                  &config, (uint8_t *)data,
                                                  sizeof data );
    }

    initialized = WICED_TRUE;
    return WICED_SUCCESS;
}

static wiced_result_t initialize_audio_device( const platform_audio_device_id_t device_id, wiced_audio_config_t* config, uint8_t* buffer, size_t buffer_length, wiced_audio_session_ref* session )
{
    wiced_result_t result = WICED_SUCCESS;

    /* Initialize device. */
    result = wiced_audio_init( device_id, session, PERIOD_SIZE );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_init failed\n");
        wiced_assert("wiced_audio_init", WICED_SUCCESS == result);
        return result;
    }

    /* Allocate audio buffer. */
    result = wiced_audio_create_buffer( *session, buffer_length,
                                        WICED_AUDIO_BUFFER_ARRAY_PTR( buffer ),
                                        NULL );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_create_buffer failed\n");
        wiced_assert("wiced_audio_create_buffer", WICED_SUCCESS == result);
        goto exit_with_error;
    }

    /* Configure session. */
    result = wiced_audio_configure( *session, config );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_configure failed\n");
        wiced_assert("wiced_audio_configure", WICED_SUCCESS == result);
        goto exit_with_error;
    }

    return result;

exit_with_error:
    if ( wiced_audio_deinit( *session ) != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_deinit failed\n");
        wiced_assert("wiced_audio_deinit", 0);
    }

    return result;
}

static uint16_t initialize_data( float volume, uint tone_frequency_in_hz, const wiced_audio_config_t* config, uint8_t* buffer, uint16_t buffer_length_in_bytes )
{
    int         sample_index             = 0;
    const uint  sample_frequency_in_hz  = config->sample_rate;
    const uint  bytes_per_sample        = config->bits_per_sample / 8;
    int16_t     *buf16                  = (int16_t *) buffer;
    uint16_t    total_samples_written   = 0;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "tone_frequency_in_hz: %d\n", tone_frequency_in_hz);

    while (1)
    {
        float rad    = (2.0 * M_PI) * sample_index /
                       (((float)sample_frequency_in_hz / tone_frequency_in_hz));
        float v      = sinf(rad) * volume;
        int16_t data = (int16_t)(v < 0 ? -INT16_MIN * v : INT16_MAX * v) & 0xFFFF;

        buf16[ sample_index ] = data;

        if ( sample_index > 0 && ((tone_frequency_in_hz * (sample_index+1) %
                                   sample_frequency_in_hz) == 0) )
        {
            total_samples_written = sample_index + 1;
            break;
        }

        sample_index++;
        wiced_assert( "frame buffer too small", buffer_length_in_bytes >=
                     (sample_index * bytes_per_sample) );
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "total_samples_written: %d\n", total_samples_written);
    return total_samples_written;
}

static wiced_result_t copy_data( uint8_t* buffer, uint16_t buffer_length )
{
    int i;
    static int last_pos;

    for ( i = 0; i < buffer_length / 2; )
    {
        int16_t *buf16 = (int16_t *) buffer;
        buf16[ i++ ] = data[ last_pos ];
        buf16[ i++ ] = data[ last_pos ];

        if ( ++last_pos >= number_of_data_samples )
        {
            last_pos = 0;
        }
    }

    return WICED_SUCCESS;
}

static wiced_result_t validate_rx_data( uint16_t available_bytes )
{
    wiced_result_t  result = WICED_SUCCESS;
    uint16_t        remaining = available_bytes;

    while ( remaining != 0 && result == WICED_SUCCESS )
    {
        uint8_t* buffer;
        uint16_t avail = remaining;

        result = wiced_audio_get_buffer( rx_session, &buffer, &avail );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_get_buffer RX failed\n");
            wiced_assert("wiced_audio_get_buffer", result == WICED_SUCCESS);
            break;
        }
        if ( avail > remaining )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "bad buffer size\n");
            wiced_assert("bad size", avail <= remaining);
            result = WICED_ERROR;
            break;
        }

        if ( is_tx_started && lib_config.enable_data_validation )
        {
            if(lib_config.use_fft == WICED_FALSE)
            {
                result = validate_data( buffer, avail );
                if ( result != WICED_SUCCESS )
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "validate_data failed\n");
                    wiced_assert("validate_data", result == WICED_SUCCESS);
                    break;
                }
            }
            else
            {
#ifdef FFT_VALIDATION
                /* copy available bytes into a gathering buffer*/
                if(fft_buffer && ((available_bytes - remaining + avail) <= BUFFER_SIZE))
                {
                    memcpy((void *)&fft_buffer[available_bytes - remaining], (void *)buffer, avail);

                }
                else
                {
                    result = WICED_ERROR;
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "fft_buffer overflow\n");
                    wiced_assert("buffer overflow", result == WICED_SUCCESS);
                    break;
                }
#endif
            }
        }

        result = wiced_audio_release_buffer( rx_session, avail );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_release_buffer RX failed\n");
            wiced_assert("wiced_audio_release_buffer", result == WICED_SUCCESS);
            break;
        }

        remaining -= avail;

#ifdef FFT_VALIDATION
        /* do fft validation only on entire data frame*/
        if(remaining == 0 && lib_config.use_fft == WICED_TRUE)
        {
            int32_t frq_hz = 0;
            static int retries;
            static wiced_bool_t signal_detected;
            int16_t *ptr = (int16_t *)fft_buffer;

            if ((iter_num - iterations - 1) <= TX_START_THRESHOLD_PERIODS)
            {
                /*clear some variables in the beginning of the each cycle
                  RX won't start until TX_START_THRESHOLD_PERIODS + 1 */
                signal_detected = WICED_FALSE;
                retries = 0;
            }
            /*rx buffer contains interleaving L and R data in 16bits samples
              extract just one channel for fft conversion.
              Note: avail is in bytes */
            for (int i = 0; i < avail/4; i++)
            {
                if(lib_config.use_hamming_window)
                {
                    // hamming window function
                    // w(n)=0.54-0.46*cos(2*Pi*n)
                    //                    ------
                    //                     N-1
                    float wl =  0.54 - 0.46*(cosf(2.0 * M_PI*i/(avail/4 - 1)));
                    fft_input_data[i] = wl*(float)ptr[i*2];
                }
                else
                {
                    fft_input_data[i] = ptr[i*2];
                }
            }
            /* print_buffer("fft_input_data", (uint8_t*)fft_input_data, avail/4); */
            /* round avail data down to even (needed for fft) */
            result = data_fft(config.sample_rate, fft_input_data, ~0x01 & (avail / 2), fft_result, &frq_hz);
            if (result != WICED_SUCCESS)
            {
                /*Couldn't do conversion, bailing out*/
                return result;
            }
            /* fft is discrete, and how it is precise depends on the
               quantity of samples. We should take in account that the
               frequency-domain data bins won't necessarily match our
               fundamental harmonic */
            if (frq_hz >= TARGET_FRQ_MIN && frq_hz <= TARGET_FRQ_MAX )
            {
                /* calculated fundamental frequency is within limits*/
                wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "iter %d: %dHz\n", (iter_num - iterations), (int)frq_hz);
                signal_detected = WICED_TRUE;
            }
            else
            {
                if(signal_detected)
                {
                    wiced_log_printf("Iter: %d, Fundamental frequency: %d, expected from %d to %d\n",
                                     (iter_num - iterations), (int)frq_hz, TARGET_FRQ_MIN, TARGET_FRQ_MAX);
                    print_buffer("rx", (uint8_t*)fft_input_data, avail);
                    result = WICED_ERROR;
                }
                else
                {
                    /* wait for good data*/
                    if( ++retries < (iter_num - TX_START_THRESHOLD_PERIODS) )
                    {
                        if(frq_hz == 0)
                        {
                            /* fft is configured to work on a full frame, hence
                               wait for a valid frame. Until we get it, the main harmonic will be 0.*/
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "iter %d: 0Hz, ignoring\n",
                                          (iter_num - iterations));
                        }
                        else
                        {
                            /* data in the beginning may contain
                               transition artifacts or distorted waveform.*/
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "iter %d: bad %dHz, ignoring\n",
                                          (iter_num - iterations), (int)frq_hz);
                        }
                        result = WICED_SUCCESS;
                    }
                    else
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "failed to get valid input data in %d rx cycles\n", retries);
                        result = WICED_ERROR;
                    }
                }
            }
            if ( result != WICED_SUCCESS )
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "failed to validate data using fft\n");
                wiced_assert("validate_data", result == WICED_SUCCESS);
                break;
            }

        } // if(remaining == 0)
#endif

    }

    return result;
}

static void print_buffer(const char *title, uint8_t* buffer, uint16_t buffer_length )
{
    int i = 0, j = 0;
    uint16_t *data = (uint16_t *)buffer;

    wiced_log_printf("\n%s buffer@%p, length %d\n", title, data, buffer_length);
    for (i = 0; i < buffer_length/2; i++)
    {
        if (j == 0)
        {
            wiced_log_printf("0x%04x: ", 2*i);
        }
        wiced_log_printf("0x%04x ", data[i]);
        if (j++ == 7)
        {
            wiced_log_printf("\n");
            j = 0;
        }
    }
    wiced_log_printf("\n");
}


#ifdef FFT_VALIDATION

static wiced_result_t data_fft(uint16_t sample_rate, const kiss_fft_scalar* in, uint16_t cnt, kiss_fft_cpx* out, int32_t* result_hz)
{
    int32_t magnitude_max = 0, magnitude, bin_freq;

    if (!fft_cfg)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "fft library is not initialized\n");
        return WICED_ERROR;
    }

    /* do the fast fourier transform */
    kiss_fftr(fft_cfg, in, out);

    for (int i = 0; i < cnt/2; i++)
    {
        /* normally magnitude is sqrt(r*r + i*i), but to save cpu cycles, skipping sqrt.
           note: for the selected sample data magnitude range is up to 0x400.0000 max,
           so int32_t should be enough for now, but increase accordingly if needed */
        magnitude = out[i].r * out[i].r + out[i].i * out[i].i;
        bin_freq = (sample_rate * i) / cnt;
        /* wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Freq:%d mag:%d\n", (int)bin_freq, (int)magnitude); */
        if (magnitude > magnitude_max)
        {
            magnitude_max = magnitude;
            *result_hz = bin_freq;
        }
    }

    return WICED_SUCCESS;
}

#endif

static wiced_result_t validate_data( uint8_t* buffer, uint16_t buffer_length )
{
    int i = 0;
    static int allowed = 10;

    if ( last_pos < 0 )
    {
        int pos = 0;
        for ( i = 0; i < buffer_length / 2; i += 2 )
        {
            int16_t *buf16 = (int16_t *) buffer;

            /*skip all zero data that we might receive before starting TX*/
            if ( buf16[ i ] == 0 )
            {
                continue;
            }

            for ( pos = 0; pos < number_of_data_samples; pos++ )
            {
                if ( buf16[ i ] == data[ pos ] )
                {
                    break;
                }
            }
            if ( number_of_data_samples == pos )
            {
                continue;
            }
            if ( pos != number_of_data_samples && buf16[ i + 1 ] == data[ pos ] )
            {
                last_pos = pos;
                break;
            }
        }
    }
    if ( last_pos < 0 )
    {
        --allowed;
        if ( allowed == 0 )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "data pattern not found\n");
            wiced_assert("data pattern not found", allowed != 0);
            return WICED_ERROR;
        }

        return WICED_SUCCESS;
    }

    while ( i < buffer_length / 2 )
    {
        int16_t *buf16 = (int16_t *) buffer;

        if ( buf16[ i ] != data[ last_pos ] )
        {
            char ch = ((i % 2)?'R':'L');
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "iter %d: invalid %c data, offset: 0x%x. Expected 0x%04x, received 0x%04x\n",
                (iter_num - iterations), ch, 2*i, (uint16_t)data[last_pos], (uint16_t)buf16[i]);
            print_buffer("rx buffer", buffer-(36), buffer_length+64);
            wiced_assert("invalid data", 0);
            return WICED_ERROR;
        }

        /* increment position in the test data buffer only if both L and R
           received data verified */
        if (i++ % 2)
        {
            last_pos++;
        }

        if ( last_pos >= number_of_data_samples )
        {
            last_pos = 0;
        }
    }

    return WICED_SUCCESS;
}

/* Copy audio buffer from session to buffer of given size. */
static wiced_result_t get_audio_data( uint8_t* buffer, uint16_t buffer_length )
{
    wiced_result_t          result;
    uint16_t                remaining = buffer_length;
    wiced_audio_session_ref sh        = rx_session;

    result = WICED_SUCCESS;

    if (lib_config.enable_sinewave)
    {
        /* Copy predefined data. */
        result = copy_data( buffer, buffer_length );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "copy RX data failed\n");
            wiced_assert("copy_data", result == WICED_SUCCESS);
            return result;
        }

        /* Validate and eat RX data. */
        if ( rx_session != NULL )
        {
            result = validate_rx_data( buffer_length );
            if ( result != WICED_SUCCESS )
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "validate RX data failed\n");
                wiced_assert("validate_rx_data", result == WICED_SUCCESS);
            }
        }
    }
    else
    {

    while ( 0 != remaining && result == WICED_SUCCESS )
    {
        uint8_t *buf;
        uint16_t avail = remaining;

        result = wiced_audio_get_buffer( sh, &buf, &avail );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_get_buffer failed\n");
            wiced_assert("wiced_audio_get_buffer", result == WICED_SUCCESS);
            break;
        }
        if ( avail > remaining )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "bad size\n");
            wiced_assert("bad size", avail <= remaining);
            result = WICED_ERROR;
            break;
        }

        memcpy( buffer, buf, avail );
        result = wiced_audio_release_buffer( sh, avail );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_release_buffer failed\n");
            wiced_assert("wiced_audio_release_buffer", result == WICED_SUCCESS);
            break;
        }
        buffer    += avail;
        remaining -= avail;
    }
    }

    return result;
}

static wiced_result_t loop_iteration( void )
{
    wiced_result_t result;
    uint16_t       remaining;
    const uint32_t timeout = BYTES_TO_MILLISECONDS(PERIOD_SIZE) + EXTRA_MILLIS;

    result = WICED_SUCCESS;

    /* Start data transmission. */
    if ( !is_tx_started )
    {
        uint32_t weight;

        /* Determine if we should start TX. */
        result = wiced_audio_get_current_buffer_weight( tx_session, &weight );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_get_current_buffer_weight failed\n");
            wiced_assert("wiced_audio_get_current_buffer_weight",
                         result == WICED_SUCCESS);
            return result;
        }

        if ( weight >= TX_START_THRESHOLD )
        {
            result = wiced_audio_start( tx_session );
            if ( result != WICED_SUCCESS )
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_start TX failed\n");
                wiced_assert("wiced_audio_start TX", result == WICED_SUCCESS);
                return result;
            }

            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "TX started\n");
            is_tx_started = 1;
        }
    }

    /* Wait for data that can be transmitted. */
    /* In the case of canned data, this defines the transmit cadence. */
    if ( rx_session != NULL )
    {
        result = wiced_audio_wait_buffer(rx_session, PERIOD_SIZE, timeout);
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_wait_buffer RX failed\n");
            wiced_assert("wiced_audio_wait_buffer RX", result == WICED_SUCCESS);
            return result;
        }
    }

    /* Wait for slot in transmit buffer. */
    result = wiced_audio_wait_buffer( tx_session, PERIOD_SIZE, timeout );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_wait_buffer TX failed (%d) (%d)\n", result, (iter_num - iterations));
        wiced_assert("wiced_audio_wait_buffer TX", result == WICED_SUCCESS);
        return result;
    }

    /* Copy available data to transmit buffer. */
    remaining = PERIOD_SIZE;
    while ( 0 != remaining && result == WICED_SUCCESS )
    {
        uint8_t *buf;
        uint16_t avail = remaining;

        result = wiced_audio_get_buffer(tx_session, &buf, &avail);
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_get_buffer failed\n");
            wiced_assert("wiced_audio_get_buffer", result == WICED_SUCCESS);
            return result;
        }
        if ( avail > remaining )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "bad size\n");
            wiced_assert("bad size", avail <= remaining);
            return WICED_ERROR;
        }

        result = get_audio_data( buf, avail );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "get_data failed\n");
            wiced_assert("get_data", result == WICED_SUCCESS);
            return result;
        }

        result = wiced_audio_release_buffer( tx_session, avail );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_release_buffer TX failed\n");
            wiced_assert("wiced_audio_release_buffer", result == WICED_SUCCESS);
            return result;
        }

        remaining -= avail;
    }

    return WICED_SUCCESS;
}

static wiced_result_t loopback_thread( void* arg )
{
    static wiced_bool_t audio_inited = WICED_FALSE;
    wiced_result_t      result = WICED_SUCCESS;
    iterations = (uint32_t)arg;
    wiced_bool_t        forever = iterations == 0 ? WICED_TRUE : WICED_FALSE;
    wiced_bool_t        use_rx = WICED_FALSE;
    loop_result = WICED_SUCCESS;

    /* Initialize platform audio. */
    if ( audio_inited == WICED_FALSE )
    {
        result = platform_init_audio();
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "initialize platform audio failed\n");
            wiced_assert("platform_init_audio", result == WICED_SUCCESS);
            loop_result = WICED_ERROR;
            return WICED_ERROR;
        }

        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "initialize platform audio success\n");
        audio_inited = WICED_TRUE;
    }

/* FIXME The audio subsystem should tell us what buffer requirements or
 *       do this allocation for us!
 */
    tx_buffer =  (uint8_t*)malloc(BUFFER_SIZE);
    if(tx_buffer == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "tx buffer allocation failed\n");
        wiced_assert("tx buffer allocation", result == WICED_SUCCESS);
        loop_result = WICED_ERROR;
        goto out_deinit_audio;
    }

    if ( lib_config.rx_audio_device != AUDIO_DEVICE_ID_NONE)
    {
        rx_buffer =  (uint8_t*)malloc(BUFFER_SIZE);
        if(rx_buffer == NULL)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "rx buffer allocation failed\n");
            wiced_assert("rx buffer allocation", result == WICED_SUCCESS);
            loop_result = WICED_ERROR;
            goto out_deinit_audio;
        }
    }
#ifdef FFT_VALIDATION
    if ( lib_config.use_fft != WICED_FALSE )
    {
        fft_buffer =  (uint8_t*)malloc(BUFFER_SIZE);
        if(fft_buffer == NULL)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "fft buffer allocation failed\n");
            wiced_assert("fft buffer allocation", result == WICED_SUCCESS);
            loop_result = WICED_ERROR;
            goto out_deinit_audio;
        }
    }
#endif
    /* Initialize TX device. */

    result = initialize_audio_device( lib_config.tx_audio_device, &config, tx_buffer,
                                      BUFFER_SIZE, &tx_session );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "initialize TX audio device failed\n");
        wiced_assert("initialize_audio_device TX", result == WICED_SUCCESS);
        loop_result = WICED_ERROR;
        goto out_deinit_audio;
    }
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "initialize TX audio device success\n");

    if ( lib_config.rx_audio_device != AUDIO_DEVICE_ID_NONE)
    {
        /*init and start RX for all cases except sinewave is on, no data validation*/
        use_rx = (!lib_config.enable_sinewave || lib_config.enable_data_validation);
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "using rx: %s\n", use_rx?"yes":"no");
        if(use_rx)
        {
            /* Initialize RX device. */
            result = initialize_audio_device( lib_config.rx_audio_device, &config, rx_buffer,
                                              BUFFER_SIZE, &rx_session );
            if ( result != WICED_SUCCESS )
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "initialize RX audio device failed\n");
                wiced_assert("initialize_audio_device RX", result == WICED_SUCCESS);
                loop_result = WICED_ERROR;
                goto out_stop_tx;
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "initialize RX audio device success\n");

            /* Start RX. */
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "starting audio\n");
            result = wiced_audio_start( rx_session );
            if ( result != WICED_SUCCESS )
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_start RX failed\n");
                wiced_assert("wiced_audio_start RX", result == WICED_SUCCESS);
                loop_result = WICED_ERROR;
                goto out_stop_rx;
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio started\n");
        }
    }

    last_pos = -1;
    /* Main loop. */
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "starting main loop\n");
    while ((( forever == WICED_TRUE ) && ( stop == WICED_FALSE )) ||
           (( forever == WICED_FALSE ) && ( iterations-- > 0 )))
    {
        result = loop_iteration();
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "loop_iteration failed- bailing!\n");
            wiced_assert("loop_iteration", result == WICED_SUCCESS);
            loop_result = WICED_ERROR;
            break;
        }
    }

    if ( is_tx_started )
    {
        result = wiced_audio_stop( tx_session );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_stop TX failed\n");
            wiced_assert("wiced_audio_stop TX", result == WICED_SUCCESS);
            loop_result = WICED_ERROR;
        }
        is_tx_started = 0;
    }

    if ( lib_config.rx_audio_device != AUDIO_DEVICE_ID_NONE)
    {
        if(use_rx)
        {
            result = wiced_audio_stop( rx_session );
            if ( result != WICED_SUCCESS )
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_stop RX failed\n");
                wiced_assert("wiced_audio_stop RX", result == WICED_SUCCESS);
                loop_result = WICED_ERROR;
            }

    out_stop_rx:
            result = wiced_audio_deinit( rx_session );
            if ( result != WICED_SUCCESS )
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_deinit RX failed\n");
                wiced_assert("wiced_audio_deinit RX", result == WICED_SUCCESS);
                loop_result = WICED_ERROR;
            }
        }
    }
out_stop_tx:
    result = wiced_audio_deinit( tx_session );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_deinit TX failed\n");
        wiced_assert("wiced_audio_deinit TX", result == WICED_SUCCESS);
        loop_result = WICED_ERROR;
    }

out_deinit_audio:
    /*
     * May not support deiniting (43909 does not). In this case keep platform
     * audio initialized. But this is not good as audio including codec is not
     * re-initialized, and other tests (particular I2C test) is talking to
     * codec and may break current test when current test run next time without
     * power-cycling.
     */
    if ( platform_deinit_audio() == WICED_SUCCESS )
    {
        /* Deiniting supported. */
        audio_inited = WICED_FALSE;
    }

    if(tx_buffer)
    {
        free(tx_buffer);
    }
    if(rx_buffer)
    {
        free(rx_buffer);
    }
#ifdef FFT_VALIDATION
    if(fft_buffer)
    {
        free(fft_buffer);
    }
#endif
    running = WICED_FALSE;

    return WICED_SUCCESS;
}

