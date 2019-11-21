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
 *  STM32 HAL based RTC implementation
 */
#include <stdint.h>
#include <string.h>
#include "platform.h"
#include "platform_config.h"
#include "platform_peripheral.h"
#include "wwd_rtos.h"
#include "wwd_assert.h"

/******************************************************
 *                      Macros
 ******************************************************/

/* Check to identify a leap year */
#define IS_LEAP_YEAR(year) ( (((((year) % 4) == 0) && (((year) % 100) != 0)) || (((year) % 400) == 0)) ? (1) : (0) )

/*
 * NOTE:
 * The RTC block will use a 1 Hz counter clock.
 * This means a time counter accuracy of 1 second.
 */
#define NUM_SECONDS_IN_MINUTE       (60)
#define NUM_MINUTES_IN_HOUR         (60)
#define NUM_HOURS_IN_DAY            (24)
#define NUM_DAYS_IN_WEEK            (7)
#define NUM_MONTHS_IN_YEAR          (12)
#define NUM_YEARS_IN_CENTURY        (100)

#define RTC_CURRENT_CENTURY         (20)

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

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/* RTC handler declaration */
static RTC_HandleTypeDef rtc_handle;

/* Default RTC time. Set to 00:00:00 01/01/2000 Saturday */
static const platform_rtc_time_t default_rtc_time =
{
   .sec     = 00,
   .min     = 00,
   .hr      = 00,
   .weekday = 7,
   .date    = 1,
   .month   = 1,
   .year    = 00,
};

static const char month_days_not_leap_year[] =
{
    0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

static const char month_days_in_leap_year[] =
{
    0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_bool_t validate_input_time_calendar( const platform_rtc_time_t* time )
{
    wiced_bool_t valid_time_calendar = WICED_FALSE;

    if ( time->sec < NUM_SECONDS_IN_MINUTE )
    {
        if ( time->min < NUM_MINUTES_IN_HOUR )
        {
            if ( time->hr < NUM_HOURS_IN_DAY )
            {
                if ( (time->weekday > 0) && (time->weekday <= NUM_DAYS_IN_WEEK) )
                {
                    if ( (time->month > 0) && (time->month <= NUM_MONTHS_IN_YEAR) )
                    {
                        if ( time->year < NUM_YEARS_IN_CENTURY )
                        {
                            if ( IS_LEAP_YEAR((RTC_CURRENT_CENTURY * 100) + time->year) == 1 )
                            {
                                if ( (time->date > 0) && (time->date <= month_days_in_leap_year[time->month]) )
                                {
                                    valid_time_calendar = WICED_TRUE;
                                }
                            }
                            else
                            {
                                if ( (time->date > 0) && (time->date <= month_days_not_leap_year[time->month]) )
                                {
                                    valid_time_calendar = WICED_TRUE;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return valid_time_calendar;
}

static platform_result_t platform_rtc_read_time_calendar( platform_rtc_time_t* time )
{
    RTC_TimeTypeDef rtc_time;
    RTC_DateTypeDef rtc_date;

    /* Get current RTC time */
    if ( HAL_RTC_GetTime(&rtc_handle, &rtc_time, RTC_FORMAT_BIN) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    /* Get current RTC date */
    if ( HAL_RTC_GetDate(&rtc_handle, &rtc_date, RTC_FORMAT_BIN) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    time->sec     = rtc_time.Seconds;
    time->min     = rtc_time.Minutes;
    time->hr      = rtc_time.Hours;
    time->date    = rtc_date.Date;
    time->month   = rtc_date.Month;
    time->year    = rtc_date.Year;

    switch ( rtc_date.WeekDay )
    {
        case RTC_WEEKDAY_SUNDAY:
            time->weekday = 1;
            break;

        case RTC_WEEKDAY_MONDAY:
            time->weekday = 2;
            break;

        case RTC_WEEKDAY_TUESDAY:
            time->weekday = 3;
            break;

        case RTC_WEEKDAY_WEDNESDAY:
            time->weekday = 4;
            break;

        case RTC_WEEKDAY_THURSDAY:
            time->weekday = 5;
            break;

        case RTC_WEEKDAY_FRIDAY:
            time->weekday = 6;
            break;

        case RTC_WEEKDAY_SATURDAY:
            time->weekday = 7;
            break;
    }

    return PLATFORM_SUCCESS;
}

static platform_result_t platform_rtc_write_time_calendar( const platform_rtc_time_t* time )
{
    RTC_DateTypeDef  rtc_date;
    RTC_TimeTypeDef  rtc_time;

    /* Validate the input time and calendar settings */
    if ( validate_input_time_calendar( time ) != WICED_TRUE )
    {
        return PLATFORM_ERROR;
    }

    rtc_date.Year = time->year;
    rtc_date.Month = time->month;
    rtc_date.Date = time->date;

    switch ( time->weekday )
    {
        case 1:
            rtc_date.WeekDay = RTC_WEEKDAY_SUNDAY;
            break;

        case 2:
            rtc_date.WeekDay = RTC_WEEKDAY_MONDAY;
            break;

        case 3:
            rtc_date.WeekDay = RTC_WEEKDAY_TUESDAY;
            break;

        case 4:
            rtc_date.WeekDay = RTC_WEEKDAY_WEDNESDAY;
            break;

        case 5:
            rtc_date.WeekDay = RTC_WEEKDAY_THURSDAY;
            break;

        case 6:
            rtc_date.WeekDay = RTC_WEEKDAY_FRIDAY;
            break;

        case 7:
            rtc_date.WeekDay = RTC_WEEKDAY_SATURDAY;
            break;
    }

    /* Set RTC date */
    if( HAL_RTC_SetDate(&rtc_handle, &rtc_date, RTC_FORMAT_BIN) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    rtc_time.Hours = time->hr;
    rtc_time.Minutes = time->min;
    rtc_time.Seconds = time->sec;
    rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;

    /* Set RTC time */
    if ( HAL_RTC_SetTime(&rtc_handle, &rtc_time, RTC_FORMAT_BIN) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    return PLATFORM_SUCCESS;
}

platform_result_t platform_rtc_get_time( platform_rtc_time_t* time )
{
    if ( time == NULL )
    {
        wiced_assert( "bad argument", 0 );
        return PLATFORM_BADARG;
    }

    return platform_rtc_read_time_calendar( time );
}

platform_result_t platform_rtc_set_time( const platform_rtc_time_t* time )
{
    if ( time == NULL )
    {
        wiced_assert( "bad argument", 0 );
        return PLATFORM_BADARG;
    }

    return platform_rtc_write_time_calendar( time );
}

platform_result_t platform_rtc_init( void )
{
    RCC_PeriphCLKInitTypeDef rtc_clock;

    /* Enable backup domain access */
    HAL_PWR_EnableBkUpAccess();

    rtc_clock.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    rtc_clock.RTCClockSelection    = RTC_CLOCK_SOURCE;

    /* Initialize RTC clock source */
    if ( HAL_RCCEx_PeriphCLKConfig(&rtc_clock) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    /* Enable RTC clock */
    __HAL_RCC_RTC_ENABLE();

    rtc_handle.Instance            = RTC;
    rtc_handle.Init.HourFormat     = RTC_HOURFORMAT_24;
    rtc_handle.Init.AsynchPrediv   = RTC_PRESCALER_A;
    rtc_handle.Init.SynchPrediv    = RTC_PRESCALER_S;
    rtc_handle.Init.OutPut         = RTC_OUTPUT_DISABLE;
    rtc_handle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    rtc_handle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

    if ( HAL_RTC_Init(&rtc_handle) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    /* Set default RTC time */
    if ( platform_rtc_write_time_calendar(&default_rtc_time) != PLATFORM_SUCCESS )
    {
        return PLATFORM_ERROR;
    }

    return PLATFORM_SUCCESS;
}
