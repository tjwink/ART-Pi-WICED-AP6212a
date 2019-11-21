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

#ifndef BUFFEREDTRACEPRINT_H_
#define BUFFEREDTRACEPRINT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef TRACE_ENABLE_BUFFERED_PRINT
#define TRACE_PROCESS_T_BUFFERED_PRINT              \
    {                                               \
        (char*) "print",                            \
        buffered_trace_print_process_trace,         \
        buffered_trace_print_flush_trace            \
    },
#else
#define TRACE_PROCESS_T_BUFFERED_PRINT
#endif /* TRACE_ENABLE_BUFFERED_PRINT */

/******************************************************
 *      Function definitions
 ******************************************************/
void buffered_trace_print_process_trace( void );
void buffered_trace_print_flush_trace( void );


/******************************************************
 *      Output graph formatting
 ******************************************************/

typedef enum
{
    Output_Style_CSV,
    Output_Style_Table
} trace_output_style_t;

typedef enum
{
    Output_TimeFormat_AbsoluteTicks,
    Output_TimeFormat_RelativeTicks,
    Output_TimeFormat_AbsoluteMilliseconds,
    Output_TimeFormat_RelativeMilliseconds
} trace_output_timeformat_t;

/** Flags */
#define FLAG_FILL_IN_BLANKS     0x00000001  /* Extrapolate data */
#define FLAG_SHOW_TASK_NAMES    0x00000002
#define FLAG_SHOW_LEGEND        0x00000004

#define do_Fill_In_Blanks(format)   ((format.flags & FLAG_FILL_IN_BLANKS )  != 0 )
#define do_Show_Task_Names(format)  ((format.flags & FLAG_SHOW_TASK_NAMES ) != 0 )
#define do_Show_Legend(format)      ((format.flags & FLAG_SHOW_LEGEND )     != 0 )

struct trace_output_format_t
{
    trace_output_style_t style;
    trace_output_timeformat_t time;
    int flags;
};
typedef struct trace_output_format_t trace_output_format_t;

/******************************************************************************/
/**     Table output                                                          */
/******************************************************************************/

/** String to act as the header of the time column */
#define TABLE_TIME_HEADER           "| Time    |"

/** Character to be repeated to separate table data from headers */
#define TABLE_SEPARATOR             '='

/** String to appear at the end of each line of data */
#define TABLE_ENDLINE               " |\r\n"

/** Character to be used to "blank out" a task when there is no action */
#define TABLE_TASK_BLANK            ' '

/** Format string for each line of data - %s refers to the data */
#define TABLE_LINE_FORMAT           "| %7lu |%s |\r\n"
#define TABLE_LINE_FORMAT_CLOCKTIME "| %7lu |%s | %lu us\r\n"

/**
 * Format string for each line of data in which the time has already been
 * printed on a previous line.
 */
#define TABLE_LINE_FORMAT_NOTIME    "|         |%s |\r\n"
#define TABLE_LINE_FORMAT_NOTIME_CLOCKTIME "|         |%s | %lu us\r\n"

/** Print a separator line. */
#define TABLE_PRINT_SEPERATOR( width )          \
    do                                          \
    {                                           \
        unsigned int i;                         \
        for ( i = 0; i < width; i++ )           \
        {                                       \
            printf( "%c", TABLE_SEPARATOR );    \
        }                                       \
        printf( "\r\n" );                       \
    }                                           \
    while ( 0 )

/** Reset (blank) the data output buffer. */
#define TABLE_RESET_LINE( array, width )        \
    do                                          \
    {                                           \
        unsigned int i;                         \
        for ( i = 0; i < width; i++ )           \
        {                                       \
            array[i] = TABLE_TASK_BLANK;        \
        }                                       \
    }                                           \
    while( 0 )

/** Transform each trace action */
#define TABLE_TRANSFORM_LINE( array, width )    \
    do                                          \
    {                                           \
        unsigned int i;                         \
        for ( i = 0; i < width ; i++ )          \
        {                                       \
            trace_action_t action = char_to_traceaction( array[i] ); \
            trace_action_t subsequent_action = subsequent_traceaction( action ); \
            array[i] = traceaction_to_char( subsequent_action ); \
        }                                       \
    }                                           \
    while( 0 )

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* BUFFEREDTRACEPRINT_H_ */
