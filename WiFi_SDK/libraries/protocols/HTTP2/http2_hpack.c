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
 */
#include "wiced.h"
#include "http2_pvt.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define LENGTH_OF(X)                        ( sizeof(X)/sizeof(X[0]) )
#define HPACK_NUM_STATIC_TABLE_ENTRIES      ( 61 )
/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum index_table_type_e
{
    INDEX_TABLE_TYPE_INDEXED,
    INDEX_TABLE_TYPE_LITERAL_WITH_INCREMENT_INDEXING,
    INDEX_TABLE_TYPE_LITERAL_WITHOUT_INDEXING,
    INDEX_TABLE_TYPE_LITERAL_NEVER_INDEXED,
    INDEX_TABLE_TYPE_DYNAMIC_TABLE_SIZE
} index_table_type_t;
/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct huffman_table_entry_s
{
    uint8_t   symbol;
    uint32_t  code;
} huffman_table_entry_t;

typedef struct huffman_table_s
{
    huffman_table_entry_t* entry;
    uint8_t                length;
} huffman_table_t;

typedef struct index_table_entry_s
{
    const char* name;
    const char* value;
} index_table_entry_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
wiced_result_t http_hpack_decode_huffman( uint32_t* value, uint8_t** buffer, uint32_t size );
static wiced_result_t http_hpack_decode_index( uint8_t* type, uint32_t* index, uint8_t** buffer, uint32_t* size );
static wiced_result_t http_hpack_get_indexed_string( uint32_t index, uint8_t is_name, uint8_t** string_buffer, uint32_t* string_buffer_size );
static wiced_result_t http_hpack_get_string( uint8_t** string_buffer, uint32_t* string_buffer_size, uint8_t** buffer, uint32_t* size );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static huffman_table_entry_t huffman_table_5bits[]=
{
    { (48), 0x0},
    { (49), 0x1},
    { (50), 0x2},
    { (97), 0x3},
    { (99), 0x4},
    { (101), 0x5},
    { (105), 0x6},
    { (111), 0x7},
    { (115), 0x8},
    { (116), 0x9},
};
static huffman_table_entry_t huffman_table_6bits[]=
{
    { (32), 0x14},
    { (37), 0x15},
    { (45), 0x16},
    { (46), 0x17},
    { (47), 0x18},
    { (51), 0x19},
    { (52), 0x1a},
    { (53), 0x1b},
    { (54), 0x1c},
    { (55), 0x1d},
    { (56), 0x1e},
    { (57), 0x1f},
    { (61), 0x20},
    { (65), 0x21},
    { (95), 0x22},
    { (98), 0x23},
    { (100), 0x24},
    { (102), 0x25},
    { (103), 0x26},
    { (104), 0x27},
    { (108), 0x28},
    { (109), 0x29},
    { (110), 0x2a},
    { (112), 0x2b},
    { (114), 0x2c},
    { (117), 0x2d},
};
static huffman_table_entry_t huffman_table_7bits[]=
{
    { (58), 0x5c},
    { (66), 0x5d},
    { (67), 0x5e},
    { (68), 0x5f},
    { (69), 0x60},
    { (70), 0x61},
    { (71), 0x62},
    { (72), 0x63},
    { (73), 0x64},
    { (74), 0x65},
    { (75), 0x66},
    { (76), 0x67},
    { (77), 0x68},
    { (78), 0x69},
    { (79), 0x6a},
    { (80), 0x6b},
    { (81), 0x6c},
    { (82), 0x6d},
    { (83), 0x6e},
    { (84), 0x6f},
    { (85), 0x70},
    { (86), 0x71},
    { (87), 0x72},
    { (89), 0x73},
    { (106), 0x74},
    { (107), 0x75},
    { (113), 0x76},
    { (118), 0x77},
    { (119), 0x78},
    { (120), 0x79},
    { (121), 0x7a},
    { (122), 0x7b},
};
static huffman_table_entry_t huffman_table_8bits[]=
{
    { (38), 0xf8},
    { (42), 0xf9},
    { (44), 0xfa},
    { (59), 0xfb},
    { (88), 0xfc},
    { (90), 0xfd},
};
static huffman_table_entry_t huffman_table_10bits[]=
{
    { (33), 0x3f8},
    { (34), 0x3f9},
    { (40), 0x3fa},
    { (41), 0x3fb},
    { (63), 0x3fc},
};
static huffman_table_entry_t huffman_table_11bits[]=
{
    { (39), 0x7fa},
    { (43), 0x7fb},
    { (124), 0x7fc},
};
static huffman_table_entry_t huffman_table_12bits[]=
{
    { (35), 0xffa},
    { (62), 0xffb},
};

static huffman_table_entry_t huffman_table_13bits[]=
{
    { (0), 0x1ff8},
    { (36), 0x1ff9},
    { (64), 0x1ffa},
    { (91), 0x1ffb},
    { (93), 0x1ffc},
    { (126), 0x1ffd},
};
static huffman_table_entry_t huffman_table_14bits[]=
{
    { (94), 0x3ffc},
    { (125), 0x3ffd},
};

static huffman_table_entry_t huffman_table_15bits[]=
{
    { (60), 0x7ffc},
    { (96), 0x7ffd},
    { (123), 0x7ffe},
};

static huffman_table_entry_t huffman_table_19bits[]=
{
    { (92), 0x7fff0},
    { (195), 0x7fff1},
    { (208), 0x7fff2},
};
static huffman_table_entry_t huffman_table_20bits[]=
{
    { (128), 0xfffe6},
    { (130), 0xfffe7},
    { (131), 0xfffe8},
    { (162), 0xfffe9},
    { (184), 0xfffea},
    { (194), 0xfffeb},
    { (224), 0xfffec},
    { (226), 0xfffed},
};
static huffman_table_entry_t huffman_table_21bits[]=
{
    { (153), 0x1fffdc},
    { (161), 0x1fffdd},
    { (167), 0x1fffde},
    { (172), 0x1fffdf},
    { (176), 0x1fffe0},
    { (177), 0x1fffe1},
    { (179), 0x1fffe2},
    { (209), 0x1fffe3},
    { (216), 0x1fffe4},
    { (217), 0x1fffe5},
    { (227), 0x1fffe6},
    { (229), 0x1fffe7},
    { (230), 0x1fffe8},
};
static huffman_table_entry_t huffman_table_22bits[]=
{
    { (129), 0x3fffd2},
    { (132), 0x3fffd3},
    { (133), 0x3fffd4},
    { (134), 0x3fffd5},
    { (136), 0x3fffd6},
    { (146), 0x3fffd7},
    { (154), 0x3fffd8},
    { (156), 0x3fffd9},
    { (160), 0x3fffda},
    { (163), 0x3fffdb},
    { (164), 0x3fffdc},
    { (169), 0x3fffdd},
    { (170), 0x3fffde},
    { (173), 0x3fffdf},
    { (178), 0x3fffe0},
    { (181), 0x3fffe1},
    { (185), 0x3fffe2},
    { (186), 0x3fffe3},
    { (187), 0x3fffe4},
    { (189), 0x3fffe5},
    { (190), 0x3fffe6},
    { (196), 0x3fffe7},
    { (198), 0x3fffe8},
    { (228), 0x3fffe9},
    { (232), 0x3fffea},
    { (233), 0x3fffeb},
};
static huffman_table_entry_t huffman_table_23bits[]=
{
    { (1), 0x7fffd8},
    { (135), 0x7fffd9},
    { (137), 0x7fffda},
    { (138), 0x7fffdb},
    { (139), 0x7fffdc},
    { (140), 0x7fffdd},
    { (141), 0x7fffde},
    { (143), 0x7fffdf},
    { (147), 0x7fffe0},
    { (149), 0x7fffe1},
    { (150), 0x7fffe2},
    { (151), 0x7fffe3},
    { (152), 0x7fffe4},
    { (155), 0x7fffe5},
    { (157), 0x7fffe6},
    { (158), 0x7fffe7},
    { (165), 0x7fffe8},
    { (166), 0x7fffe9},
    { (168), 0x7fffea},
    { (174), 0x7fffeb},
    { (175), 0x7fffec},
    { (180), 0x7fffed},
    { (182), 0x7fffee},
    { (183), 0x7fffef},
    { (188), 0x7ffff0},
    { (191), 0x7ffff1},
    { (197), 0x7ffff2},
    { (231), 0x7ffff3},
    { (239), 0x7ffff4},
};
static huffman_table_entry_t huffman_table_24bits[]=
{
    { (9), 0xffffea},
    { (142), 0xffffeb},
    { (144), 0xffffec},
    { (145), 0xffffed},
    { (148), 0xffffee},
    { (159), 0xffffef},
    { (171), 0xfffff0},
    { (206), 0xfffff1},
    { (215), 0xfffff2},
    { (225), 0xfffff3},
    { (236), 0xfffff4},
    { (237), 0xfffff5},
};
static huffman_table_entry_t huffman_table_25bits[]=
{
    { (199), 0x1ffffec},
    { (207), 0x1ffffed},
    { (234), 0x1ffffee},
    { (235), 0x1ffffef},
};
static huffman_table_entry_t huffman_table_26bits[]=
{
    { (192), 0x3ffffe0},
    { (193), 0x3ffffe1},
    { (200), 0x3ffffe2},
    { (201), 0x3ffffe3},
    { (202), 0x3ffffe4},
    { (205), 0x3ffffe5},
    { (210), 0x3ffffe6},
    { (213), 0x3ffffe7},
    { (218), 0x3ffffe8},
    { (219), 0x3ffffe9},
    { (238), 0x3ffffea},
    { (240), 0x3ffffeb},
    { (242), 0x3ffffec},
    { (243), 0x3ffffed},
    { (255), 0x3ffffee},
};
static huffman_table_entry_t huffman_table_27bits[]=
{
    { (203), 0x7ffffde},
    { (204), 0x7ffffdf},
    { (211), 0x7ffffe0},
    { (212), 0x7ffffe1},
    { (214), 0x7ffffe2},
    { (221), 0x7ffffe3},
    { (222), 0x7ffffe4},
    { (223), 0x7ffffe5},
    { (241), 0x7ffffe6},
    { (244), 0x7ffffe7},
    { (245), 0x7ffffe8},
    { (246), 0x7ffffe9},
    { (247), 0x7ffffea},
    { (248), 0x7ffffeb},
    { (250), 0x7ffffec},
    { (251), 0x7ffffed},
    { (252), 0x7ffffee},
    { (253), 0x7ffffef},
    { (254), 0x7fffff0},
};
static huffman_table_entry_t huffman_table_28bits[]=
{
    { (2), 0xfffffe2},
    { (3), 0xfffffe3},
    { (4), 0xfffffe4},
    { (5), 0xfffffe5},
    { (6), 0xfffffe6},
    { (7), 0xfffffe7},
    { (8), 0xfffffe8},
    { (11), 0xfffffe9},
    { (12), 0xfffffea},
    { (14), 0xfffffeb},
    { (15), 0xfffffec},
    { (16), 0xfffffed},
    { (17), 0xfffffee},
    { (18), 0xfffffef},
    { (19), 0xffffff0},
    { (20), 0xffffff1},
    { (21), 0xffffff2},
    { (23), 0xffffff3},
    { (24), 0xffffff4},
    { (25), 0xffffff5},
    { (26), 0xffffff6},
    { (27), 0xffffff7},
    { (28), 0xffffff8},
    { (29), 0xffffff9},
    { (30), 0xffffffa},
    { (31), 0xffffffb},
    { (127), 0xffffffc},
    { (220), 0xffffffd},
    { (249), 0xffffffe},
};
static huffman_table_entry_t huffman_table_30bits[]=
{
    { (10), 0x3ffffffc},
    { (13), 0x3ffffffd},
    { (22), 0x3ffffffe},
};

/* List of tables of different sizes starting from 5 bit up to 30 bits */
static huffman_table_t huffman_tables[] =
{
    { huffman_table_5bits,  LENGTH_OF( huffman_table_5bits  ) },
    { huffman_table_6bits,  LENGTH_OF( huffman_table_6bits  ) },
    { huffman_table_7bits,  LENGTH_OF( huffman_table_7bits  ) },
    { huffman_table_8bits,  LENGTH_OF( huffman_table_8bits  ) },
    {                NULL,  0 },
    { huffman_table_10bits, LENGTH_OF( huffman_table_10bits ) },
    { huffman_table_11bits, LENGTH_OF( huffman_table_11bits ) },
    { huffman_table_12bits, LENGTH_OF( huffman_table_12bits ) },
    { huffman_table_13bits, LENGTH_OF( huffman_table_13bits ) },
    { huffman_table_14bits, LENGTH_OF( huffman_table_14bits ) },
    { huffman_table_15bits, LENGTH_OF( huffman_table_15bits ) },
    {                 NULL, 0 },
    {                 NULL, 0 },
    {                 NULL, 0 },
    { huffman_table_19bits, LENGTH_OF( huffman_table_19bits ) },
    { huffman_table_20bits, LENGTH_OF( huffman_table_20bits ) },
    { huffman_table_21bits, LENGTH_OF( huffman_table_21bits ) },
    { huffman_table_22bits, LENGTH_OF( huffman_table_22bits ) },
    { huffman_table_23bits, LENGTH_OF( huffman_table_23bits ) },
    { huffman_table_24bits, LENGTH_OF( huffman_table_24bits ) },
    { huffman_table_25bits, LENGTH_OF( huffman_table_25bits ) },
    { huffman_table_26bits, LENGTH_OF( huffman_table_26bits ) },
    { huffman_table_27bits, LENGTH_OF( huffman_table_27bits ) },
    { huffman_table_28bits, LENGTH_OF( huffman_table_28bits ) },
    {                 NULL, 0 },
    { huffman_table_30bits, LENGTH_OF( huffman_table_30bits ) },
};

/* Static index table, size doesn't matter cause it will not change */
static const index_table_entry_t static_index_table[] =
{
        {NULL, NULL},
    {":authority", "" },
    {":method", "GET" },
    {":method", "POST" },
    {":path", "/" },
    {":path", "/index.html" },
    {":scheme", "http" },
    {":scheme", "https" },
    {":status", "200" },
    {":status", "204" },
    {":status", "206" },
    {":status", "304" },
    {":status", "400" },
    {":status", "404" },
    {":status", "500" },
    {"accept-charset", "" },
    {"accept-encoding", "gzip, deflate" },
    {"accept-language", "" },
    {"accept-ranges", "" },
    {"accept", "" },
    {"access-control-allow-origin", "" },
    {"age", "" },
    {"allow", "" },
    {"authorization", "" },
    {"cache-control", "" },
    {"content-disposition", "" },
    {"content-encoding", "" },
    {"content-language", "" },
    {"content-length", "" },
    {"content-location", "" },
    {"content-range", "" },
    {"content-type", "" },
    {"cookie", "" },
    {"date", "" },
    {"etag", "" },
    {"expect", "" },
    {"expires", "" },
    {"from", "" },
    {"host", "" },
    {"if-match", "" },
    {"if-modified-since", "" },
    {"if-none-match", "" },
    {"if-range", "" },
    {"if-unmodified-since", "" },
    {"last-modified", "" },
    {"link", "" },
    {"location", "" },
    {"max-forwards", "" },
    {"proxy-authenticate", "" },
    {"proxy-authorization", "" },
    {"range", "" },
    {"referer", "" },
    {"refresh", "" },
    {"retry-after", "" },
    {"server", "" },
    {"set-cookie", "" },
    {"strict-transport-security", "" },
    {"transfer-encoding", "" },
    {"user-agent", "" },
    {"vary", "" },
    {"via", "" },
    {"www-authenticate", "" }
};

/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_result_t http_hpack_huffman_find( uint32_t code, uint8_t* symbol, huffman_table_t* table )
{
    uint8_t index;
    volatile huffman_table_entry_t* entry = table->entry;
    for ( index = 0; index < table->length; index++ )
    {
        if ( code == entry[ index ].code )
        {
            *symbol = entry[ index ].symbol;
            return WICED_SUCCESS;
        }
    }
    return WICED_ERROR;
}

static wiced_result_t http_hpack_huffman_get_symbol( uint8_t** symbol_buffer, uint32_t* symbol_buffer_size, uint8_t** stream, uint32_t* size, uint8_t* bit_ptr,uint8_t* eos )
{
    volatile uint32_t code;
    uint8_t* byte = *stream;
    volatile uint8_t  bit  = *bit_ptr;
    uint8_t  bit_count = 5; /* minimum huffman code */

    if ( *size == 0 )
    {
        return WICED_ERROR;
    }

    /* get first 5 bits */
    if ( bit >= 4 )
    {
        /* We have more than 5 bits to read */
        uint8_t shift = (uint8_t)(bit - 4);
        code =    (uint32_t)( ( ( *byte ) & ( 0x1F << shift ) ) >> shift );
        bit = (uint8_t)(bit - 4);
    }
    else
    {
        uint8_t remaining = (uint8_t) ( 4 - bit );
        /* Take the last few bits from current byte */
        uint32_t mask = (uint32_t)( ~( 0xFFFFFFFF << ( bit + 1) ) );
        code = (uint32_t) ( *byte & mask );

        if ( *size > 1 )
        {
            code = code << remaining;
            /* Take the remaining bits from the next byte */
            byte++;
            *size = *size - 1;
            bit  = (uint8_t) (7 - remaining + 1);
            code = (uint32_t)( code | ( (uint32_t)(*byte >> (  bit ) ) ) );
        }
        else
        {
            /* This is the last byte, with less than 4 bits, better all be 1s */
            if ( code == mask )
            {
                /* Ok end of huffman, and proceed to next byte */
                *eos = 1;
                byte++;
                *stream = byte;
                return WICED_SUCCESS;
            }
            else
            {
                return WICED_ERROR;
            }
        }
    }

    while ( *size )
    {
        volatile huffman_table_t* table = &huffman_tables[ bit_count - 5 ];
        if ( table->entry != NULL )
        {
            uint8_t symbol;
            if ( http_hpack_huffman_find( code, &symbol, (huffman_table_t*)table ) == WICED_SUCCESS )
            {
                if ( *symbol_buffer_size > 0 )
                {
                    uint8_t* symbol_ptr = *symbol_buffer;
                    *symbol_ptr = symbol;
                    *symbol_buffer += 1;
                    *symbol_buffer_size -= 1;
                    if ( bit > 0 )
                    {
                        bit--;
                    }
                    else
                    {
                        byte++;
                        *size = *size - 1;
                        bit = 7;
                    }
                    if ( *size == 0 )
                    {
                        /* Ok we are done, no more bytes */
                        *eos  = 1;
                    }
                    *bit_ptr = bit;
                    *stream = byte;
                    return WICED_SUCCESS;
                }
                else
                {
                    return WICED_OUT_OF_HEAP_SPACE;
                }
            }
        }
        if ( bit > 0 )
        {
            bit--;
        }
        else
        {
            byte++;
            *size = *size - 1;
            bit = 7;
        }
        if ( *size )
        {
            code = code << 1;
            if ( *byte & ( 1 << bit ) )
            {
                code++;
            }
            bit_count++;
        }
        else
        {
            if ( ( code = 0x1 ) || ( code = 0x3 ) || ( code = 0x7 )|| ( code = 0xF )|| ( code = 0x1F )|| ( code = 0x3F )|| ( code = 0x4F ) )
            {
                *eos = 1;
                *stream = byte;
                return WICED_SUCCESS;
            }
        }
    }
    return WICED_ERROR;
}

wiced_result_t http_hpack_put_integer( uint32_t value, uint8_t prefix, uint8_t** buffer, uint32_t* size )
{
    uint32_t final_size = *size;
    uint8_t* byte_ptr = *buffer;

    if ( final_size == 0 )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        uint8_t prefix_max_value = (uint8_t)( ( 1 << prefix ) - 1 );
        if ( prefix_max_value > value )
        {
            (*byte_ptr) &= (uint8_t)( 0xFF << prefix );
            *(byte_ptr++) |= (uint8_t)value;
            final_size--;
        }
        else
        {
            (*byte_ptr++) |= (uint8_t)( prefix_max_value );
            value -= prefix_max_value;
            final_size--;
            while ( ( final_size != 0 ) && ( value >= 128 ) )
            {
                prefix_max_value = (uint8_t)( ( value % 128 ) + 128 );
                *(byte_ptr++) = prefix_max_value;
                value = value / 128;
                final_size--;
            }
            if ( final_size == 0 )
            {
                return WICED_OUT_OF_HEAP_SPACE;
            }
            else
            {
                (*byte_ptr++) = (uint8_t)value;
                final_size--;
            }
        }
    }
    *buffer = byte_ptr;
    *size   = final_size;
    return WICED_SUCCESS;
}

wiced_result_t http_hpack_get_integer( uint32_t* value, uint8_t prefix, uint8_t** buffer, uint32_t* size )
{
    uint32_t value32;
    uint32_t final_size = *size;
    uint8_t* byte_ptr = (*buffer);

    if ( final_size == 0 )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        uint8_t prefix_max_value = (uint8_t)( ( 1 << prefix ) - 1 );
        value32 = ( *byte_ptr ) & prefix_max_value;
        HTTP_DEBUG(( "[%s()] : [%d] : *byte_ptr = [%d], prefix_max_value = [0x%X], value32 = [%lu]\n", __FUNCTION__, __LINE__,  *byte_ptr, prefix_max_value, value32 ));
        byte_ptr++;
        final_size--;
        if ( value32 < prefix_max_value )
        {
            *value = value32;
        }
        else
        {
            uint8_t M = 0;
            while ( final_size != 0 )
            {
                uint8_t val8 = *byte_ptr++;
                if ( val8 >= 128 )
                {
                    final_size--;
                    value32 += (uint32_t)( ( val8 & 127 ) * ( 1 << M ) );
                    M = (uint8_t)( M + 7 );
                }
                else
                {
                    final_size--;
                    value32 += (uint32_t)( ( val8 & 127 ) * ( 1 << M ) );
                    *buffer = byte_ptr;
                    *value = value32;
                    *size = final_size;
                    return WICED_SUCCESS;
                }
            }
            return WICED_OUT_OF_HEAP_SPACE;
        }
    }
    *buffer = byte_ptr;
    *value = value32;
    *size = final_size;
    return WICED_SUCCESS;
}

static wiced_result_t http_hpack_get_indexed_string( uint32_t index, uint8_t is_name, uint8_t** string_buffer, uint32_t* string_buffer_size )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( *string_buffer_size == 0 )
    {
        result = WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        const char* str;
        uint32_t str_length;
        HTTP_DEBUG(( "[%s()] : [%d] : Index = [%d]\n", __FUNCTION__, __LINE__, (int)index ));
        /* WAR : To avoid crash when header refers to the index of dynamic entries */
        if ( index <= HPACK_NUM_STATIC_TABLE_ENTRIES )
        {
            str = is_name ? static_index_table[ index ].name : static_index_table[ index ].value;
        }
        else
        {
            HTTP_DEBUG(( "[%s()] : [%d] : Need to handle dynamic entries\n", __FUNCTION__, __LINE__ ));
            str = "dynamic_entries";
        }
        HTTP_DEBUG(( "[%s()] : [%d] : Str = [%s]\n", __FUNCTION__, __LINE__, str ));
        str_length = strlen( str );
        if ( str_length <= *string_buffer_size )
        {
            memcpy( *string_buffer, str, str_length );
            *string_buffer += str_length;
            *string_buffer_size -= str_length;
        }
    }
    return result;
}

static wiced_result_t http_hpack_get_string( uint8_t** string_buffer, uint32_t* string_buffer_size, uint8_t** buffer, uint32_t* size )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( ( *size == 0 ) || ( *string_buffer_size == 0 ) )
    {
        result = WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        uint8_t     huffman_encoded = (*(*buffer)) & 0x80;
        uint32_t    string_length;

        if ( ( result = http_hpack_get_integer( &string_length, 7, buffer, size ) ) == WICED_SUCCESS )
        {
            if ( string_length > *string_buffer_size )
            {
                result = WICED_OUT_OF_HEAP_SPACE;
            }
            else
            {
                if ( huffman_encoded )
                {
                    uint8_t  bit    = 7;
                    uint8_t  eos    = 0;
                    uint32_t huffman_length = string_length;
                    while ( ( result = http_hpack_huffman_get_symbol( string_buffer, string_buffer_size, buffer,  &huffman_length, &bit, &eos ) ) == WICED_SUCCESS )
                    {
                        if ( eos == 1)
                        {
                            *size -= string_length;
                            break;
                        }
                    }
                }
                else
                {
                    memcpy( *string_buffer, *buffer, string_length );
                    *string_buffer      += string_length;
                    *buffer             += string_length;
                    *size               -= string_length;
                    *string_buffer_size -= string_length;
                }
            }
        }
    }
    return result;
}

static wiced_result_t http_hpack_put_string( uint8_t* string, uint32_t string_length, uint8_t** buffer, uint32_t* size, uint8_t huffman_encode )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( ( *size == 0 ) || ( string_length == 0 ) )
    {
        result = WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        (*buffer)[0] = huffman_encode ? 0x80 : 0x00;
        if ( ( result = http_hpack_put_integer( string_length, 7, buffer, size ) ) == WICED_SUCCESS )
        {
            if ( huffman_encode )
            {
                /* not supported for now */
                (void)huffman_tables;
                return WICED_ERROR;
            }
            else
            {
                if ( string_length > *size )
                {
                     result = WICED_OUT_OF_HEAP_SPACE;
                }
                else
                {
                    memcpy( *buffer, string, string_length );
                    *buffer             += string_length;
                    *size               -= string_length;
                }
            }
        }
    }
    return WICED_SUCCESS;
}


static wiced_result_t http_hpack_decode_index( uint8_t* type, uint32_t* index, uint8_t** buffer, uint32_t* size )
{
    if ( *size == 0 )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        uint8_t* u8_ptr = *buffer;
        uint8_t  index_opcode = *u8_ptr;

        if ( index_opcode & 0x80 )
        {
            /* Indexed Header Field representation */
            *type = INDEX_TABLE_TYPE_INDEXED;
            return http_hpack_get_integer( index, 7, buffer, size );
        }
        else if ( ( index_opcode & 0xC0 ) == 0x40 )
        {
            /* Literal Header Field representation  with incremental indexing */
            *type = INDEX_TABLE_TYPE_LITERAL_WITH_INCREMENT_INDEXING;
            return http_hpack_get_integer( index, 6, buffer, size );
        }
        else if ( ( index_opcode & 0xF0 ) == 0x00 )
        {
            /* Literal Header Field representation  without incremental indexing */
            *type = INDEX_TABLE_TYPE_LITERAL_WITHOUT_INDEXING;
            return http_hpack_get_integer( index, 4, buffer, size );
        }
        else if ( ( index_opcode & 0xF0 ) == 0x10 )
        {
            /* Literal Header Field representation  NEVER indexed */
            *type = INDEX_TABLE_TYPE_LITERAL_NEVER_INDEXED;
            return http_hpack_get_integer( index, 4, buffer, size );
        }
        else if ( ( index_opcode & 0xE0 ) == 0x20 )
        {
            /* Literal Header Field representation  NEVER indexed */
            *type = INDEX_TABLE_TYPE_DYNAMIC_TABLE_SIZE;
            return http_hpack_get_integer( index, 5, buffer, size );
        }
        else
        {
            return WICED_ERROR;
        }
    }
}


static wiced_result_t http_hpack_encode_index( uint8_t type, uint32_t index, uint8_t** buffer, uint32_t* size )
{
    if ( *size == 0 )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        uint8_t* u8_ptr = *buffer;

        switch ( type )
        {
            case INDEX_TABLE_TYPE_INDEXED:
                *u8_ptr = 0x80;
                return http_hpack_put_integer( index, 7, buffer, size );
            break;
            case INDEX_TABLE_TYPE_LITERAL_WITH_INCREMENT_INDEXING:
                *u8_ptr = 0x40;
                return http_hpack_put_integer( index, 6, buffer, size );
            break;
            case INDEX_TABLE_TYPE_LITERAL_WITHOUT_INDEXING:
                *u8_ptr = 0x00;
                return http_hpack_put_integer( index, 4, buffer, size );
            break;
            case INDEX_TABLE_TYPE_LITERAL_NEVER_INDEXED:
                *u8_ptr = 0x10;
                return http_hpack_put_integer( index, 4, buffer, size );
            break;
            case INDEX_TABLE_TYPE_DYNAMIC_TABLE_SIZE:
                *u8_ptr = 0x20;
                return http_hpack_put_integer( index, 5, buffer, size );
            break;
            default:
                return WICED_ERROR;
        }
    }
}

static wiced_result_t http_hpack_decode_header( http_header_info_t* header, uint8_t** header_buffer, uint32_t* header_buffer_size, uint8_t** buffer, uint32_t* size )
{
    if ( ( *size == 0 ) || ( *header_buffer_size == 0 ) )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        uint8_t         type;
        uint32_t        index;
        wiced_result_t  result = WICED_SUCCESS;

        if ( ( result = http_hpack_decode_index( &type, &index, buffer, size ) ) == WICED_SUCCESS )
        {
            switch ( type )
            {
                case INDEX_TABLE_TYPE_INDEXED:
                    /* All indexed, just read it */
                    /* For now assume only static indexing */
                    {
                       header->name = *header_buffer;
                       if ( ( result = http_hpack_get_indexed_string( index, 1, header_buffer, header_buffer_size ) ) == WICED_SUCCESS )
                       {
                           header->name_length = (uint32_t)(*header_buffer) - (uint32_t)header->name;
                           header->value = *header_buffer;
                           result = http_hpack_get_indexed_string( index, 0, header_buffer, header_buffer_size );
                           header->value_length = (uint32_t)(*header_buffer) - (uint32_t)header->value;;
                       }
                    }
                    break;
                case INDEX_TABLE_TYPE_LITERAL_WITH_INCREMENT_INDEXING:
                case INDEX_TABLE_TYPE_LITERAL_NEVER_INDEXED:
                case INDEX_TABLE_TYPE_LITERAL_WITHOUT_INDEXING:
                    {
                        header->name = *header_buffer;
                        if ( index != 0 )
                        {
                            result = http_hpack_get_indexed_string( index, 1, header_buffer, header_buffer_size );
                        }
                        else
                        {
                            result = http_hpack_get_string( header_buffer, header_buffer_size, buffer, size );
                        }
                        if ( result == WICED_SUCCESS )
                        {
                            /* Set value pointer */
                            header->name_length = (uint32_t)(*header_buffer) - (uint32_t)header->name;
                            header->value = *header_buffer;
                            result = http_hpack_get_string( header_buffer, header_buffer_size, buffer, size );
                            header->value_length = (uint32_t)(*header_buffer) - (uint32_t)header->value;
                        }
                    }

                    if (INDEX_TABLE_TYPE_LITERAL_WITH_INCREMENT_INDEXING)
                    {
                        static int dyn_index = HPACK_NUM_STATIC_TABLE_ENTRIES;

                        UNUSED_VARIABLE(dyn_index);
                        /* Adding entry to the dynamic table list */
                        HTTP_DEBUG(( "[%s()] : [%d] : Need to handle dynamic literal\n", __FUNCTION__, __LINE__ ));
                        HTTP_DEBUG(( "[%s()] : [%d] : Index [%d] : Name = [%.*s], Value = [%.*s]\n", __FUNCTION__, __LINE__, ++dyn_index, (int)header->name_length, header->name, (int)header->value_length, header->value ));
                    }
                    break;
                case INDEX_TABLE_TYPE_DYNAMIC_TABLE_SIZE:
                    {
                        header->name_length = 0;
                        header->value_length = 0;
                    }
                    break;
                default:
                    break;
            }
        }
        return result;
    }
}


static wiced_result_t http_hpack_encode_header( http_header_info_t* header, uint8_t** buffer, uint32_t* size )
{
    if ( *size == 0 )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        uint8_t         type           = INDEX_TABLE_TYPE_LITERAL_WITHOUT_INDEXING;
        uint8_t         huffman_encode = 0;
        uint32_t        index          = 0;
        wiced_result_t  result         = WICED_SUCCESS;

        if ( ( result = http_hpack_encode_index( type, index, buffer, size ) ) == WICED_SUCCESS )
        {
            switch ( type )
            {
                case INDEX_TABLE_TYPE_INDEXED:
                    {
                       result = WICED_ERROR;
                    }
                    break;
                case INDEX_TABLE_TYPE_LITERAL_WITH_INCREMENT_INDEXING:
                    {
                        result = WICED_ERROR;
                    }
                    break;
                case INDEX_TABLE_TYPE_LITERAL_NEVER_INDEXED:
                case INDEX_TABLE_TYPE_LITERAL_WITHOUT_INDEXING:
                    {
                        if ( index != 0 )
                        {
                            //result = http_hpack_get_indexed_string( index, 1, header_buffer, header_buffer_size );
                            result = WICED_ERROR;
                        }
                        else
                        {
                            result = http_hpack_put_string( header->name, header->name_length, buffer, size, huffman_encode );
                        }
                        if ( result == WICED_SUCCESS )
                        {
                            /* Set value pointer */
                            result = http_hpack_put_string( header->value, header->value_length, buffer, size, huffman_encode );
                        }
                    }
                    break;
                case INDEX_TABLE_TYPE_DYNAMIC_TABLE_SIZE:
                    {
                        result = WICED_ERROR;
                    }
                    break;
                default:
                    break;
            }
        }
        return result;
    }
}

wiced_result_t http_hpack_encode_headers( http_header_info_t* head, uint8_t** buffer, uint32_t* size )
{
    wiced_result_t       result = WICED_SUCCESS;
    http_header_info_t*  header = head;
    while( ( result == WICED_SUCCESS ) && header )
    {
        result = http_hpack_encode_header ( header, buffer, size );
        header = header->next;
    }
    return result;
}

wiced_result_t http_hpack_decode_headers( http_header_info_t* head, uint8_t** header_buffer, uint32_t* header_buffer_size, uint8_t** buffer, uint32_t* size )
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t        align_overhead = 0;

    head->next  = NULL;
    while( ( result == WICED_SUCCESS ) && (*size) )
    {
        http_header_info_t*  header;

        /* Align header buffer to word alignment to map it with http_header_info_t */
        align_overhead = (uint8_t)((((uint32_t)*header_buffer % 4) == 0) ? 0 : (4 - ((uint8_t)((uint32_t)*header_buffer % 4))));
        *header_buffer += align_overhead;
        *header_buffer_size -= align_overhead;

        header = (http_header_info_t*)(*header_buffer);

        if ( (*header_buffer_size) < sizeof(http_header_info_t) )
        {
            result = WICED_OUT_OF_HEAP_SPACE;
            break;
        }
        header->next = NULL;
        *header_buffer = (uint8_t*)header + sizeof( http_header_info_t );
        *header_buffer_size -= sizeof( http_header_info_t );
        result = http_hpack_decode_header ( header, header_buffer, header_buffer_size, buffer, size );
        if ( result == WICED_SUCCESS )
        {
            head->next = header;
            head = header;
        }
    }
    return result;
}
