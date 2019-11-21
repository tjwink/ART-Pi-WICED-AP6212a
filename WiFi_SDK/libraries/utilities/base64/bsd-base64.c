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
/*    $OpenBSD: base64.c,v 1.3 1997/11/08 20:46:55 deraadt Exp $    */

/*
 * Copyright (c) 1996 by Internet Software Consortium.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND INTERNET SOFTWARE CONSORTIUM DISCLAIMS
 * ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL INTERNET SOFTWARE
 * CONSORTIUM BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
 * SOFTWARE.
 */

/*
 * Portions Copyright (c) 1995 by International Business Machines, Inc.
 *
 * International Business Machines, Inc. (hereinafter called IBM) grants
 * permission under its copyrights to use, copy, modify, and distribute this
 * Software with or without fee, provided that the above copyright notice and
 * all paragraphs of this notice appear in all copies, and that the name of IBM
 * not be used in connection with the marketing of any product incorporating
 * the Software or modifications thereof, without specific, written prior
 * permission.
 *
 * To the extent it has a right to do so, IBM grants an immunity from suit
 * under its patents, if any, for the use, sale or manufacture of products to
 * the extent that such products are used for performing Domain Name System
 * dynamic updates in TCP/IP networks by means of the Software.  No immunity is
 * granted for any product per se or for any other function of any product.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", AND IBM DISCLAIMS ALL WARRANTIES,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE.  IN NO EVENT SHALL IBM BE LIABLE FOR ANY SPECIAL,
 * DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER ARISING
 * OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE, EVEN
 * IF IBM IS APPRISED OF THE POSSIBILITY OF SUCH DAMAGES.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "base64.h"
#include "wwd_assert.h"


#define BASE64_PAD_BYTE( options )      ( (unsigned char)( options & 0xff ) )
#define BASE64_62_VALUE_BYTE( options ) ( (unsigned char)( options >>  16 ) )
#define BASE64_63_VALUE_BYTE( options ) ( (unsigned char)( options >>   8 ) )

/* Array of base64 characters except for last two which depend on type of conversion */
static const char base64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";

/* (From RFC1521 and draft-ietf-dnssec-secext-03.txt)
   The following encoding technique is taken from RFC 1521 by Borenstein
   and Freed.  It is reproduced here in a slightly edited form for
   convenience.

   A 65-character subset of US-ASCII is used, enabling 6 bits to be
   represented per printable character. (The extra 65th character, "=",
   is used to signify a special processing function.)

   The encoding process represents 24-bit groups of input bits as output
   strings of 4 encoded characters. Proceeding from left to right, a
   24-bit input group is formed by concatenating 3 8-bit input groups.
   These 24 bits are then treated as 4 concatenated 6-bit groups, each
   of which is translated into a single digit in the base64 alphabet.

   Each 6-bit group is used as an index into an array of 64 printable
   characters. The character referenced by the index is placed in the
   output string.

                         Table 1: The Base64 Alphabet

      Value Encoding  Value Encoding  Value Encoding  Value Encoding
          0 A            17 R            34 i            51 z
          1 B            18 S            35 j            52 0
          2 C            19 T            36 k            53 1
          3 D            20 U            37 l            54 2
          4 E            21 V            38 m            55 3
          5 F            22 W            39 n            56 4
          6 G            23 X            40 o            57 5
          7 H            24 Y            41 p            58 6
          8 I            25 Z            42 q            59 7
          9 J            26 a            43 r            60 8
         10 K            27 b            44 s            61 9
         11 L            28 c            45 t            62 +
         12 M            29 d            46 u            63 /
         13 N            30 e            47 v
         14 O            31 f            48 w         (pad) =
         15 P            32 g            49 x
         16 Q            33 h            50 y

   Special processing is performed if fewer than 24 bits are available
   at the end of the data being encoded.  A full encoding quantum is
   always completed at the end of a quantity.  When fewer than 24 input
   bits are available in an input group, zero bits are added (on the
   right) to form an integral number of 6-bit groups.  Padding at the
   end of the data is performed using the '=' character.

   Since all base64 input is an integral number of octets, only the
         -------------------------------------------------
   following cases can arise:

       (1) the final quantum of encoding input is an integral
           multiple of 24 bits; here, the final unit of encoded
       output will be an integral multiple of 4 characters
       with no "=" padding,
       (2) the final quantum of encoding input is exactly 8 bits;
           here, the final unit of encoded output will be two
       characters followed by two "=" padding characters, or
       (3) the final quantum of encoding input is exactly 16 bits;
           here, the final unit of encoded output will be three
       characters followed by one "=" padding character.
   */

int is_base64_space( int c )
{
    return ( ( c >= 0x09 && c <= 0x0D ) || ( c == 0x20 ) );
}

static inline unsigned char base64_enc_val( unsigned char val, base64_options_t options )
{
    if ( val < 62 )
    {
        return base64[ val ];
    }
    else if ( val == 62)
    {
        return BASE64_62_VALUE_BYTE( options );
    }

    return BASE64_63_VALUE_BYTE( options );
}

int base64_encode( unsigned char const* src, int32_t src_length, unsigned char* target, uint32_t target_size, base64_options_t options )
{
    size_t        datalength = 0;
    unsigned char input[3];
    unsigned char output[4];
    int           i;

    if ( src_length < 0 )
    {
        src_length = strlen( (char const*) src );
    }

    while ( 2 < src_length )
    {
        input[ 0 ]  = *src++;
        input[ 1 ]  = *src++;
        input[ 2 ]  = *src++;
        src_length -= 3;

        output[ 0 ] = input[ 0 ] >> 2;
        output[ 1 ] = ( ( input[ 0 ] & 0x03 ) << 4 ) + ( input[ 1 ] >> 4 );
        output[ 2 ] = ( ( input[ 1 ] & 0x0f ) << 2 ) + ( input[ 2 ] >> 6 );
        output[ 3 ] = input[ 2 ] & 0x3f;

        wiced_assert("", (output[0] < 64) && (output[1] < 64) && (output[2] < 64) && (output[3] < 64));

        if ( datalength + 4 > target_size )
        {
            return ( -1 );
        }

        target[ datalength++ ] = base64_enc_val( output[ 0 ], options );
        target[ datalength++ ] = base64_enc_val( output[ 1 ], options );
        target[ datalength++ ] = base64_enc_val( output[ 2 ], options );
        target[ datalength++ ] = base64_enc_val( output[ 3 ], options );
    }

    /* Now we worry about padding. */
    if ( src_length != 0 )
    {
        /* Get what's left. */
        input[ 0 ] = input[ 1 ] = input[ 2 ] = '\0';
        for ( i = 0; i < src_length; i++ )
        {
            input[ i ] = *src++;
        }

        output[ 0 ] = input[ 0 ] >> 2;
        output[ 1 ] = ( ( input[ 0 ] & 0x03 ) << 4 ) + ( input[ 1 ] >> 4 );
        output[ 2 ] = ( ( input[ 1 ] & 0x0f ) << 2 ) + ( input[ 2 ] >> 6 );

        wiced_assert("", ( output[ 0 ] < 64 ) && ( output[ 1 ] < 64 ) && ( output[ 2 ] < 64 ));

        if ( datalength + 2 > target_size )
        {
            return ( -1 );
        }
        target[ datalength++ ] = base64_enc_val( output[ 0 ], options );
        target[ datalength++ ] = base64_enc_val( output[ 1 ], options );

        if  ( datalength + 1 <= target_size )
        {
            if ( src_length != 1 )
            {
                target[ datalength++ ] = base64[ output[ 2 ] ];
            }
            else if ( BASE64_PAD_BYTE( options ) != 0 )
            {
                target[ datalength++ ] = BASE64_PAD_BYTE( options );
            }
        }
        if (( BASE64_PAD_BYTE( options ) != 0 ) &&  ( datalength + 1 <= target_size ) )
        {
            target[ datalength++ ] = BASE64_PAD_BYTE( options );
        }
    }

    if ( datalength >= target_size )
    {
        return ( -1 );
    }

    target[ datalength ] = '\0'; /* Returned value doesn't count \0. */

    return datalength;
}

/* skips all whitespace anywhere.
   converts characters, four at a time, starting at (or after)
   src from base - 64 numbers into three 8 bit bytes in the target area.
   it returns the number of data bytes stored at the target, or -1 on error.
 */

/* src_len = -1  : null terminated string */
int base64_decode( unsigned char const* src, int32_t src_length, unsigned char* target, uint32_t target_size, base64_options_t options )
{
    int         tarindex;
    int         state;
    char        ch;
    char*       pos;
    unsigned char const* orig_src = src;

    state    = 0;
    tarindex = 0;

    while ( ( ( ch = *src++ ) != '\0' ) && ( ( src_length < 0 ) || ( src - orig_src <= src_length ) ) )
    {
        /* Skip whitespace anywhere. */
        if ( is_base64_space( ch ) != 0 )
        {
            continue;
        }

        if ( ch == BASE64_PAD_BYTE( options ) )
        {
            break;
        }

        pos = strchr( base64, ch );
        if ( pos == 0 ) /* A non-base64 character. */
        {
            if ( ch == BASE64_62_VALUE_BYTE( options ) )
            {
                pos = (char*) base64 + 62;
            }
            else if ( ch == BASE64_63_VALUE_BYTE( options ) )
            {
                pos = (char*) base64 + 63;
            }
            else
            {
                return ( -1 );
            }
        }

        switch ( state )
        {
            case 0:
                if ( target != NULL  )
                {
                    if ( tarindex >= target_size )
                    {
                        return ( -1 );
                    }
                    target[ tarindex ] = ( pos - base64 ) << 2;
                }
                state = 1;
                break;

            case 1:
                if ( target != NULL  )
                {
                    if ( tarindex + 1 >= target_size )
                    {
                        return ( -1 );
                    }
                    target[ tarindex ]    |= ( pos - base64 ) >> 4;
                    target[ tarindex + 1 ] = ( ( pos - base64 ) & 0x0f ) << 4;
                }
                tarindex++;
                state = 2;
                break;

            case 2:
                if ( target != NULL )
                {
                    if ( tarindex + 1 >= target_size )
                    {
                        return ( -1 );
                    }
                    target[ tarindex ]    |= ( pos - base64 ) >> 2;
                    target[ tarindex + 1 ] = ( ( pos - base64 ) & 0x03 ) << 6;
                }
                tarindex++;
                state = 3;
                break;

            case 3:
                if ( target != NULL )
                {
                    if ( tarindex >= target_size )
                    {
                        return ( -1 );
                    }
                    target[ tarindex ] |= ( pos - base64 );
                }
                tarindex++;
                state = 0;
                break;
        }
    }

    /* We are done decoding Base-64 chars.  Let's see if we ended on a byte boundary, and/or with erroneous trailing characters. */

    if ( BASE64_PAD_BYTE( options ) != 0 )
    {
        if ( ( ch == BASE64_PAD_BYTE( options ) ) && ( ( src_length < 0 ) || ( src - orig_src <= src_length ) ) )
        {
            /* We got a pad char. Skip it, get next */
            ch = *src++;

            switch ( state )
            {
                case 0: /* Invalid = in first position */
                case 1: /* Invalid = in second position */
                    return ( -1 );

                case 2: /* Valid, means one byte of info */
                    /* Skip any number of spaces. */
                    for ( ; ( ( ch != '\0' ) && ( ( src_length < 0 ) || ( src - orig_src <= src_length ) ) ); ch = *src++ )
                    {
                        if ( is_base64_space( ch ) == 0 )
                        {
                            break;
                        }
                    }
                    /* Make sure there is another trailing = sign. */
                    if ( ( ch != BASE64_PAD_BYTE( options ) ) || ( ( src_length > 0 ) && ( src - orig_src > src_length ) ) )
                    {
                        return ( -1 );
                    }

                    /* Skip the = */
                    ch = *src++;

                    /* Fall through to "single trailing =" case. */
                    /* no break: FALLTHROUGH */

                case 3: /* Valid, means two bytes of info */
                    /*
                     * We know this char is an =.  Is there anything but
                     * whitespace after it?
                     */
                    for ( ; ( ( ch != '\0' ) && ( ( src_length < 0 ) || ( src - orig_src <= src_length ) ) ); ch = *src++ )
                    {
                        if ( is_base64_space( ch ) == 0 )
                        {
                            return ( -1 );
                        }
                    }

                    /*
                     * Now make sure for cases 2 and 3 that the "extra"
                     * bits that slopped past the last full byte were
                     * zeros.  If we don't check them, they become a
                     * subliminal channel.
                     */
                    if ( ( target != NULL ) && ( target[ tarindex ] != 0 ) )
                    {
                        return ( -1 );
                    }
                    break;
            }
        }
        else
        {
            /*
             * We ended by seeing the end of the string.  Make sure we
             * have no partial bytes lying around.
             */
            if ( state != 0 )
            {
                return ( -1 );
            }
        }
    }

    if ( tarindex >= target_size )
    {
        return ( -1 );
    }
    if ( target != NULL )
    {
        target[ tarindex ] = '\0'; /* Returned value doesn't count \0. */
    }


    return (tarindex);
}


typedef struct
{
    const char*             plain_text;
    const char*             base64_coded;
    int                     num_padding_bytes;
    int                     length;
} base64_test_vector_t;


static const base64_test_vector_t base64_test_vectors[] =
{
        { "",       "",         0, -1 },
        { "f",      "Zg==",     2, -1 },
        { "fo",     "Zm8=",     1, -1 },
        { "foo",    "Zm9v",     0, -1 },
        { "foob",   "Zm9vYg==", 2, -1 },
        { "fooba",  "Zm9vYmE=", 1, -1 },
        { "foobar", "Zm9vYmFy", 0, -1 },
        { "The quick brown fox jumps over the lazy dog. Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.",
          "VGhlIHF1aWNrIGJyb3duIGZveCBqdW1wcyBvdmVyIHRoZSBsYXp5IGRvZy4gTG9yZW0gaXBzdW0gZG9sb3Igc2l0IGFtZXQsIGNvbnNlY3RldHVyIGFkaXBpc2ljaW5nIGVsaXQsIHNlZCBkbyBlaXVzbW9kIHRlbXBvciBpbmNpZGlkdW50IHV0IGxhYm9yZSBldCBkb2xvcmUgbWFnbmEgYWxpcXVhLiBVdCBlbmltIGFkIG1pbmltIHZlbmlhbSwgcXVpcyBub3N0cnVkIGV4ZXJjaXRhdGlvbiB1bGxhbWNvIGxhYm9yaXMgbmlzaSB1dCBhbGlxdWlwIGV4IGVhIGNvbW1vZG8gY29uc2VxdWF0LiBEdWlzIGF1dGUgaXJ1cmUgZG9sb3IgaW4gcmVwcmVoZW5kZXJpdCBpbiB2b2x1cHRhdGUgdmVsaXQgZXNzZSBjaWxsdW0gZG9sb3JlIGV1IGZ1Z2lhdCBudWxsYSBwYXJpYXR1ci4gRXhjZXB0ZXVyIHNpbnQgb2NjYWVjYXQgY3VwaWRhdGF0IG5vbiBwcm9pZGVudCwgc3VudCBpbiBjdWxwYSBxdWkgb2ZmaWNpYSBkZXNlcnVudCBtb2xsaXQgYW5pbSBpZCBlc3QgbGFib3J1bS4=",
          1, -1
        },
        { "\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f"
          "\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1a\x1b\x1c\x1d\x1e\x1f"
          "\x20\x21\x22\x23\x24\x25\x26\x27\x28\x29\x2a\x2b\x2c\x2d\x2e\x2f"
          "\x30\x31\x32\x33\x34\x35\x36\x37\x38\x39\x3a\x3b\x3c\x3d\x3e\x3f"
          "\x40\x41\x42\x43\x44\x45\x46\x47\x48\x49\x4a\x4b\x4c\x4d\x4e\x4f"
          "\x50\x51\x52\x53\x54\x55\x56\x57\x58\x59\x5a\x5b\x5c\x5d\x5e\x5f"
          "\x60\x61\x62\x63\x64\x65\x66\x67\x68\x69\x6a\x6b\x6c\x6d\x6e\x6f"
          "\x70\x71\x72\x73\x74\x75\x76\x77\x78\x79\x7a\x7b\x7c\x7d\x7e\x7f"
          "\x80\x81\x82\x83\x84\x85\x86\x87\x88\x89\x8a\x8b\x8c\x8d\x8e\x8f"
          "\x90\x91\x92\x93\x94\x95\x96\x97\x98\x99\x9a\x9b\x9c\x9d\x9e\x9f"
          "\xa0\xa1\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xab\xac\xad\xae\xaf"
          "\xb0\xb1\xb2\xb3\xb4\xb5\xb6\xb7\xb8\xb9\xba\xbb\xbc\xbd\xbe\xbf"
          "\xc0\xc1\xc2\xc3\xc4\xc5\xc6\xc7\xc8\xc9\xca\xcb\xcc\xcd\xce\xcf"
          "\xd0\xd1\xd2\xd3\xd4\xd5\xd6\xd7\xd8\xd9\xda\xdb\xdc\xdd\xde\xdf"
          "\xe0\xe1\xe2\xe3\xe4\xe5\xe6\xe7\xe8\xe9\xea\xeb\xec\xed\xee\xef"
          "\xf0\xf1\xf2\xf3\xf4\xf5\xf6\xf7\xf8\xf9\xfa\xfb\xfc\xfd\xfe\xff",
          "AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gISIjJCUmJygpKissLS4vMDEyMzQ1Njc4"
          "OTo7PD0+P0BBQkNERUZHSElKS0xNTk9QUVJTVFVWV1hZWltcXV5fYGFiY2RlZmdoaWprbG1ub3Bx"
          "cnN0dXZ3eHl6e3x9fn+AgYKDhIWGh4iJiouMjY6PkJGSk5SVlpeYmZqbnJ2en6ChoqOkpaanqKmq"
          "q6ytrq+wsbKztLW2t7i5uru8vb6/wMHCw8TFxsfIycrLzM3Oz9DR0tPU1dbX2Nna29zd3t/g4eLj"
          "5OXm5+jp6uvs7e7v8PHy8/T19vf4+fr7/P3+/w==",
          2,
          256
        },
        { "\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f"
          "\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1a\x1b\x1c\x1d\x1e\x1f"
          "\x20\x21\x22\x23\x24\x25\x26\x27\x28\x29\x2a\x2b\x2c\x2d\x2e\x2f"
          "\x30\x31\x32\x33\x34\x35\x36\x37\x38\x39\x3a\x3b\x3c\x3d\x3e\x3f"
          "\x40\x41\x42\x43\x44\x45\x46\x47\x48\x49\x4a\x4b\x4c\x4d\x4e\x4f"
          "\x50\x51\x52\x53\x54\x55\x56\x57\x58\x59\x5a\x5b\x5c\x5d\x5e\x5f"
          "\x60\x61\x62\x63\x64\x65\x66\x67\x68\x69\x6a\x6b\x6c\x6d\x6e\x6f"
          "\x70\x71\x72\x73\x74\x75\x76\x77\x78\x79\x7a\x7b\x7c\x7d\x7e\x7f"
          "\x80\x81\x82\x83\x84\x85\x86\x87\x88\x89\x8a\x8b\x8c\x8d\x8e\x8f"
          "\x90\x91\x92\x93\x94\x95\x96\x97\x98\x99\x9a\x9b\x9c\x9d\x9e\x9f"
          "\xa0\xa1\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xab\xac\xad\xae\xaf"
          "\xb0\xb1\xb2\xb3\xb4\xb5\xb6\xb7\xb8\xb9\xba\xbb\xbc\xbd\xbe\xbf"
          "\xc0\xc1\xc2\xc3\xc4\xc5\xc6\xc7\xc8\xc9\xca\xcb\xcc\xcd\xce\xcf"
          "\xd0\xd1\xd2\xd3\xd4\xd5\xd6\xd7\xd8\xd9\xda\xdb\xdc\xdd\xde\xdf"
          "\xe0\xe1\xe2\xe3\xe4\xe5\xe6\xe7\xe8\xe9\xea\xeb\xec\xed\xee\xef"
          "\xf0\xf1\xf2\xf3\xf4\xf5\xf6\xf7\xf8\xf9\xfa\xfb\xfc\xfd\xfe\xff"
          "\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f"
          "\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1a\x1b\x1c\x1d\x1e\x1f"
          "\x20\x21\x22\x23\x24\x25\x26\x27\x28\x29\x2a\x2b\x2c\x2d\x2e\x2f"
          "\x30\x31\x32\x33\x34\x35\x36\x37\x38\x39\x3a\x3b\x3c\x3d\x3e\x3f"
          "\x40\x41\x42\x43\x44\x45\x46\x47\x48\x49\x4a\x4b\x4c\x4d\x4e\x4f"
          "\x50\x51\x52\x53\x54\x55\x56\x57\x58\x59\x5a\x5b\x5c\x5d\x5e\x5f"
          "\x60\x61\x62\x63\x64\x65\x66\x67\x68\x69\x6a\x6b\x6c\x6d\x6e\x6f"
          "\x70\x71\x72\x73\x74\x75\x76\x77\x78\x79\x7a\x7b\x7c\x7d\x7e\x7f"
          "\x80\x81\x82\x83\x84\x85\x86\x87\x88\x89\x8a\x8b\x8c\x8d\x8e\x8f"
          "\x90\x91\x92\x93\x94\x95\x96\x97\x98\x99\x9a\x9b\x9c\x9d\x9e\x9f"
          "\xa0\xa1\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xab\xac\xad\xae\xaf"
          "\xb0\xb1\xb2\xb3\xb4\xb5\xb6\xb7\xb8\xb9\xba\xbb\xbc\xbd\xbe\xbf"
          "\xc0\xc1\xc2\xc3\xc4\xc5\xc6\xc7\xc8\xc9\xca\xcb\xcc\xcd\xce\xcf"
          "\xd0\xd1\xd2\xd3\xd4\xd5\xd6\xd7\xd8\xd9\xda\xdb\xdc\xdd\xde\xdf"
          "\xe0\xe1\xe2\xe3\xe4\xe5\xe6\xe7\xe8\xe9\xea\xeb\xec\xed\xee\xef"
          "\xf0\xf1\xf2\xf3\xf4\xf5\xf6\xf7\xf8\xf9\xfa\xfb\xfc\xfd\xfe\xff",
          "AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gISIjJCUmJygpKissLS4vMDEyMzQ1Njc4"
          "OTo7PD0+P0BBQkNERUZHSElKS0xNTk9QUVJTVFVWV1hZWltcXV5fYGFiY2RlZmdoaWprbG1ub3Bx"
          "cnN0dXZ3eHl6e3x9fn+AgYKDhIWGh4iJiouMjY6PkJGSk5SVlpeYmZqbnJ2en6ChoqOkpaanqKmq"
          "q6ytrq+wsbKztLW2t7i5uru8vb6/wMHCw8TFxsfIycrLzM3Oz9DR0tPU1dbX2Nna29zd3t/g4eLj"
          "5OXm5+jp6uvs7e7v8PHy8/T19vf4+fr7/P3+/wABAgMEBQYHCAkKCwwNDg8QERITFBUWFxgZGhsc"
          "HR4fICEiIyQlJicoKSorLC0uLzAxMjM0NTY3ODk6Ozw9Pj9AQUJDREVGR0hJSktMTU5PUFFSU1RV"
          "VldYWVpbXF1eX2BhYmNkZWZnaGlqa2xtbm9wcXJzdHV2d3h5ent8fX5/gIGCg4SFhoeIiYqLjI2O"
          "j5CRkpOUlZaXmJmam5ydnp+goaKjpKWmp6ipqqusra6vsLGys7S1tre4ubq7vL2+v8DBwsPExcbH"
          "yMnKy8zNzs/Q0dLT1NXW19jZ2tvc3d7f4OHi4+Tl5ufo6err7O3u7/Dx8vP09fb3+Pn6+/z9/v8=",
          1,
          512
        },
};


typedef struct
{
        base64_options_t option;
        char val62;
        char val63;
        char pad;
} test_option_t;

static const test_option_t test_options[] =
{
        { BASE64_STANDARD,                       '+', '/', '=' },
        { BASE64_NO_PADDING,                     '+', '/',  0  },
        { BASE64_URL_SAFE_CHARSET,               '-', '_',  0  },
        { BASE64_URL_SAFE_CHARSET_WITH_PADDING,  '-', '_', '=' },
        { BASE64_Y64,                            '.', '_', '-' },
        { BASE64_XML_TOKEN,                      '.', '-',  0  },
        { BASE64_XML_IDENTIFIER,                 '_', ':',  0  },
        { BASE64_PROG_IDENTIFIER1,               '_', '-',  0  },
        { BASE64_PROG_IDENTIFIER2,               '.', '_',  0  },
        { BASE64_REGEX,                          '!', '-',  0  },
};

int base64_test( void )
{
    int test_num;
    for ( test_num = 0; test_num < sizeof(base64_test_vectors)/sizeof(base64_test_vector_t); test_num++ )
    {
        int option_num;
        int output_buffer_length = strlen( base64_test_vectors[ test_num ].base64_coded ) + 1;
        char* correct_output_buffer = (char*) malloc( output_buffer_length );
        char* output_buffer = (char*) malloc( output_buffer_length );
        for ( option_num = 0; option_num < sizeof(test_options)/sizeof(test_option_t); option_num++ )
        {
            int retval;

            strcpy( correct_output_buffer, base64_test_vectors[ test_num ].base64_coded );

            char* replace_char = correct_output_buffer;
            while ( ( replace_char = strchr( replace_char, '+' ) ) != NULL )
            {
                *replace_char = test_options[ option_num ].val62;
                replace_char++;
            }
            replace_char = correct_output_buffer;
            while ( ( replace_char = strchr( replace_char, '/' ) ) != NULL )
            {
                *replace_char = test_options[ option_num ].val63;
                replace_char++;
            }
            replace_char = correct_output_buffer;
            while ( ( replace_char = strchr( replace_char, '=' ) ) != NULL )
            {
                *replace_char = test_options[ option_num ].pad;
                replace_char++;
            }

            int plain_text_length = base64_test_vectors[test_num].length;

            /* Check Normal encode */
            retval = base64_encode( (const unsigned char*)base64_test_vectors[test_num].plain_text,
                                    base64_test_vectors[test_num].length,
                                    (unsigned char*)output_buffer,
                                    output_buffer_length,
                                    test_options[ option_num ].option );
            if ( retval != strlen(correct_output_buffer) )
            {
                wiced_assert("", 0 == 1);
                free( output_buffer );
                free( correct_output_buffer );
                return -1;
            }

            if ( strcmp( output_buffer, correct_output_buffer ) != 0 )
            {
                wiced_assert("", 0 == 1);
                free( output_buffer );
                free( correct_output_buffer );
                return -2;
            }


            if ( plain_text_length == -1 )
            {
                plain_text_length = strlen(base64_test_vectors[test_num].plain_text);
            }

            /* Check Normal decode */
            retval = base64_decode( (unsigned char*)correct_output_buffer,
                                    output_buffer_length,
                                    (unsigned char*)output_buffer,
                                    output_buffer_length,
                                    test_options[ option_num ].option );
            if ( retval != plain_text_length )
            {
                wiced_assert("", 0 == 1);
                free( output_buffer );
                free( correct_output_buffer );
                return -5;
            }

            if ( ( ( base64_test_vectors[test_num].length == -1 ) && ( strcmp( output_buffer, base64_test_vectors[test_num].plain_text ) != 0 ) ) ||
                 ( memcmp( output_buffer, base64_test_vectors[test_num].plain_text, plain_text_length ) != 0 ) )
            {
                wiced_assert("", 0 == 1);
                free( output_buffer );
                free( correct_output_buffer );
                return -6;
            }

            retval = base64_decode( (unsigned char*)correct_output_buffer,
                                    -1,
                                    (unsigned char*)output_buffer,
                                    plain_text_length+1,
                                    test_options[ option_num ].option );
            if ( retval != plain_text_length )
            {
                wiced_assert("", 0 == 1);
                free( output_buffer );
                free( correct_output_buffer );
                return -7;
            }

            if ( ( ( base64_test_vectors[test_num].length == -1 ) && ( strcmp( output_buffer, base64_test_vectors[test_num].plain_text ) != 0 ) ) ||
                 ( memcmp( output_buffer, base64_test_vectors[test_num].plain_text, plain_text_length ) != 0 ) )
            {
                wiced_assert("", 0 == 1);
                free( output_buffer );
                free( correct_output_buffer );
                return -8;
            }
        }
        free(output_buffer);
        free(correct_output_buffer);
    }

    return 0;
}
