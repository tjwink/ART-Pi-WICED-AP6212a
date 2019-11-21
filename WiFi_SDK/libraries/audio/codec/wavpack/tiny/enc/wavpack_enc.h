////////////////////////////////////////////////////////////////////////////
//                           **** WAVPACK ****                            //
//                  Hybrid Lossless Wavefile Compressor                   //
//              Copyright (c) 1998 - 2007 Conifer Software.               //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// wavpack.h

#ifndef WAVPACK_H
#define WAVPACK_H

// This header file contains all the definitions required to use the
// functions in "wputils.c" to read and write WavPack files and streams.

#include <sys/types.h>

#ifdef __BORLANDC__
typedef unsigned long uint32_t;
typedef long int32_t;
#elif defined(_WIN32) && !defined(__MINGW32__)
#include <stdlib.h>
typedef unsigned __int64 uint64_t;
typedef unsigned __int32 uint32_t;
typedef __int64 int64_t;
typedef __int32 int32_t;
#else
#include <inttypes.h>
#endif

typedef unsigned char   uchar;

#if !defined(__GNUC__) || defined(WIN32)
typedef unsigned short  ushort;
typedef unsigned int    uint;
#endif

///////////////////////// WavPack Configuration ///////////////////////////////

// This external structure is used during encode to provide configuration to
// the encoding engine.

typedef struct {
    int bitrate, shaping_weight;
    int bits_per_sample, bytes_per_sample, num_channels;
    int32_t block_samples, sample_rate;
    uint32_t flags;
} WavpackEncConfig;

#define CONFIG_HYBRID_FLAG      8       // hybrid mode
#define CONFIG_JOINT_STEREO     0x10    // joint stereo
#define CONFIG_HYBRID_SHAPE     0x40    // noise shape (hybrid mode only)
#define CONFIG_FAST_FLAG        0x200   // fast mode
#define CONFIG_HIGH_FLAG        0x800   // high quality mode
#define CONFIG_VERY_HIGH_FLAG   0x1000  // very high
#define CONFIG_SHAPE_OVERRIDE   0x8000  // shaping mode specified
#define CONFIG_JOINT_OVERRIDE   0x10000 // joint-stereo mode specified
#define CONFIG_CREATE_WVC       0x80000 // create correction file
#define CONFIG_OPTIMIZE_WVC     0x100000 // maximize bybrid compression

////////////// Callbacks used for writing WavPack streams //////////

typedef int (*WavpackBlockOutput)(void *id, void *data, int32_t bcount);

//////////////////////// function prototypes and macros //////////////////////

typedef void WavpackEncContext;

#ifdef __cplusplus
extern "C" {
#endif

char *WavpackEncGetErrorMessage (WavpackEncContext *wpc);
uint32_t WavpackEncGetNumSamples (WavpackEncContext *wpc);
uint32_t WavpackEncGetSampleIndex (WavpackEncContext *wpc);
int WavpackLossyEncBlocks (WavpackEncContext *wpc);
WavpackEncContext *WavpackEncCloseFile (WavpackEncContext *wpc);
uint32_t WavpackEncGetSampleRate (WavpackEncContext *wpc);
int WavpackEncGetBitsPerSample (WavpackEncContext *wpc);
int WavpackEncGetBytesPerSample (WavpackEncContext *wpc);
int WavpackEncGetNumChannels (WavpackEncContext *wpc);
uint32_t WavpackEncGetFileSize (WavpackEncContext *wpc);
WavpackEncContext *WavpackEncOpenFileOutput (WavpackBlockOutput blockout, void *wv_id, void *wvc_id);
int WavpackEncSetConfiguration (WavpackEncContext *wpc, WavpackEncConfig *config, uint32_t total_samples);
int WavpackEncPackInit (WavpackEncContext *wpc);
int WavpackEncPackSamples (WavpackEncContext *wpc, int32_t *sample_buffer, uint32_t sample_count);
int WavpackEncFlushSamples (WavpackEncContext *wpc);
void WavpackEncUpdateNumSamples (WavpackEncContext *wpc, void *first_block);
void WavpackEncNativeToLittleEndian (void *data, char *format);
void WavpackEncLittleEndianToNative (void *data, char *format);

#ifdef __cplusplus
}
#endif

#endif
