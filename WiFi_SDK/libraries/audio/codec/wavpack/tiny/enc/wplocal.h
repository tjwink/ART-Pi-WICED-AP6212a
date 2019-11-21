////////////////////////////////////////////////////////////////////////////
//                           **** WAVPACK ****                            //
//                  Hybrid Lossless Wavefile Compressor                   //
//              Copyright (c) 1998 - 2007 Conifer Software.               //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// wplocal.h

#ifndef WPLOCAL_H
#define WPLOCAL_H

#if defined(WIN32)
#define FASTCALL __fastcall
#else
#define FASTCALL
#endif

#include <sys/types.h>

// This header file contains all the definitions required by WavPack.

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

#include <stdio.h>

#define FALSE 0
#define TRUE 1

#if defined(__GNUC__) 
#define GCC_UNUSED __attribute__( ( __unused__ ) )
#endif

////////////////////////////// WavPack Header /////////////////////////////////

// Note that this is the ONLY structure that is written to (or read from)
// WavPack 4.0 files, and is the preamble to every block in both the .wv
// and .wvc files.

typedef struct {
    char ckID [4];
    uint32_t ckSize;
    short version;
    short track_index;
    uint32_t total_samples, block_index, block_samples, flags, crc;
} WavpackHeader;

#define WavpackHeaderFormat "4LS2LLLLL"

// or-values for "flags"

#define BYTES_STORED    3       // 1-4 bytes/sample
#define MONO_FLAG       4       // not stereo
#define HYBRID_FLAG     8       // hybrid mode
#define JOINT_STEREO    0x10    // joint stereo
#define CROSS_DECORR    0x20    // no-delay cross decorrelation
#define HYBRID_SHAPE    0x40    // noise shape (hybrid mode only)
#define FLOAT_DATA      0x80    // ieee 32-bit floating point data

#define INT32_DATA      0x100   // special extended int handling
#define HYBRID_BITRATE  0x200   // bitrate noise (hybrid mode only)
#define HYBRID_BALANCE  0x400   // balance noise (hybrid stereo mode only)

#define INITIAL_BLOCK   0x800   // initial block of multichannel segment
#define FINAL_BLOCK     0x1000  // final block of multichannel segment

#define SHIFT_LSB       13
#define SHIFT_MASK      (0x1fL << SHIFT_LSB)

#define MAG_LSB         18
#define MAG_MASK        (0x1fL << MAG_LSB)

#define SRATE_LSB       23
#define SRATE_MASK      (0xfL << SRATE_LSB)

#define FALSE_STEREO    0x40000000      // block is stereo, but data is mono

#define IGNORED_FLAGS   0x18000000      // reserved, but ignore if encountered
#define NEW_SHAPING     0x20000000      // use IIR filter for negative shaping
#define UNKNOWN_FLAGS   0x80000000      // also reserved, but refuse decode if
                                        //  encountered

#define MONO_DATA (MONO_FLAG | FALSE_STEREO)

#define CUR_STREAM_VERS     0x405       // stream version we are writing now


//////////////////////////// WavPack Metadata /////////////////////////////////

// This is an internal representation of metadata.

typedef struct {
    uchar temp_data [64];
    int32_t byte_length;
    void *data;
    uchar id;
} WavpackMetadata;

#define ID_UNIQUE               0x3f
#define ID_OPTIONAL_DATA        0x20
#define ID_ODD_SIZE             0x40
#define ID_LARGE                0x80

#define ID_DUMMY                0x0
#define ID_ENCODER_INFO         0x1
#define ID_DECORR_TERMS         0x2
#define ID_DECORR_WEIGHTS       0x3
#define ID_DECORR_SAMPLES       0x4
#define ID_ENTROPY_VARS         0x5
#define ID_HYBRID_PROFILE       0x6
#define ID_SHAPING_WEIGHTS      0x7
#define ID_FLOAT_INFO           0x8
#define ID_INT32_INFO           0x9
#define ID_WV_BITSTREAM         0xa
#define ID_WVC_BITSTREAM        0xb
#define ID_WVX_BITSTREAM        0xc
#define ID_CHANNEL_INFO         0xd

#define ID_RIFF_HEADER          (ID_OPTIONAL_DATA | 0x1)
#define ID_RIFF_TRAILER         (ID_OPTIONAL_DATA | 0x2)
#define ID_REPLAY_GAIN          (ID_OPTIONAL_DATA | 0x3)
#define ID_CUESHEET             (ID_OPTIONAL_DATA | 0x4)
#define ID_CONFIG_BLOCK         (ID_OPTIONAL_DATA | 0x5)
#define ID_MD5_CHECKSUM         (ID_OPTIONAL_DATA | 0x6)
#define ID_SAMPLE_RATE          (ID_OPTIONAL_DATA | 0x7)

///////////////////////// WavPack Configuration ///////////////////////////////

// This external structure is used during encode to provide configuration to
// the encoding engine.

typedef struct {
    int bitrate, shaping_weight;
    int bits_per_sample, bytes_per_sample, num_channels;
    int32_t block_samples, sample_rate;
    uint32_t flags;
} WavpackEncConfig;

#define CONFIG_BYTES_STORED     3       // 1-4 bytes/sample
#define CONFIG_MONO_FLAG        4       // not stereo
#define CONFIG_HYBRID_FLAG      8       // hybrid mode
#define CONFIG_JOINT_STEREO     0x10    // joint stereo
#define CONFIG_CROSS_DECORR     0x20    // no-delay cross decorrelation
#define CONFIG_HYBRID_SHAPE     0x40    // noise shape (hybrid mode only)
#define CONFIG_FLOAT_DATA       0x80    // ieee 32-bit floating point data

#define CONFIG_FAST_FLAG        0x200   // fast mode
#define CONFIG_HIGH_FLAG        0x800   // high quality mode
#define CONFIG_VERY_HIGH_FLAG   0x1000  // very high
#define CONFIG_BITRATE_KBPS     0x2000  // bitrate is kbps, not bits / sample
#define CONFIG_AUTO_SHAPING     0x4000  // automatic noise shaping
#define CONFIG_SHAPE_OVERRIDE   0x8000  // shaping mode specified
#define CONFIG_JOINT_OVERRIDE   0x10000 // joint-stereo mode specified
#define CONFIG_CREATE_EXE       0x40000 // create executable
#define CONFIG_CREATE_WVC       0x80000 // create correction file
#define CONFIG_OPTIMIZE_WVC     0x100000 // maximize bybrid compression
#define CONFIG_CALC_NOISE       0x800000 // calc noise in hybrid mode
#define CONFIG_LOSSY_MODE       0x1000000 // obsolete (for information)
#define CONFIG_EXTRA_MODE       0x2000000 // extra processing mode
#define CONFIG_SKIP_WVX         0x4000000 // no wvx stream w/ floats & big ints
#define CONFIG_MD5_CHECKSUM     0x8000000 // compute & store MD5 signature
#define CONFIG_OPTIMIZE_MONO    0x80000000 // optimize for mono streams posing as stereo

/*
 * These config flags are no longer used (or were never used) although there
 * may be WavPack files that have some of these bits set in the config
 * metadata structure, so be careful reusing them for something else.
 *
#define CONFIG_ADOBE_MODE       0x100   // "adobe" mode for 32-bit floats
#define CONFIG_VERY_FAST_FLAG   0x400   // double fast
#define CONFIG_VERY_HIGH_FLAG   0x1000  // double high (not used yet)
#define CONFIG_COPY_TIME        0x20000 // copy file-time from source
#define CONFIG_QUALITY_MODE     0x200000 // psychoacoustic quality mode
#define CONFIG_RAW_FLAG         0x400000 // raw mode (not implemented yet)
#define CONFIG_QUIET_MODE       0x10000000 // don't report progress %
#define CONFIG_IGNORE_LENGTH    0x20000000 // ignore length in wav header
#define CONFIG_NEW_RIFF_HEADER  0x40000000 // generate new RIFF wav header
 *
 */

//////////////////////////////// WavPack Stream ///////////////////////////////

// This internal structure contains everything required to handle a WavPack
// "stream", which is defined as a stereo or mono stream of audio samples. For
// multichannel audio several of these would be required. Each stream contains
// pointers to hold a complete allocated block of WavPack data, although it's
// possible to decode WavPack blocks without buffering an entire block.

typedef struct bs {
    uchar *buf, *end, *ptr;
    void (*wrap)(struct bs *bs);
    int error, bc;
    uint32_t sr;
} Bitstream;

#define MAX_STREAMS 1
#define MAX_NTERMS 16
#define MAX_TERM 8

struct decorr_pass {
    int term, delta, weight_A, weight_B;
    int32_t samples_A [MAX_TERM], samples_B [MAX_TERM];
    int32_t aweight_A, aweight_B;
};

typedef struct {
    WavpackHeader wphdr;

    uchar *blockbuff, *blockend;
    uchar *block2buff, *block2end;

    int bits, num_terms, shift, lossy_block;
    uint32_t sample_index, crc, crc_x;
    Bitstream wvbits, wvcbits;

    struct {
        int32_t shaping_acc [2], shaping_delta [2], error [2];
    } dc;

    struct decorr_pass decorr_passes [MAX_NTERMS];

    struct {
        uint32_t bitrate_delta [2], bitrate_acc [2];
        uint32_t median [3] [2], slow_level [2], error_limit [2];
        uint32_t pend_data, holding_one, zeros_acc;
        int holding_zero, pend_count;
    } w;
} WavpackStream;

/////////////////////////////// WavPack Context ///////////////////////////////

// This internal structure holds everything required to encode or decode WavPack
// files. It is recommended that direct access to this structure be minimized
// and the provided utilities used instead.

typedef int (*WavpackBlockOutput)(void *id, void *data, int32_t bcount);

typedef struct {
    WavpackEncConfig config;

    WavpackBlockOutput blockout;
    void *wv_out, *wvc_out;

    int wvc_flag, lossy_blocks;
    uint32_t block_samples, acc_samples;
    uint32_t filelen, file2len, total_samples;

    int stream_version;
    WavpackStream stream;

    char error_message [80];
} WavpackEncContext;

//////////////////////// function prototypes and macros //////////////////////

#define CLEAR(destin) memset (&destin, 0, sizeof (destin));

// These macros implement the weight application and update operations
// that are at the heart of the decorrelation loops. Note that when there
// are several alternative versions of the same macro (marked with PERFCOND)
// then the versions are functionally equivalent with respect to WavPack
// decoding and the user should choose the one that provides the best
// performance.

#if 1   // PERFCOND
#define apply_weight_i(weight, sample) ((weight * sample + 512) >> 10)
#else
#define apply_weight_i(weight, sample) ((((weight * sample) >> 8) + 2) >> 2)
#endif

#define apply_weight_f(weight, sample) (((((sample & 0xffffL) * weight) >> 9) + \
    (((sample & ~0xffffL) >> 9) * weight) + 1) >> 1)

#if 1   // PERFCOND
#define apply_weight(weight, sample) (sample != (short) sample ? \
    apply_weight_f (weight, sample) : apply_weight_i (weight, sample))
#else
#define apply_weight(weight, sample) ((int32_t)((weight * (int64_t) sample + 512) >> 10))
#endif

#if 0   // PERFCOND
#define update_weight(weight, delta, source, result) \
    if (source && result) { int32_t s = (int32_t) (source ^ result) >> 31; weight = (delta ^ s) + (weight - s); }
#elif 1
#define update_weight(weight, delta, source, result) \
    if (source && result) weight += (((source ^ result) >> 30) | 1) * delta
#else
#define update_weight(weight, delta, source, result) \
    if (source && result) (source ^ result) < 0 ? (weight -= delta) : (weight += delta)
#endif

#define update_weight_clip(weight, delta, source, result) \
    if (source && result && ((source ^ result) < 0 ? (weight -= delta) < -1024 : (weight += delta) > 1024)) \
        weight = weight < 0 ? -1024 : 1024

// bits.c

void bs_open_write (Bitstream *bs, uchar *buffer_start, uchar *buffer_end);
uint32_t bs_remain_write (Bitstream *bs);
uint32_t bs_close_write (Bitstream *bs);

#define bs_is_open(bs) ((bs)->ptr != NULL)

#define putbit(bit, bs) { if (bit) (bs)->sr |= (1L << (bs)->bc); \
    if (++((bs)->bc) == 8) { \
        *((bs)->ptr) = (bs)->sr; \
        (bs)->sr = (bs)->bc = 0; \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs); \
    }}

#define putbit_0(bs) { \
    if (++((bs)->bc) == 8) { \
        *((bs)->ptr) = (bs)->sr; \
        (bs)->sr = (bs)->bc = 0; \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs); \
    }}

#define putbit_1(bs) { (bs)->sr |= (1L << (bs)->bc); \
    if (++((bs)->bc) == 8) { \
        *((bs)->ptr) = (bs)->sr; \
        (bs)->sr = (bs)->bc = 0; \
        if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs); \
    }}

#define putbits(value, nbits, bs) { \
    (bs)->sr |= (int32_t)(value) << (bs)->bc; \
    if (((bs)->bc += (nbits)) >= 8) \
        do { \
            *((bs)->ptr) = (bs)->sr; \
            (bs)->sr >>= 8; \
            if (((bs)->bc -= 8) > 24) (bs)->sr |= ((value) >> ((nbits) - (bs)->bc)); \
            if (++((bs)->ptr) == (bs)->end) (bs)->wrap (bs); \
        } while ((bs)->bc >= 8); \
}

// pack.c

void pack_init (WavpackEncContext *wpc);
int pack_start_block (WavpackEncContext *wpc);
uint32_t pack_samples (WavpackEncContext *wpc, int32_t *buffer, uint32_t sample_count);
int pack_finish_block (WavpackEncContext *wpc);

// words.c stuff

void init_words (WavpackStream *wps);
void word_set_bitrate (WavpackStream *wps);
void write_entropy_vars (WavpackStream *wps, WavpackMetadata *wpmd);
void write_hybrid_profile (WavpackStream *wps, WavpackMetadata *wpmd);
int read_entropy_vars (WavpackStream *wps, WavpackMetadata *wpmd);
int read_hybrid_profile (WavpackStream *wps, WavpackMetadata *wpmd);
int32_t FASTCALL send_word (WavpackStream *wps, int32_t value, int chan);
void FASTCALL send_word_lossless (WavpackStream *wps, int32_t value, int chan);
int32_t FASTCALL get_word (WavpackStream *wps, int chan, int32_t *correction);
int32_t FASTCALL get_word_lossless (WavpackStream *wps, int chan);
void flush_word (WavpackStream *wps);
int32_t nosend_word (WavpackStream *wps, int32_t value, int chan);
void scan_word (WavpackStream *wps, int32_t *samples, uint32_t num_samples, int dir);

int log2s (int32_t value);
int32_t exp2s (int log);
uint32_t log2buffer (int32_t *samples, uint32_t num_samples, int limit);

signed char store_weight (int weight);
int restore_weight (signed char weight);

#define WORD_EOF (1L << 31)

// wputils.c

int WavpackGetVersion (WavpackEncContext *wpc);
uint32_t WavpackUnpackSamples (WavpackEncContext *wpc, int32_t *buffer, uint32_t samples);
uint32_t WavpackGetNumSamples (WavpackEncContext *wpc);
uint32_t WavpackGetSampleIndex (WavpackEncContext *wpc);
int WavpackGetNumErrors (WavpackEncContext *wpc);
int WavpackLossyBlocks (WavpackEncContext *wpc);
int WavpackSeekSample (WavpackEncContext *wpc, uint32_t sample);
WavpackEncContext *WavpackCloseFile (WavpackEncContext *wpc);
uint32_t WavpackGetSampleRate (WavpackEncContext *wpc);
int WavpackGetBitsPerSample (WavpackEncContext *wpc);
int WavpackGetBytesPerSample (WavpackEncContext *wpc);
int WavpackGetNumChannels (WavpackEncContext *wpc);
int WavpackGetReducedChannels (WavpackEncContext *wpc);
int WavpackGetMD5Sum (WavpackEncContext *wpc, uchar data [16]);
uint32_t WavpackGetWrapperBytes (WavpackEncContext *wpc);
uchar *WavpackGetWrapperData (WavpackEncContext *wpc);
void WavpackFreeWrapper (WavpackEncContext *wpc);
double WavpackGetProgress (WavpackEncContext *wpc);
uint32_t WavpackGetFileSize (WavpackEncContext *wpc);
double WavpackGetRatio (WavpackEncContext *wpc);
double WavpackGetAverageBitrate (WavpackEncContext *wpc, int count_wvc);
double WavpackGetInstantBitrate (WavpackEncContext *wpc);
int WavpackGetNumTagItems (WavpackEncContext *wpc);
int WavpackGetTagItem (WavpackEncContext *wpc, const char *item, char *value, int size);
int WavpackGetTagItemIndexed (WavpackEncContext *wpc, int index, char *item, int size);
int WavpackAppendTagItem (WavpackEncContext *wpc, const char *item, const char *value, int vsize);
int WavpackDeleteTagItem (WavpackEncContext *wpc, const char *item);
int WavpackWriteTag (WavpackEncContext *wpc);

WavpackEncContext *WavpackOpenFileOutput (WavpackBlockOutput blockout, void *wv_id, void *wvc_id);
int WavpackSetConfiguration (WavpackEncContext *wpc, WavpackEncConfig *config, uint32_t total_samples);
int WavpackAddWrapper (WavpackEncContext *wpc, void *data, uint32_t bcount);
int WavpackStoreMD5Sum (WavpackEncContext *wpc, uchar data [16]);
int WavpackPackInit (WavpackEncContext *wpc);
int WavpackPackSamples (WavpackEncContext *wpc, int32_t *sample_buffer, uint32_t sample_count);
int WavpackFlushSamples (WavpackEncContext *wpc);
void WavpackUpdateNumSamples (WavpackEncContext *wpc, void *first_block);
void *WavpackGetWrapperLocation (void *first_block, uint32_t *size);
void WavpackEncNativeToLittleEndian (void *data, char *format);
void WavpackEncLittleEndianToNative (void *data, char *format);

#endif
