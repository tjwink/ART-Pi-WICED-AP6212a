////////////////////////////////////////////////////////////////////////////
//                           **** WAVPACK ****                            //
//                  Hybrid Lossless Wavefile Compressor                   //
//              Copyright (c) 1998 - 2007 Conifer Software.               //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// tinypack.c

// This is the main module for the demonstration WavPack command-line
// encoder using the "tiny encoder". It accepts a source WAV file, a
// destination WavPack file (.wv) and an optional WavPack correction file
// (.wvc) on the command-line. It supports all 4 encoding qualities in
// pure lossless, hybrid lossy and hybrid lossless modes. Valid input are
// mono or stereo integer WAV files with bitdepths from 8 to 24.

// This program (and the tiny encoder) do not handle placing the WAV RIFF
// header into the WavPack file. The latest version of the regular WavPack
// unpacker (4.40) and the "tiny decoder" will generate the RIFF header
// automatically on unpacking. However, older versions of the command-line
// program will complain about this and require unpacking in "raw" mode.


#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>

#include "wavpack_enc.h"

#define VERSION_STR "4.40"
#define DATE_STR "2007-01-16"

#define CLEAR(destin) memset (&destin, 0, sizeof (destin));

#define FALSE 0
#define TRUE 1

// RIFF / wav header formats (these occur at the beginning of wav files)

typedef struct {
    char ckID [4];
    uint32_t ckSize;
    char formType [4];
} RiffChunkHeader;

typedef struct {
    char ckID [4];
    uint32_t ckSize;
} ChunkHeader;

#define ChunkHeaderFormat "4L"

typedef struct {
    ushort FormatTag, NumChannels;
    uint32_t SampleRate, BytesPerSecond;
    ushort BlockAlign, BitsPerSample;
    ushort cbSize, ValidBitsPerSample;
    int32_t ChannelMask;
    ushort SubFormat;
    char GUID [14];
} WaveHeader;

#define WaveHeaderFormat "SSLLSSSSLS"

///////////////////////////// local variable storage //////////////////////////

static const char *sign_on = "\n"
" TINYPACK  Tiny Audio Compressor  Version %s  %s\n"
" Copyright (c) 1998 - 2007 Conifer Software.  All Rights Reserved.\n\n";

static const char *usage =
" Usage:   TINYPACK [-options] infile.wav outfile.wv [outfile.wvc]\n"
"             (default is lossless)\n\n"
" Options: -bn = enable hybrid compression, n = 2.0 to 24.0 bits/sample\n"
"          -c  = create correction file (.wvc) for hybrid mode (=lossless)\n"
"          -cc = maximum hybrid compression (hurts lossy quality & decode speed)\n"
"          -f  = fast mode (fast, but some compromise in compression ratio)\n"
"          -h  = high quality (better compression in all modes, but slower)\n"
"          -hh = very high quality (best compression in all modes, but slowest\n"
"                                 and NOT recommended for portable hardware use)\n"
"          -jn = joint-stereo override (0 = left/right, 1 = mid/side)\n"
"          -sn = noise shaping override (hybrid only, n = -1.0 to 1.0, 0 = off)\n\n"
" Web:     Visit www.wavpack.com for latest version and info\n";

/////////////////////////// local function declarations ///////////////////////

static int pack_file (char *infilename, char *outfilename, char *out2filename, const WavpackEncConfig *config);
static int pack_audio (WavpackEncContext *wpc, FILE *infile);

static int DoReadFile (FILE *hFile, void *lpBuffer, uint32_t nNumberOfBytesToRead, uint32_t *lpNumberOfBytesRead);
static int DoWriteFile (FILE *hFile, void *lpBuffer, uint32_t nNumberOfBytesToWrite, uint32_t *lpNumberOfBytesWritten);

#define NO_ERROR 0L
#define SOFT_ERROR 1
#define HARD_ERROR 2

//////////////////////////////////////////////////////////////////////////////
// The "main" function for the command-line WavPack compressor.             //
//////////////////////////////////////////////////////////////////////////////

int main (argc, argv) int argc; char **argv;
{
    char *infilename = NULL, *outfilename = NULL, *out2filename = NULL;
    WavpackEncConfig config;
    int error_count = 0;
    int result;

    CLEAR (config);

    // loop through command-line arguments

    while (--argc)
        if (**++argv == '-' || **argv == '/')
            while (*++*argv)
                switch (**argv) {

                    case 'C': case 'c':
                        if (config.flags & CONFIG_CREATE_WVC)
                            config.flags |= CONFIG_OPTIMIZE_WVC;
                        else
                            config.flags |= CONFIG_CREATE_WVC;

                        break;

                    case 'F': case 'f':
                        config.flags |= CONFIG_FAST_FLAG;
                        break;

                    case 'H': case 'h':
                        if (config.flags & CONFIG_HIGH_FLAG)
                            config.flags |= CONFIG_VERY_HIGH_FLAG;
                        else
                            config.flags |= CONFIG_HIGH_FLAG;

                        break;

                    case 'K': case 'k':
                        config.block_samples = strtol (++*argv, argv, 10);
                        --*argv;
                        break;

                    case 'B': case 'b':
                        config.flags |= CONFIG_HYBRID_FLAG;
                        config.bitrate = (int)(strtod (++*argv, argv) * 256.0);
                        --*argv;

                        if (config.bitrate < 512 || config.bitrate > 4096) {
                            fprintf (stderr, "hybrid spec must be 2.0 to 16.0!\n");
                            ++error_count;
                        }

                        break;

                    case 'J': case 'j':
                        switch (strtol (++*argv, argv, 10)) {

                            case 0:
                                config.flags |= CONFIG_JOINT_OVERRIDE;
                                config.flags &= ~CONFIG_JOINT_STEREO;
                                break;

                            case 1:
                                config.flags |= (CONFIG_JOINT_OVERRIDE | CONFIG_JOINT_STEREO);
                                break;

                            default:
                                fprintf (stderr, "-j0 or -j1 only!\n");
                                ++error_count;
                        }
                        
                        --*argv;
                        break;

                    case 'S': case 's':
                        config.shaping_weight = (int)(strtod (++*argv, argv) * 1024.0);

                        if (!config.shaping_weight) {
                            config.flags |= CONFIG_SHAPE_OVERRIDE;
                            config.flags &= ~CONFIG_HYBRID_SHAPE;
                        }
                        else if (config.shaping_weight >= -1024 && config.shaping_weight <= 1024)
                            config.flags |= (CONFIG_HYBRID_SHAPE | CONFIG_SHAPE_OVERRIDE);
                        else {
                            fprintf (stderr, "-s-1.00 to -s1.00 only!\n");
                            ++error_count;
                        }
                        
                        --*argv;
                        break;

                    default:
                        fprintf (stderr, "illegal option: %c !\n", **argv);
                        ++error_count;
                }
        else if (!infilename)
            infilename = *argv;
        else if (!outfilename)
            outfilename = *argv;
        else if (!out2filename)
            out2filename = *argv;
        else {
            fprintf (stderr, "extra unknown argument: %s !\n", *argv);
            ++error_count;
        }

    // check for various command-line argument problems

    if (!(~config.flags & (CONFIG_HIGH_FLAG | CONFIG_FAST_FLAG))) {
        fprintf (stderr, "high and fast modes are mutually exclusive!\n");
        ++error_count;
    }

    if (config.flags & CONFIG_HYBRID_FLAG) {
        if ((config.flags & CONFIG_CREATE_WVC) && !out2filename) {
            fprintf (stderr, "need name for correction file!\n");
            ++error_count;
        }
    }
    else {
        if (config.flags & (CONFIG_SHAPE_OVERRIDE | CONFIG_CREATE_WVC)) {
            fprintf (stderr, "-s and -c options are for hybrid mode (-b) only!\n");
            ++error_count;
        }
    }

    if (out2filename && !(config.flags & CONFIG_CREATE_WVC)) {
        fprintf (stderr, "third filename specified without -c option!\n");
        ++error_count;
    }

    if (!error_count)
        fprintf (stderr, sign_on, VERSION_STR, DATE_STR);
    else
        return 1;

    if (!infilename || !outfilename || (!out2filename && (config.flags & CONFIG_CREATE_WVC))) {
        printf ("%s", usage);
        return 1;
    }

    result = pack_file (infilename, outfilename, out2filename, &config);

    if (result) {
        fprintf (stderr, "error occured!\n");
        ++error_count;
    }

    return error_count ? 1 : 0;
}

// This structure and function are used to write completed WavPack blocks in
// a device independent way.

typedef struct {
    uint32_t bytes_written, first_block_size;
    FILE *file;
    int error;
} write_id;

static int write_block (void *id, void *data, int32_t length)
{
    write_id *wid = (write_id *) id;
    uint32_t bcount;

    if (wid->error)
        return FALSE;

    if (wid && wid->file && data && length) {
        if (!DoWriteFile (wid->file, data, length, &bcount) || bcount != length) {
            fclose (wid->file);
            wid->file = NULL;
            wid->error = 1;
            return FALSE;
        }
        else {
            wid->bytes_written += length;

            if (!wid->first_block_size)
                wid->first_block_size = bcount;
        }
    }

    return TRUE;
}

// This function packs a single file "infilename" and stores the result at
// "outfilename". If "out2filename" is specified, then the "correction"
// file would go there. The files are opened and closed in this function
// and the "config" structure specifies the mode of compression.

static int pack_file (char *infilename, char *outfilename, char *out2filename, const WavpackEncConfig *config)
{
    uint32_t total_samples, bcount;
    WavpackEncConfig loc_config = *config;
    RiffChunkHeader riff_chunk_header;
    write_id wv_file, wvc_file;
    ChunkHeader chunk_header;
    WaveHeader WaveHeader;
    WavpackEncContext *wpc;
    FILE *infile;
    int result;

    CLEAR (wv_file);
    CLEAR (wvc_file);
    wpc = WavpackEncOpenFileOutput (write_block, &wv_file, out2filename ? &wvc_file : NULL);

    // open the source file for reading

    if ((infile = fopen (infilename, "rb")) == NULL) {
        fprintf (stderr, "can't open file %s!\n", infilename);
        WavpackEncCloseFile (wpc);
        return SOFT_ERROR;
    }

    // open output file for writing

    if ((wv_file.file = fopen (outfilename, "w+b")) == NULL) {
        fprintf (stderr, "can't create file %s!\n", outfilename);
        fclose (infile);
        WavpackEncCloseFile (wpc);
        return SOFT_ERROR;
    }

    if ((!DoReadFile (infile, &riff_chunk_header, sizeof (RiffChunkHeader), &bcount) ||
        bcount != sizeof (RiffChunkHeader) || strncmp (riff_chunk_header.ckID, "RIFF", 4) ||
        strncmp (riff_chunk_header.formType, "WAVE", 4))) {
            fprintf (stderr, "%s is not a valid .WAV file!\n", infilename);
            fclose (infile);
            fclose (wv_file.file);
            remove (outfilename);
            WavpackEncCloseFile (wpc);
            return SOFT_ERROR;
    }

    // loop through all elements of the RIFF wav header (until the data chuck)

    while (1) {

        if (!DoReadFile (infile, &chunk_header, sizeof (ChunkHeader), &bcount) ||
            bcount != sizeof (ChunkHeader)) {
                fprintf (stderr, "%s is not a valid .WAV file!\n", infilename);
                fclose (infile);
                fclose (wv_file.file);
                remove (outfilename);
                WavpackEncCloseFile (wpc);
                return SOFT_ERROR;
        }

        WavpackEncLittleEndianToNative (&chunk_header, ChunkHeaderFormat);

        // if it's the format chunk, we want to get some info out of there and
        // make sure it's a .wav file we can handle

        if (!strncmp (chunk_header.ckID, "fmt ", 4)) {
            int supported = TRUE, format;

            if (chunk_header.ckSize < 16 || chunk_header.ckSize > sizeof (WaveHeader) ||
                !DoReadFile (infile, &WaveHeader, chunk_header.ckSize, &bcount) ||
                bcount != chunk_header.ckSize) {
                    fprintf (stderr, "%s is not a valid .WAV file!\n", infilename);
                    fclose (infile);
                    fclose (wv_file.file);
                    remove (outfilename);
                    WavpackEncCloseFile (wpc);
                    return SOFT_ERROR;
            }

            WavpackEncLittleEndianToNative (&WaveHeader, WaveHeaderFormat);

            format = (WaveHeader.FormatTag == 0xfffe && chunk_header.ckSize == 40) ?
                WaveHeader.SubFormat : WaveHeader.FormatTag;

            loc_config.bits_per_sample = chunk_header.ckSize == 40 ?
                WaveHeader.ValidBitsPerSample : WaveHeader.BitsPerSample;

            if (format != 1)
                supported = FALSE;

            if (!WaveHeader.NumChannels || WaveHeader.NumChannels > 2 ||
                WaveHeader.BlockAlign / WaveHeader.NumChannels < (loc_config.bits_per_sample + 7) / 8 ||
                WaveHeader.BlockAlign / WaveHeader.NumChannels > 3 ||
                WaveHeader.BlockAlign % WaveHeader.NumChannels)
                    supported = FALSE;

            if (loc_config.bits_per_sample < 1 || loc_config.bits_per_sample > 24)
                supported = FALSE;

            if (!supported) {
                fprintf (stderr, "%s is an unsupported .WAV format!\n", infilename);
                fclose (infile);
                fclose (wv_file.file);
                remove (outfilename);
                WavpackEncCloseFile (wpc);
                return SOFT_ERROR;
            }
        }
        else if (!strncmp (chunk_header.ckID, "data", 4)) {

            // on the data chunk, get size and exit loop

            total_samples = chunk_header.ckSize / WaveHeader.BlockAlign;
            break;
        }
        else {          // just skip over unknown chunks

            int bytes_to_skip = (chunk_header.ckSize + 1) & ~1L;
            char *buff = malloc (bytes_to_skip);

            if (!DoReadFile (infile, buff, bytes_to_skip, &bcount) || bcount != bytes_to_skip) {
                fprintf (stderr, "%s\n", WavpackEncGetErrorMessage (wpc));
                fclose (infile);
                fclose (wv_file.file);
                remove (outfilename);
                free (buff);
                WavpackEncCloseFile (wpc);
                return SOFT_ERROR;
            }

            free (buff);
        }
    }

    loc_config.bytes_per_sample = WaveHeader.BlockAlign / WaveHeader.NumChannels;
    loc_config.num_channels = WaveHeader.NumChannels;
    loc_config.sample_rate = WaveHeader.SampleRate;

    WavpackEncSetConfiguration (wpc, &loc_config, total_samples);

    // if we are creating a "correction" file, open it now for writing

    if (out2filename) {
        if ((wvc_file.file = fopen (out2filename, "w+b")) == NULL) {
            fprintf (stderr, "can't create correction file!\n");
            fclose (infile);
            fclose (wv_file.file);
            remove (outfilename);
            WavpackEncCloseFile (wpc);
            return SOFT_ERROR;
        }
    }

    // pack the audio portion of the file now

    result = pack_audio (wpc, infile);

    fclose (infile);    // we're now done with input file, so close

    // we're now done with any WavPack blocks, so flush any remaining data

    if (result == NO_ERROR && !WavpackEncFlushSamples (wpc)) {
        fprintf (stderr, "%s\n", WavpackEncGetErrorMessage (wpc));
        result = HARD_ERROR;
    }

    // At this point we're done writing to the output files. However, in some
    // situations we might have to back up and re-write the initial blocks.
    // Currently the only case is if we're ignoring length.

    if (result == NO_ERROR && WavpackEncGetNumSamples (wpc) != WavpackEncGetSampleIndex (wpc)) {
        fprintf (stderr, "couldn't read all samples, file may be corrupt!!\n");
        result = SOFT_ERROR;
    }

    // at this point we're done with the files, so close 'em whether there
    // were any other errors or not

    if (fclose (wv_file.file)) {
        fprintf (stderr, "can't close WavPack file!\n");

        if (result == NO_ERROR)
            result = SOFT_ERROR;
    }

    if (out2filename && fclose (wvc_file.file)) {
        fprintf (stderr, "can't close correction file!\n");

        if (result == NO_ERROR)
            result = SOFT_ERROR;
    }

    // if there were any errors, delete the output files, close the context,
    // and return the error

    if (result != NO_ERROR) {
        remove (outfilename);

        if (out2filename)
            remove (out2filename);

        WavpackEncCloseFile (wpc);
        return result;
    }

    WavpackEncCloseFile (wpc);
    return NO_ERROR;
}

// This function handles the actual audio data compression. It assumes that the
// input file is positioned at the beginning of the audio data and that the
// WavPack configuration has been set. This is where the conversion from RIFF
// little-endian standard the executing processor's format is done.

#define INPUT_SAMPLES (sizeof (int) == 2 ? 4096 : 65536)

static int pack_audio (WavpackEncContext *wpc, FILE *infile)
{
    uint32_t samples_remaining;
    int32_t *sample_buffer;
    int bytes_per_sample;
    uchar *input_buffer;

    WavpackEncPackInit (wpc);
    bytes_per_sample = WavpackEncGetBytesPerSample (wpc) * WavpackEncGetNumChannels (wpc);
    input_buffer = malloc (INPUT_SAMPLES * bytes_per_sample);
    sample_buffer = malloc (INPUT_SAMPLES * sizeof (int32_t) * WavpackEncGetNumChannels (wpc));
    samples_remaining = WavpackEncGetNumSamples (wpc);

    if (!input_buffer || !sample_buffer) {
        fprintf (stderr, "can't allocate input sample buffers!\n");
        return HARD_ERROR;
    }

    while (1) {
        uint32_t sample_count, bytes_to_read, bytes_read = 0;

        if (samples_remaining > INPUT_SAMPLES)
            bytes_to_read = INPUT_SAMPLES * bytes_per_sample;
        else
            bytes_to_read = samples_remaining * bytes_per_sample;

        samples_remaining -= bytes_to_read / bytes_per_sample;
        DoReadFile (infile, input_buffer, bytes_to_read, &bytes_read);
        sample_count = bytes_read / bytes_per_sample;

        if (!sample_count)
            break;

        if (sample_count) {
            uint cnt = sample_count * WavpackEncGetNumChannels (wpc);
            uchar *sptr = input_buffer;
            int32_t *dptr = sample_buffer;

            switch (WavpackEncGetBytesPerSample (wpc)) {

                case 1:
                    while (cnt--)
                        *dptr++ = *sptr++ - 128;

                    break;

                case 2:
                    while (cnt--) {
                        *dptr++ = sptr [0] | ((int32_t)(signed char) sptr [1] << 8);
                        sptr += 2;
                    }

                    break;

                case 3:
                    while (cnt--) {
                        *dptr++ = sptr [0] | ((int32_t) sptr [1] << 8) | ((int32_t)(signed char) sptr [2] << 16);
                        sptr += 3;
                    }

                    break;
            }
        }

        if (!WavpackEncPackSamples (wpc, sample_buffer, sample_count)) {
            fprintf (stderr, "%s\n", WavpackEncGetErrorMessage (wpc));
            free (sample_buffer);
            free (input_buffer);
            return HARD_ERROR;
        }
    }

    free (sample_buffer);
    free (input_buffer);

    if (!WavpackEncFlushSamples (wpc)) {
        fprintf (stderr, "%s\n", WavpackEncGetErrorMessage (wpc));
        return HARD_ERROR;
    }

    return NO_ERROR;
}

//////////////////////////// File I/O Wrapper ////////////////////////////////

static int DoReadFile (FILE *hFile, void *lpBuffer, uint32_t nNumberOfBytesToRead, uint32_t *lpNumberOfBytesRead)
{
    uint32_t bcount;

    *lpNumberOfBytesRead = 0;

    while (nNumberOfBytesToRead) {
        bcount = fread ((uchar *) lpBuffer + *lpNumberOfBytesRead, 1, nNumberOfBytesToRead, hFile);

        if (bcount) {
            *lpNumberOfBytesRead += bcount;
            nNumberOfBytesToRead -= bcount;
        }
        else
            break;
    }

    return !ferror (hFile);
}

static int DoWriteFile (FILE *hFile, void *lpBuffer, uint32_t nNumberOfBytesToWrite, uint32_t *lpNumberOfBytesWritten)
{
    uint32_t bcount;

    *lpNumberOfBytesWritten = 0;

    while (nNumberOfBytesToWrite) {
        bcount = fwrite ((uchar *) lpBuffer + *lpNumberOfBytesWritten, 1, nNumberOfBytesToWrite, hFile);

        if (bcount) {
            *lpNumberOfBytesWritten += bcount;
            nNumberOfBytesToWrite -= bcount;
        }
        else
            break;
    }

    return !ferror (hFile);
}
