////////////////////////////////////////////////////////////////////////////
//                           **** WAVPACK ****                            //
//                  Hybrid Lossless Wavefile Compressor                   //
//              Copyright (c) 1998 - 2006 Conifer Software.               //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

This package contains a tiny version of the WavPack 4.40 encoder that might
be used in a "resource limited" CPU environment or form the basis for a
hardware encoding implementation. It is packaged with a demo command-line
program that accepts file specifications for a RIFF WAV file source and a
WavPack file (.wv) destination and optionally a correction file (.wvc) for
demonstrating the hybrid lossless mode. The program is standard C, and a
win32 executable is included which was compiled under MS Visual C++ 6.0
using this command:

cl /O2 tinypack.c wputils.c pack.c words.c bits.c

This program (and the tiny encoder) do not handle placing the WAV RIFF header
into the WavPack file. The latest version of the regular WavPack unpacker
(4.40) and the "tiny decoder" will generate the RIFF header automatically on
unpacking and plugins do not generally use the RIFF header information because
all relevant information is stored natively. However, older versions of the
command-line program will complain about this and require unpacking in "raw"
mode.

For the highest performance the hybrid mode noise shaping default is off,
so the noise in lossy mode will have a perfectly flat spectrum. However, it
can be turned on from the command-line for testing. Also note that unlike
the regular command-line version of WavPack, the hybrid mode bitrate must
be specified in bits per sample rather than kbps.

Unlike the standard WavPack library, this version does not require the entire
block's worth of samples to be visible when the block is packed (the standard
library handles this internally, but still requires the memory resource).
Instead, this version contains buffers to build the blocks incrementally and
provides logic to terminate blocks when the output buffers are nearly full
rather than require that the output buffers always provide a "worst case" size.
This works well because WavPack blocks may contain any number of samples (up
to 128k) and are even reasonably efficient down to just a few thousand samples.

For demonstration purposes this uses a single static copy of the WavpackContext
structure and the output buffers, so obviously it cannot be used for more than
one input file at a time. Also, this decoder will not encode multichannel files
nor will it encode files with floating-point data.

To make this code viable on the greatest number of hardware platforms, the
following are true:

   speed is about 3x realtime on an AMD K6 300 MHz
      ("high" mode 16/44 stereo; normal mode is about twice that fast)

   no floating-point math required; just 32b * 32b = 32b int multiply
   large data areas are static and less than 64K total
   executable code and tables are less than 48K
   no malloc / free usage

To maintain compatibility on various platforms, the following conventions
are used:

   a "char" must be exactly 8-bits
   a "short" must be exactly 16-bits
   an "int" must be at least 16-bits, but may be larger
   the "long" type is not used to avoid problems with 64-bit compilers

A special version has also been created that does not assume 8-bit char types
which makes it suitable for certain TMS DSP processors using the C compiler
that comes with Code Composer Studio that has no 8-bit data type. Contact me
if you are interested in that.

Questions or comments should be directed to david@wavpack.com
