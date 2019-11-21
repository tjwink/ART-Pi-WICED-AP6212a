 AAC decoder wrapper for Fraunhofer AAC SDK (FDK)
 This library is dedicated to the decoding of .m4a audio files
 with a simplified API to decode an incoming packet stream into
 PCM samples.


 List of Files
 -------------

 AACDEC: decoder component
   aacdec.h         : external API of the aac decoder wrapper
   aacdec_types.h   : types defined for the above api
   aacdec_core.h    : component data structure, private
   aacdec.c         : implemementation

 MP4PARSE: internal component interface with the FDK
   mp4parse.h       : internal API for mp4parse
   mp4parse_types.h : types for the above api
   mp4parse_core.h  : private component structure
   mp4parse.c       : implementation


 GLOBALS:
   globals.h
   logger.h

 TOOLS: macros (only macros) to handle sleeps and thread creation
   sysclk.h         : sysclock: simple include select
   sysclk_wiced.h   : clock syscall for wiced
   sysclk_x86.h     : clock syscall for linux/x86
   tsp.h            : thread_sync_procedures: simple include select
   tsp_wiced.h      : tsp for wiced
   tsp_x86.h        : tsp for linux/x86


 -EOF-