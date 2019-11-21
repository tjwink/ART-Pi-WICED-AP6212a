/*
 * $ Copyright Broadcom Corporation $
 */

/*
 * This compatible header includes corresponding header files for getopt_xxx functions
 * depending on toolchain/c_runtime: For GNU/newlib, the getopt.h are included by default;
 * For IAR/Dlib, the ported iar_getopt are used.
 */

#if defined(__IAR_SYSTEMS_ICC__)
  #include "iar_getopt.h"
#elif defined(__NUTTX__)
  #include "nuttx_getopt.h"
#else
  #include "getopt.h"
#endif
