#ifndef _CY_DEFS_
#define _CY_DEFS_

/* AAC STEREO */
/* define to hardcode support for MAX 2ch in the FDK */
//#define CY_FDK_AAC_STEREO
/* */
#ifdef CY_FDK_AAC_STEREO
/* low mem settings for stereo AAC only */
#define CY_FDK_AUDIO_CH_NUM_SETUP (2)
#else
/* HE-AACv2 full support (7.1 channel) */
#define CY_FDK_AUDIO_CH_NUM_SETUP (8)
#endif

/* AAC-LC */
/* define to disable support for SBR and PS */
/* which are related to HE-AACv1/v2         */
/* */
//#define CY_FDK_AAC_LC_ONLY

#endif
