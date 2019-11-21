/**************************************************************************/
/*                                                                        */
/*            Copyright (c) 1996-2017 by Express Logic Inc.               */
/*                                                                        */
/*  This software is copyrighted by and is the sole property of Express   */
/*  Logic, Inc.  All rights, title, ownership, or other interests         */
/*  in the software remain the property of Express Logic, Inc.  This      */
/*  software may only be used in accordance with the corresponding        */
/*  license agreement.  Any unauthorized use, duplication, transmission,  */
/*  distribution, or disclosure of this software is expressly forbidden.  */
/*                                                                        */
/*  This Copyright notice may not be removed or modified without prior    */
/*  written consent of Express Logic, Inc.                                */
/*                                                                        */
/*  Express Logic, Inc. reserves the right to modify this software        */
/*  without notice.                                                       */
/*                                                                        */
/*  Express Logic, Inc.                     info@expresslogic.com         */
/*  11423 West Bernardo Court               http://www.expresslogic.com   */
/*  San Diego, CA  92127                                                  */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** FileX Component                                                       */
/**                                                                       */
/**   User Specific                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
/*                                                                        */
/*  PORT SPECIFIC C INFORMATION                            RELEASE        */
/*                                                                        */
/*    fx_user.h                                           PORTABLE C      */
/*                                                           5.5          */
/*                                                                        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    William E. Lamie, Express Logic, Inc.                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file contains user defines for configuring FileX in specific   */
/*    ways. This file will have an effect only if the application and     */
/*    FileX library are built with FX_INCLUDE_USER_DEFINE_FILE defined.   */
/*    Note that all the defines in this file may also be made on the      */
/*    command line when building FileX library and application objects.   */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  12-12-2005     William E. Lamie         Initial Version 5.0           */
/*  07-18-2007     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.1    */
/*  03-01-2009     William E. Lamie         Modified comment(s), and      */
/*                                            added new conditional       */
/*                                            define to support disabling */
/*                                            of cache fill on direct     */
/*                                            reads, resulting in         */
/*                                            version 5.2                 */
/*  11-01-2015     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.3    */
/*  04-15-2016     William E. Lamie         Modified comment(s), and      */
/*                                            modified the default values */
/*                                            of symbols, added new       */
/*                                            symbols for fault tolerant  */
/*                                            and 64-bit sector addresses */
/*                                            in I/O driver, resulting    */
/*                                            in version 5.4              */
/*  04-03-2017     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.5    */
/*                                                                        */
/**************************************************************************/

#ifndef FX_USER_H
#define FX_USER_H


/* Define various build options for the FileX port.  The application should either make changes
   here by commenting or un-commenting the conditional compilation defined OR supply the defines
   though the compiler's equivalent of the -D option.  */


/* Override various options with default values already assigned in fx_api.h or fx_port.h. Please
   also refer to fx_port.h for descriptions on each of these options.  */


/* Defines the maximum size of long file names supported by FileX. The default value is 33. The
   minimum value is 13 and the maximum value is 256.  */

/*#define FX_MAX_LONG_NAME_LEN            256   */
/*#define FX_MAX_LAST_NAME_LEN            256   */      /* Must be as large or larger than FX_MAX_LONG_NAME_LEN */


/* Defines the maximum number of logical sectors that can be cached by FileX. The cache memory
   supplied to FileX at fx_media_open determines how many sectors can actually be cached.  */

/*#define FX_MAX_SECTOR_CACHE             256   */      /* Minimum value is 2, all other values must be power of 2.  */


/* Defines the size in bytes of the bit map used to update the secondary FAT sectors. The larger the value the
   less unnecessary secondary FAT sector writes.   */

/*#define FX_FAT_MAP_SIZE                 128  */       /* Minimum value is 1, no maximum value.  */


/* Defines the number of entries in the FAT cache.  */

/*#define FX_MAX_FAT_CACHE                16   */       /* Minimum value is 8, all values must be a power of 2.  */


/* Defines the number of seconds the time parameters are updated in FileX.  */

/*#define FX_UPDATE_RATE_IN_SECONDS       10   */


/* Defines the number of ThreadX timer ticks required to achieve the update rate specified by
   FX_UPDATE_RATE_IN_SECONDS defined previously. By default, the ThreadX timer tick is 10ms,
   so the default value for this constant is 1000.  */

/*#define FX_UPDATE_RATE_IN_TICKS         1000 */


/* Defined, FileX is built without update to the time parameters.  */

/*#define FX_NO_TIMER  */


/* Defined, FileX is running in a single-threaded environment and does not need thread
   protection.  */

/*#define FX_SINGLE_THREAD   */


/* Defined, FileX does not update already opened files.  */

/*#define FX_DONT_UPDATE_OPEN_FILES   */


/* Defined, the file search cache optimization is disabled.  */

/*#define FX_MEDIA_DISABLE_SEARCH_CACHE  */


/* Defined, the direct read sector update of cache is disabled.  */

/*#define FX_DISABLE_DIRECT_DATA_READ_CACHE_FILL  */


/* Defined, gathering of media statistics is disabled.  */

/*#define FX_MEDIA_STATISTICS_DISABLE  */


/* Defined, legacy single open logic for the same file is enabled.  */

/*#define FX_SINGLE_OPEN_LEGACY   */


/* Defined, renaming inherits path information.  */

/*#define FX_RENAME_PATH_INHERIT   */


/* Defined, local path logic is disabled.  */

/*#define FX_NO_LOCAL_PATH   */


/* Defined, FileX is able to access exFAT file system. */

/* #define FX_ENABLE_EXFAT   */


/* Defined, data sector write requests are flushed immediately to the driver.  */

/*#define FX_FAULT_TOLERANT_DATA  */


/* Defined, system sector write requests (including FAT and directory entry requests)
   are flushed immediately to the driver.  */

/*#define FX_FAULT_TOLERANT   */


/* Defined, enables 64-bits sector addresses used in I/O driver.  */

/*#define FX_DRIVER_USE_64BIT_LBA   */


/* Defined, enables FileX fault tolerant service.  */

/*#define FX_ENABLE_FAULT_TOLERANT   */


/* Define byte offset in boot sector where the cluster number of the Fault Tolerant Log file is.
   Note that this field (byte 116 to 119) is marked as reserved by FAT 12/16/32/exFAT specification. */

/*#define FX_FAULT_TOLERANT_BOOT_INDEX      116 */


/* Define the requirement for minimal size of cluster in bytes. It must be multiple of sector size.
   When size of cluster is less than this value, fault tolerant feature does not work properly.
   The default value works with the worst case for long file name. */

/*#define FX_FAULT_TOLERANT_MINIMAL_CLUSTER 3072 */


#endif

