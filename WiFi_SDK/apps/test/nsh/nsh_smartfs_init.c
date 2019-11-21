/****************************************************************************
 * apps/nshlib/nsh_smartfs_init.c
 *
 *   Copyright (C) 2007-2012, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nsh.h"

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/smart.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/mksmartfs.h>

#include <arch/bcm4390x/direct_sflash.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Debug for init smartfs
 * #define SMARTFS_INIT_DEBUG
 */

#ifndef CONFIG_MTD_SMART_START_ADDRESS
#define MTD_START_ADDRESS ( 0x200000 )
#else
#define MTD_START_ADDRESS ( CONFIG_MTD_SMART_START_ADDRESS )
#endif /* CONFIG_MTD_SMART_START_ADDRESS */
#ifndef CONFIG_MTD_SMART_PARTITION_MAX
#define MTD_PARTITION_MAX ( 2 ) //create 2 partition in default
#else
#define MTD_PARTITION_MAX ( CONFIG_MTD_SMART_PARTITION_MAX )
#endif /* CONFIG_MTD_SMART_PARTITION_MAX */

#ifdef CONFIG_RAMMTD
#undef MTD_START_ADDRESS
#define MTD_START_ADDRESS ( 0 )
#undef MTD_PARTITION_MAX
#define MTD_PARTITION_MAX ( 1 )
#define MTD_RAM_MAXSIZE ( 1024 * 1024 ) // 1MBytes
static uint8_t g_simflash[MTD_RAM_MAXSIZE];
#endif /* CONFIG_RAMMTD */

#define SMARTFS_ERROR (-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_smartfs_initialize
 *
 * Description:
 *   Initialize smartfs while nsh start.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nsh_smartfs_initialize(void)
{
#ifdef CONFIG_MTD_SMART
    struct mtd_dev_s* mtd;
#ifndef CONFIG_RAMMTD
    struct spi_dev_s* spi;
#endif
    struct mtd_dev_s *part[MTD_PARTITION_MAX];

    struct mtd_geometry_s geo;
    off_t start_address;
    off_t partition_size;
    int ret, i;
    bool partition_unformat_flag;

    ret = 0;
#ifdef CONFIG_RAMMTD
    /* Initialize SmartFS from RAM */
    mtd = rammtd_initialize(g_simflash, MTD_RAM_MAXSIZE);
    if (!mtd)
    {
        fprintf(stderr, "RAMMTD not found\n");
        ret = SMARTFS_ERROR;
        goto error;
    }
    fprintf(stderr, "RAMMTD found\n");
#else
    /* Initialize SmartFS from SFLASH */
    spi = up_bcm4390x_sflash_spiinitialize();
    if (!spi)
    {
        fprintf(stderr, "Failed to initialize BCM4390x direct sflash\n");
        ret = SMARTFS_ERROR;
        goto error;
    }

    mtd = 0;
#ifdef CONFIG_MTD_MACRONIX
    mtd = mxic_initialize( spi );
#endif /* CONFIG_MTD_MACRONIX */

    if (!mtd)
    {
        fprintf(stderr, "FLASH not found\n");
        ret = SMARTFS_ERROR;
        goto error;
    }
    fprintf(stderr, "FLASH found\n");
#endif /* CONFIG_RAMMTD */

    ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
    if (ret < 0)
    {
        fprintf(stderr, "ERROR: mtd->ioctl failed: %d\n", ret);
        ret = SMARTFS_ERROR;
        goto error;
    }

#ifdef SMARTFS_INIT_DEBUG
    fprintf(stderr, "Flash Geometry:\n");
    fprintf(stderr, "  blocksize:      %lu\n", (unsigned long)geo.blocksize);
    fprintf(stderr, "  erasesize:      %lu\n", (unsigned long)geo.erasesize);
    fprintf(stderr, "  neraseblocks:   %lu\n", (unsigned long)geo.neraseblocks);
#endif

    fprintf(stderr, ">>> FLASH or RAMMTD information <<<\n");
    fprintf(stderr, "     Total size :                       %lu\n", (unsigned long)(geo.erasesize * geo.neraseblocks));
    fprintf(stderr, "     Start address (reserved address) : %lu\n", MTD_START_ADDRESS);
    start_address = MTD_START_ADDRESS / geo.blocksize;
    partition_size = ( ( ( geo.erasesize * geo.neraseblocks ) / geo.blocksize ) - start_address ) / MTD_PARTITION_MAX;

    partition_unformat_flag = 0;
    for ( i = 0; i < MTD_PARTITION_MAX; i++ )
    {
        part[i] = mtd_partition(mtd, start_address, partition_size );
        ret = smart_initialize(i, part[i], NULL);
        if (ret < 0)
        {
            fprintf(stderr, "Smartfs create partiton[%d] failed !\n", i);
            ret = SMARTFS_ERROR;
            goto error;
        }
        fprintf(stderr, "     Create partition[%d] done, size :   %lu %s\n",
                i, (unsigned long)( geo.blocksize * partition_size ), ( ret == SMART_FMT_ISFORMATTED ) ? "(formatted)" : "(un-format)");
        if ( ret != SMART_FMT_ISFORMATTED )
        {
            partition_unformat_flag = 1;
        }
        start_address += partition_size;
    }

    /* If there's any exist partition which not be formated,
     * we will show below Tip for helping user to format partition. */
    if ( partition_unformat_flag )
    {
        fprintf(stderr, "    **************************************\n" );
        fprintf(stderr, "    *Tip - How to format/mount partition *\n" );
        fprintf(stderr, "    *ex :                                *\n" );
        fprintf(stderr, "    * mksmartfs /dev/smart0              *\n" );
        fprintf(stderr, "    * mount -t smartfs /dev/smart0 /mnt0 *\n" );
        fprintf(stderr, "    **************************************\n" );
    }

error:
    if (ret == SMARTFS_ERROR)
    {
        fprintf(stderr, "SmartFS initial : Fail\n");
    }
#endif /* CONFIG_MTD_SMART */
}
