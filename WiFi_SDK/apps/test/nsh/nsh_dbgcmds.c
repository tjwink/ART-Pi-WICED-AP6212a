/****************************************************************************
 * apps/nshlib/dbg_dbgcmds.c
 *
 *   Copyright (C) 2008-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <nuttx/rtc.h>

#if CONFIG_NFILE_DESCRIPTORS > 0
# include <fcntl.h>
#endif

#include "nsh.h"
#include "nsh_console.h"

#if defined(CONFIG_ARCH_CHIP_BCM4390X) && defined(CONFIG_SPI)
#include <nuttx/spi/spi.h>
#include "wiced_platform.h"
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dbgmem_s
{
  bool         dm_write;  /* true: perfrom write operation */
  void        *dm_addr;   /* Address to access */
  uint32_t     dm_value;  /* Value to write */
  unsigned int dm_count;  /* The number of bytes to access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined (CONFIG_RTC) && !defined(CONFIG_NSH_DISABLE_DATE)
static char* months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mem_parse
 ****************************************************************************/

static int mem_parse(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv,
              struct dbgmem_s *mem)
{
  char *pcvalue = strchr(argv[1], '=');
  unsigned long lvalue = 0;

  /* Check if we are writing a value */

  if (pcvalue)
    {
      *pcvalue = '\0';
      pcvalue++;

      lvalue = (unsigned long)strtol(pcvalue, NULL, 16);
      if (lvalue > 0xffffffffL)
        {
          return -EINVAL;
        }

      mem->dm_write = true;
      mem->dm_value = (uint32_t)lvalue;
    }
  else
    {
      mem->dm_write = false;
      mem->dm_value = 0;
    }

  /* Get the address to be accessed */

  mem->dm_addr = (void*)((uintptr_t)strtol(argv[1], NULL, 16));

  /* Get the number of bytes to access */

  if (argc > 2)
    {
      mem->dm_count = (unsigned int)strtol(argv[2], NULL, 16);
    }
  else
    {
      mem->dm_count = 1;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_mb
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_MB
int cmd_mb(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct dbgmem_s mem;
  volatile uint8_t *ptr;
  unsigned int i;
  int ret;

  ret = mem_parse(vtbl, argc, argv, &mem);
  if (ret == 0)
    {
      /* Loop for the number of requested bytes */

      for (i = 0, ptr = (volatile uint8_t*)mem.dm_addr; i < mem.dm_count; i++, ptr++)
        {
          /* Print the value at the address */

          nsh_output(vtbl, "  %p = 0x%02x", ptr, *ptr);

          /* Are we supposed to write a value to this address? */

          if (mem.dm_write)
            {
              /* Yes, was the supplied value within range? */

              if (mem.dm_value > 0x000000ff)
                {
                  nsh_output(vtbl, g_fmtargrange, argv[0]);
                  return ERROR;
                }

              /* Write the value and re-read the address so that we print its
               * current value (if the address is a process address, then the
               * value read might not necessarily be the value written).
               */

              *ptr = (uint8_t)mem.dm_value;
              nsh_output(vtbl, " -> 0x%02x", *ptr);
            }

          /* Make sure we end it with a newline */

          nsh_output(vtbl, "\n", *ptr);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: cmd_mh
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_MH
int cmd_mh(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct dbgmem_s mem;
  volatile uint16_t *ptr;
  unsigned int i;
  int ret;

  ret = mem_parse(vtbl, argc, argv, &mem);
  if (ret == 0)
    {
      /* Loop for the number of requested bytes */

      for (i = 0, ptr = (volatile uint16_t*)mem.dm_addr;
           i < mem.dm_count;
           i += 2, ptr++)
        {
          /* Print the value at the address */

          nsh_output(vtbl, "  %p = 0x%04x", ptr, *ptr);

          /* Are we supposed to write a value to this address? */

          if (mem.dm_write)
            {
              /* Yes, was the supplied value within range? */

              if (mem.dm_value > 0x0000ffff)
                {
                  nsh_output(vtbl, g_fmtargrange, argv[0]);
                  return ERROR;
                }

              /* Write the value and re-read the address so that we print its
               * current value (if the address is a process address, then the
               * value read might not necessarily be the value written).
               */

              *ptr = (uint16_t)mem.dm_value;
              nsh_output(vtbl, " -> 0x%04x", *ptr);
            }

          /* Make sure we end it with a newline */

          nsh_output(vtbl, "\n", *ptr);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: cmd_mw
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_MW
int cmd_mw(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct dbgmem_s mem;
  volatile uint32_t *ptr;
  unsigned int i;
  int ret;

  ret = mem_parse(vtbl, argc, argv, &mem);
  if (ret == 0)
    {
      /* Loop for the number of requested bytes */

      for (i = 0, ptr = (volatile uint32_t*)mem.dm_addr; i < mem.dm_count; i += 4, ptr++)
        {
          /* Print the value at the address */

          nsh_output(vtbl, "  %p = 0x%08x", ptr, *ptr);

          /* Are we supposed to write a value to this address? */

          if (mem.dm_write)
            {
              /* Write the value and re-read the address so that we print its
               * current value (if the address is a process address, then the
               * value read might not necessarily be the value written).
               */

              *ptr = mem.dm_value;
              nsh_output(vtbl, " -> 0x%08x", *ptr);
            }

          /* Make sure we end it with a newline */

          nsh_output(vtbl, "\n", *ptr);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nsh_dumpbuffer
 ****************************************************************************/

void nsh_dumpbuffer(FAR struct nsh_vtbl_s *vtbl, const char *msg,
                    const uint8_t *buffer, ssize_t nbytes)
{
  char line[128];
  int ch;
  int i;
  int j;

  nsh_output(vtbl, "%s:\n", msg);
  for (i = 0; i < nbytes; i += 16)
    {
      sprintf(line, "%04x: ", i);

      for ( j = 0; j < 16; j++)
        {
          if (i + j < nbytes)
            {
              sprintf(&line[strlen(line)], "%02x ", buffer[i+j] );
            }
          else
            {
              strcpy(&line[strlen(line)], "   ");
            }
        }

      for ( j = 0; j < 16; j++)
        {
          if (i + j < nbytes)
            {
              ch = buffer[i+j];
              sprintf(&line[strlen(line)], "%c", ch >= 0x20 && ch <= 0x7e ? ch : '.');
            }
        }

      nsh_output(vtbl, "%s\n", line);
    }
}

/****************************************************************************
 * Name: cmd_xd, hex dump of memory
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_XD
int cmd_xd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  FAR char *addr;
  FAR char *endptr;
  int       nbytes;

  addr = (char*)((uintptr_t)strtol(argv[1], &endptr, 16));
  if (argv[0][0] == '\0' || *endptr != '\0')
    {
      return ERROR;
    }

  nbytes = (int)strtol(argv[2], &endptr, 0);
  if (argv[0][0] == '\0' || *endptr != '\0' || nbytes < 0)
    {
      return ERROR;
    }

  nsh_dumpbuffer(vtbl, "Hex dump", (uint8_t*)addr, nbytes);
  return OK;
}
#endif

/****************************************************************************
 * Name: cmd_hexdump, hex dump of files
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
#ifndef CONFIG_NSH_DISABLE_HEXDUMP
int cmd_hexdump(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  FAR uint8_t *buffer;
  char msg[32];
  off_t position;
  int fd;
  int ret = OK;
#ifdef CONFIG_NSH_CMDOPT_HEXDUMP
  off_t skip = 0;
  off_t count = 0xfffffff;
  off_t dumpbytes;
  int x;
#endif

  /* Open the file for reading */

  fd = open(argv[1], O_RDONLY);
  if (fd < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, "hexdump", "open", NSH_ERRNO);
      return ERROR;
    }

  buffer = (FAR uint8_t *)malloc(IOBUFFERSIZE);
  if(buffer == NULL)
    {
      nsh_output(vtbl, g_fmtcmdfailed, "hexdump", "malloc", NSH_ERRNO);
      return ERROR;
    }

#ifdef CONFIG_NSH_CMDOPT_HEXDUMP
  for (x = 2; x < argc; x++)
    {
      if (strncmp(argv[x], "skip=", 5) == 0)
        {
          skip = atoi(&argv[x][5]);
        }
      else if (strncmp(argv[x], "count=", 6) == 0)
        {
          count = atoi(&argv[x][6]);
        }
    }
#endif

  position = 0;
  for (;;)
    {
      int nbytesread = read(fd, buffer, IOBUFFERSIZE);

      /* Check for read errors */

      if (nbytesread < 0)
        {
          int errval = errno;
          nsh_output(vtbl, g_fmtcmdfailed, "hexdump", "read",
                     NSH_ERRNO_OF(errval));
          ret = ERROR;
          break;
        }
      else if (nbytesread > 0)
        {
#ifdef CONFIG_NSH_CMDOPT_HEXDUMP
          if (position < skip)
            {
              /* Skip bytes until we reach the skip point */

              position += nbytesread;
              if (position > skip)
                {
                  dumpbytes = position - skip;
                  if (dumpbytes > count)
                    {
                      dumpbytes = count;
                    }

                  snprintf(msg, sizeof(msg), "%s at %08x", argv[1], skip);
                  nsh_dumpbuffer(vtbl, msg,
                                 &buffer[nbytesread - (position-skip)],
                                 dumpbytes);

                  if (count > dumpbytes)
                    {
                      count -= dumpbytes;
                    }
                  else
                    {
                      break;
                    }
                }

              /* Don't print if we are in skip mode */

              continue;
            }

          /* Limit dumpbuffer to count if less than a full buffer needed */

          if (nbytesread > count)
            {
              nbytesread = count;
            }
#endif

          snprintf(msg, sizeof(msg), "%s at %08x", argv[1], position);
          nsh_dumpbuffer(vtbl, msg, buffer, nbytesread);
          position += nbytesread;

#ifdef CONFIG_NSH_CMDOPT_HEXDUMP
          if (count > nbytesread)
            {
              count -= nbytesread;
            }
          else
            {
              break;
            }
#endif
        }
      else
        {
          break; // EOF
        }
    }

  (void)close(fd);
  free(buffer);
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_date
 ****************************************************************************/

#if defined (CONFIG_RTC) && !defined(CONFIG_NSH_DISABLE_DATE)
/*
 * This function only accepts date and time format in "MMM DD HH:MM:SS YYYY".
 * This function is implemented since NuttX libc does not provide strptime().
 */
static int str_to_time(FAR const char* time_str, FAR struct tm* time_ptr)
{
  int result = ERROR;
  char time_buf[256];
  char* time_token;
  int token_index;
  int token_count;
  int sec = -1;
  int min = -1;
  int hour = -1;
  int mday = -1;
  int mon = -1;
  int year = -1;

  if (strlen(time_str) >= sizeof(time_buf))
    {
      result = ERROR;
    }
  else
    {
      strcpy(time_buf, time_str);
      token_index = 0;
      token_count = 6;
      time_token = strtok(time_buf, " :");

      while (time_token != NULL)
        {
          if (token_index == 0)
            {
              /* MMM (month, range 0 to 11) */
              for (mon = 0 ; mon < 12 ; mon++)
                {
                  if (strcmp(time_token, months[mon]) == 0)
                    {
                      break;
                    }
                }
            }
          else if (token_index == 1)
            {
              /* DD (day of the month, range 1 to 31) */
              mday = atoi(time_token);
            }
          else if (token_index == 2)
            {
              /* HH (hours, range 0 to 23) */
              hour = atoi(time_token);
            }
          else if (token_index == 3)
            {
              /* MM (minutes, range 0 to 59) */
              min = atoi(time_token);
            }
          else if (token_index == 4)
            {
              /* SS (seconds,  range 0 to 61) */
              sec = atoi(time_token);
            }
          else if (token_index == 5)
            {
              /* YYYY (year, range >= 2000 ) */
              year = atoi(time_token);
            }

          time_token = strtok(NULL, " :");
          token_index++;
        }

      if (token_index != token_count)
        {
          result = ERROR;
        }
      else
        {
          if (((mon >= 0) && (mon <= 11)) &&
              ((mday >= 1) && (mday <= 31)) &&
              ((hour >= 0) && (hour <= 23)) &&
              ((min >= 0) && (min <= 59)) &&
              ((sec >= 0) && (sec <= 61)) &&
              (year >= 2000))
            {
              time_ptr->tm_sec = sec;
              time_ptr->tm_min = min;
              time_ptr->tm_hour = hour;
              time_ptr->tm_mday = mday;
              time_ptr->tm_mon = mon;
              time_ptr->tm_year = year - 1900;
#ifdef CONFIG_LIBC_LOCALTIME
              time_ptr->tm_isdst = -1;
#endif

              result = OK;
            }
          else
            {
              result = ERROR;
            }
        }
    }

  return result;
}

int cmd_date(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int ret = ERROR;

  if (argc == 1)
    {
      /* Get the date and time (RTC should already be initialized during boot-up) */
#ifdef CONFIG_RTC_DATETIME
      FAR struct tm time;
      char time_buf[256];

      if (up_rtc_getdatetime(&time) != OK)
        {
          ret = ERROR;
        }
      else
        {
          /* The expected date and time format is "MMM DD HH:MM:SS YYYY" */
          if (strftime(time_buf, sizeof(time_buf), "%b %d %H:%M:%S %Y", &time) == 0)
            {
              ret = ERROR;
            }
          else
            {
              nsh_output(vtbl, "%s\n", time_buf);
              ret = OK;
            }
        }
#else
#ifndef CONFIG_RTC_HIRES
      FAR time_t time_sec;
      FAR struct tm time;
      char time_buf[256];

      time_sec = up_rtc_time();

      if (time_sec == 0)
        {
          ret = ERROR;
        }
      else
        {
          (void)gmtime_r(&time_sec, &time);

          /* The expected date and time format is "MMM DD HH:MM:SS YYYY" */
          if (strftime(time_buf, sizeof(time_buf), "%b %d %H:%M:%S %Y", &time) == 0)
            {
              ret = ERROR;
            }
          else
            {
              nsh_output(vtbl, "%s\n", time_buf);
              ret = OK;
            }
        }
#else
      ret = ERROR;
#endif
#endif

      if (ret != OK)
        {
          nsh_output(vtbl, "ERROR - Failed to get date and time\n");
        }
    }
  else if (argc == 3)
    {
      if (strcmp(argv[1], "-s") != 0)
        {
          ret = ERROR;
        }
      else
        {
          /* Set the date and time (RTC should already be initialized during boot-up) */
          struct tm time;
          FAR struct timespec ts;

          /* The expected date and time format is "MMM DD HH:MM:SS YYYY" */
          if (str_to_time(argv[2], &time) != OK)
            {
              ret = ERROR;
            }
          else
            {
              ts.tv_sec = mktime(&time);

              if (up_rtc_settime(&ts) != OK)
                {
                  ret = ERROR;
                }
              else
                {
                  ret = OK;
                }
            }
        }

      if (ret != OK)
        {
          nsh_output(vtbl, "ERROR - Failed to set date and time\n");
        }
    }
  else
    {
      nsh_output(vtbl, "ERROR - Invalid usage of date command\n");
      ret = ERROR;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: cmd_spi_readid
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_BCM4390X
#ifdef CONFIG_SPI
#  define SFLASH_READ_JEDEC_ID 0x9f
int cmd_spi_readid(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *ptr          = argv[1];
  uint8_t read_cmd[] = {SFLASH_READ_JEDEC_ID, 0, 0, 0};
  uint8_t read_id[]  = {0, 0, 0, 0};
  int port;
  char *endptr;
  struct spi_dev_s *spi;

  if(ptr[0] < '0' || ptr[0] > '9')
  {
    goto invalid_arg;
  }

  port = strtol(&ptr[0], &endptr, 0);

  spi = up_spiinitialize(port);

  if(!spi)
  {
      nsh_output(vtbl, "ERROR ! Failed to initialize SPI Port \n");
      return ERROR;
  }

#ifndef CONFIG_SPI_EXCHANGE
# error CONFIG_SPI_EXCHANGE must be defined for spi_readid
#else
  SPI_LOCK(spi, TRUE);
  SPI_SELECT(spi, SPIDEV_FLASH, TRUE);
  SPI_EXCHANGE(spi, (const void*)&read_cmd, &read_id, sizeof(read_cmd));
  SPI_SELECT(spi, SPIDEV_FLASH, FALSE);
  SPI_LOCK(spi, FALSE);
  nsh_output(vtbl, "SPI Flash id is %x %x %x %x \n", read_id[0], read_id[1], read_id[2], read_id[3]);
#endif

  return OK;

invalid_arg:
  nsh_output(vtbl, g_fmtarginvalid, argv[0]);
  return ERROR;

}
#endif /* CONFIG_SPI */

#include "command_console.h"
#include "wifi/command_console_wifi.h"
#include "platform/command_console_platform.h"

int cmd_wiced_console(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  static bool inited = false;

  if (!inited)
  {
    static const command_t commands[] =
    {
      WIFI_COMMANDS
      PLATFORM_COMMANDS
      CMD_TABLE_END
    };

    command_console_init(WICED_UART_MAX, 0, NULL, 0, NULL, " ");
    console_add_cmd_table(commands);

    inited = true;
  }

  console_parse_cmd(argv[1]);

  return OK;
}

#endif /* CONFIG_ARCH_CHIP_BCM4390X */
