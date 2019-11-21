/**************************************************************************
 *
 * Copyright (c) 2000-2003 Intel Corporation 
 * All rights reserved. 
 * Copyright (c) 2012 France Telecom All rights reserved. 
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 *
 * - Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer. 
 * - Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution. 
 * - Neither name of Intel Corporation nor the names of its contributors 
 * may be used to endorse or promote products derived from this software 
 * without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL INTEL OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

/*!
 * \addtogroup Sock
 * 
 * @{
 * 
 * \file
 *
 * \brief Implements the sockets functionality.
 */

#include "config.h"

#include "sock.h"

#include "unixutil.h"	/* for socklen_t, EAFNOSUPPORT */
#include "upnp.h"
#include "UpnpStdInt.h" /* for ssize_t */

#include "upnpdebug.h"
#include "upnputil.h"

#include <assert.h>
#ifndef WICED
#include <errno.h>
#include <fcntl.h>	/* for F_GETFL, F_SETFL, O_NONBLOCK */
#include <time.h>
#endif /* WICED */
#include <string.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define DEFAULT_TCP_RECEIVE_DELAY_MSECS (10)

int sock_init(SOCKINFO *info, SOCKET sockfd)
{
	assert(info);

	if ( sockfd == INVALID_SOCKET || !SOCKET_PTR_VALID(sockfd) )
	{
	    return UPNP_E_INVALID_PARAM;
	}

	memset(info, 0, sizeof(SOCKINFO));
	info->socket = sockfd;

	if ( GET_SOCKET_TYPE(info->socket) == UPNP_SOCKET_TCP )
	{
	    if ( wiced_tcp_stream_init(GET_SOCKET_STREAM(info->socket), GET_TCP_SOCKET(info->socket)) != WICED_SUCCESS )
	    {
	        CLEAR_SOCKET_STREAM_PTR(info->socket);
	        UpnpPrintf(UPNP_CRITICAL, HTTP, __FILE__, __LINE__, "[PACO]wiced_tcp_stream_init() failed !\n");
	        return UPNP_E_SOCKET_ERROR;
	    }
	    else
	    {
	        SET_SOCKET_STREAM_PTR(info->socket);
	    }
	}

	return UPNP_E_SUCCESS;
}

int sock_init_with_ip(SOCKINFO *info, SOCKET sockfd,
#ifdef WICED
     wiced_ip_address_with_port_t *foreign_sockaddr
#else /* !WICED */
	struct sockaddr *foreign_sockaddr
#endif /* WICED */
	)
{
	int ret;

	ret = sock_init(info, sockfd);
	if (ret != UPNP_E_SUCCESS) {
		return ret;
	}

#ifdef WICED
	memcpy(&info->foreign_sockaddr, foreign_sockaddr,
	       sizeof(wiced_ip_address_with_port_t));
#else /* !WICED */
	memcpy(&info->foreign_sockaddr, foreign_sockaddr,
	       sizeof(struct sockaddr));
#endif /* WICED */

	return UPNP_E_SUCCESS;
}

#ifdef WICED
int sock_disconnect(SOCKINFO *info)
{
    int ret = UPNP_E_SOCKET_ERROR;

    if ((info != NULL) && (info->socket != INVALID_SOCKET) && SOCKET_PTR_VALID(info->socket))
    {
        if ( GET_SOCKET_TYPE(info->socket) == UPNP_SOCKET_TCP )
        {
            wiced_result_t result;

            if ( SOCKET_STREAM_PTR_VALID(info->socket) )
            {
                wiced_tcp_stream_deinit(GET_SOCKET_STREAM(info->socket));
                CLEAR_SOCKET_STREAM_PTR(info->socket);
            }
            result = wiced_tcp_disconnect(GET_TCP_SOCKET(info->socket));
            if ( result != WICED_SUCCESS )
            {
                UpnpPrintf(UPNP_INFO, HTTP, __FILE__, __LINE__,
                           "Error in wiced_tcp_disconnect(): %d\n", (int)result);
            }
            else
            {
                ret = UPNP_E_SUCCESS;
            }
        }
    }

    return ret;
}
#endif /* WICED */

int sock_destroy(SOCKINFO *info, int ShutdownMethod)
{
	int ret = UPNP_E_SUCCESS;

#ifdef WICED
	UNUSED_PARAMETER(ShutdownMethod);

	if ((info->socket != INVALID_SOCKET) && SOCKET_PTR_VALID(info->socket))
	{
	    if ( GET_SOCKET_TYPE(info->socket) == UPNP_SOCKET_TCP )
	    {
	        wiced_result_t result;

	        result = wiced_tcp_disconnect(GET_TCP_SOCKET(info->socket));
	        if ( result != WICED_SUCCESS )
	        {
	            UpnpPrintf(UPNP_INFO, HTTP, __FILE__, __LINE__,
	                       "Error in wiced_tcp_disconnect(): %d\n", (int)result);
	        }
	    }
	    if (sock_close(info->socket) == -1)
	    {
	        ret = UPNP_E_SOCKET_ERROR;
	    }
	    info->socket = INVALID_SOCKET;
	}
#else /* !WICED */
	char errorBuffer[ERROR_BUFFER_LEN];

	if (info->socket != INVALID_SOCKET) {
		if (shutdown(info->socket, ShutdownMethod) == -1) {
			strerror_r(errno, errorBuffer, ERROR_BUFFER_LEN);
			UpnpPrintf(UPNP_INFO, HTTP, __FILE__, __LINE__,
				   "Error in shutdown: %s\n", errorBuffer);
		}
		if (sock_close(info->socket) == -1) {
			ret = UPNP_E_SOCKET_ERROR;
		}
		info->socket = INVALID_SOCKET;
	}
#endif /* WICED */

	return ret;
}

/*!
 * \brief Receives or sends data. Also returns the time taken to receive or
 * send data.
 *
 * \return
 *	\li \c numBytes - On Success, no of bytes received or sent or
 *	\li \c UPNP_E_TIMEDOUT - Timeout
 *	\li \c UPNP_E_SOCKET_ERROR - Error on socket calls
 */
#ifdef WICED
static int sock_read_write(
    /*! [in] Socket Information Object. */
    SOCKINFO *info,
    /*! [out] Buffer to get data to or send data from. */
    char *buffer,
    /*! [in] Size of the buffer. */
    size_t bufsize,
    /*! [in] timeout value. */
    int *timeoutSecs,
    /*! [in] Boolean value specifying read or write option. */
    int bRead)
{
    int                rc;
    wiced_result_t     result;
    uint32_t           tick;
    uint32_t           tack;
    uint32_t           byte_count;
    int                runtime_in_secs;

    if (*timeoutSecs < 0)
    {
        rc = UPNP_E_TIMEDOUT;
        goto _exit;
    }

    if ( GET_SOCKET_TYPE(info->socket) != UPNP_SOCKET_TCP )
    {
        rc = UPNP_E_SOCKET_ERROR;
        UpnpPrintf(UPNP_CRITICAL, HTTP, __FILE__, __LINE__, "[PACO]sock_read_write() NOT a TCP socket !\n");
        goto _exit;
    }

    wiced_time_get_time(&tick);

    if (bRead != 0)
    {
        byte_count = 0;

        do
        {
            result = wiced_tcp_stream_read_with_count( GET_SOCKET_STREAM(info->socket), buffer, (uint16_t)bufsize, WICED_NO_WAIT, &byte_count);
            if ( result != WICED_TCPIP_SUCCESS )
            {
                if ( *timeoutSecs == 0 )
                {
                    rc = UPNP_E_SOCKET_ERROR;
                    UpnpPrintf(UPNP_CRITICAL, HTTP, __FILE__, __LINE__, "[PACO] wiced_tcp_stream_read_with_count() failed NO WAIT, byte_count=%lu !\n", byte_count);
                    goto _exit;
                }
                else
                {
                    wiced_rtos_delay_milliseconds(DEFAULT_TCP_RECEIVE_DELAY_MSECS);
                    wiced_time_get_time(&tack);
                    if ( (tack - tick) >= (uint32_t)((*timeoutSecs) * 1000) )
                    {
                        rc = UPNP_E_SOCKET_ERROR;
                        UpnpPrintf(UPNP_CRITICAL, HTTP, __FILE__, __LINE__, "[PACO] wiced_tcp_stream_read_with_count() TIMED OUT, byte_count=%lu !\n", byte_count);
                        break;
                    }
                }
            }
            else
            {
                rc = byte_count;
                break;
            }
        } while (WICED_TRUE);

    }
    else
    {
        result = wiced_tcp_stream_write( GET_SOCKET_STREAM(info->socket), buffer, (uint32_t)bufsize);
        wiced_tcp_stream_flush( GET_SOCKET_STREAM(info->socket) );
        if ( result != WICED_TCPIP_SUCCESS )
        {
            rc = UPNP_E_SOCKET_ERROR;
            UpnpPrintf(UPNP_CRITICAL, HTTP, __FILE__, __LINE__, "[PACO] wiced_tcp_stream_write() failed !\n");
            goto _exit;
        }
        rc = bufsize;
    }

    wiced_time_get_time(&tack);
    runtime_in_secs = (tack - tick) / 1000;
    if ( *timeoutSecs != 0 )
    {
        *timeoutSecs -= runtime_in_secs;
    }

 _exit:
    return rc;
}
#else /* !WICED */
static int sock_read_write(
	/*! [in] Socket Information Object. */
	SOCKINFO *info,
	/*! [out] Buffer to get data to or send data from. */
	char *buffer,
	/*! [in] Size of the buffer. */
	size_t bufsize,
	/*! [in] timeout value. */
	int *timeoutSecs,
	/*! [in] Boolean value specifying read or write option. */
	int bRead)
{
	int retCode;
	fd_set readSet;
	fd_set writeSet;
	struct timeval timeout;
	long numBytes;
	time_t start_time = time(NULL);
	SOCKET sockfd = info->socket;
	long bytes_sent = 0;
	size_t byte_left = (size_t)0;
	ssize_t num_written;

	if (*timeoutSecs < 0)
		return UPNP_E_TIMEDOUT;
	FD_ZERO(&readSet);
	FD_ZERO(&writeSet);
	if (bRead)
		FD_SET(sockfd, &readSet);
	else
		FD_SET(sockfd, &writeSet);
	timeout.tv_sec = *timeoutSecs;
	timeout.tv_usec = 0;
	while (TRUE) {
		if (*timeoutSecs == 0)
			retCode = select(sockfd + 1, &readSet, &writeSet,
				NULL, NULL);
		else
			retCode = select(sockfd + 1, &readSet, &writeSet,
				NULL, &timeout);
		if (retCode == 0)
			return UPNP_E_TIMEDOUT;
		if (retCode == -1) {
			if (errno == EINTR)
				continue;
			return UPNP_E_SOCKET_ERROR;
		} else
			/* read or write. */
			break;
	}
#ifdef SO_NOSIGPIPE
	{
		int old;
		int set = 1;
		socklen_t olen = sizeof(old);
		getsockopt(sockfd, SOL_SOCKET, SO_NOSIGPIPE, &old, &olen);
		setsockopt(sockfd, SOL_SOCKET, SO_NOSIGPIPE, &set, sizeof(set));
#endif
		if (bRead) {
			/* read data. */
			numBytes = (long)recv(sockfd, buffer, bufsize, MSG_NOSIGNAL);
		} else {
			byte_left = bufsize;
			bytes_sent = 0;
			while (byte_left != (size_t)0) {
				/* write data. */
				num_written = send(sockfd,
					buffer + bytes_sent, byte_left,
					MSG_DONTROUTE | MSG_NOSIGNAL);
				if (num_written == -1) {
#ifdef SO_NOSIGPIPE
					setsockopt(sockfd, SOL_SOCKET,
						SO_NOSIGPIPE, &old, olen);
#endif
					return (int)num_written;
				}
				byte_left -= (size_t)num_written;
				bytes_sent += num_written;
			}
			numBytes = bytes_sent;
		}
#ifdef SO_NOSIGPIPE
		setsockopt(sockfd, SOL_SOCKET, SO_NOSIGPIPE, &old, olen);
	}
#endif
	if (numBytes < 0)
		return UPNP_E_SOCKET_ERROR;
	/* subtract time used for reading/writing. */
	if (*timeoutSecs != 0)
		*timeoutSecs -= (int)(time(NULL) - start_time);

	return (int)numBytes;
}
#endif /* WICED */

int sock_read(SOCKINFO *info, char *buffer, size_t bufsize, int *timeoutSecs)
{
	return sock_read_write(info, buffer, bufsize, timeoutSecs, TRUE);
}

int sock_write(SOCKINFO *info, const char *buffer, size_t bufsize, int *timeoutSecs)
{
	/* Consciently removing constness. */
	return sock_read_write(info, (char *)buffer, bufsize, timeoutSecs, FALSE);
}

int sock_make_blocking(SOCKET sock)
{
#ifdef WIN32
	u_long val = 0;
	return ioctlsocket(sock, FIONBIO, &val);
#elif defined(WICED)
	UNUSED_PARAMETER(sock);
#else
	int val;

	val = fcntl(sock, F_GETFL, 0);
	if (fcntl(sock, F_SETFL, val & ~O_NONBLOCK) == -1) {
		return -1;
	}
#endif
	return 0;
}


int sock_make_no_blocking(SOCKET sock)
{
#ifdef WIN32
	u_long val = 1;
	return ioctlsocket(sock, FIONBIO, &val);
#elif defined(WICED)
	UNUSED_PARAMETER(sock);
#else /* WIN32 */
	int val;

	val = fcntl(sock, F_GETFL, 0);
	if (fcntl(sock, F_SETFL, val | O_NONBLOCK) == -1) {
		return -1;
	}
#endif /* WIN32 */
	return 0;
}

/* @} Sock */
