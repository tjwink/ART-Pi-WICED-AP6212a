/**************************************************************************
 *
 * Copyright (c) 2000-2003 Intel Corporation
 * All rights reserved.
 * Copyright (C) 2012 France Telecom All rights reserved.
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

#include "config.h"

#if EXCLUDE_MINISERVER == 0

/*!
 * \file
 *
 * \brief Implements the functionality and utility functions
 * used by the Miniserver module.
 *
 * The miniserver is a central point for processing all network requests.
 * It is made of:
 *   - The SSDP sockets for discovery.
 *   - The HTTP listeners for description / control / eventing.
 *
 */

#include "miniserver.h"

#include "httpreadwrite.h"
#include "ithread.h"
#include "ssdplib.h"
#include "statcodes.h"
#include "ThreadPool.h"
#include "unixutil.h" /* for socklen_t, EAFNOSUPPORT */
#include "upnpapi.h"
#include "upnputil.h"

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

/*! . */
#define APPLICATION_LISTENING_PORT 49152
#define MAX_LISTENING_SOCKETS      5

struct mserv_request_t {
	/*! Connection handle. */
	SOCKET connfd;
	/*! . */
	wiced_ip_address_with_port_t foreign_sockaddr;
};

/*! . */
typedef enum {
	/*! . */
	MSERV_IDLE,
	/*! . */
	MSERV_RUNNING,
	/*! . */
	MSERV_STOPPING
} MiniServerState;

#define EVENT_QUEUE_DEPTH (20)

typedef enum
{
    MSERV_EVENT_NONE        = 0,
    MSERV_EVENT_CONNECT     = (1 << 0),
    MSERV_EVENT_RECEIVE_TCP = (1 << 1),
    MSERV_EVENT_DISCONNECT  = (1 << 2),
    MSERV_EVENT_RECEIVE_UDP = (1 << 3),
    MSERV_EVENT_STOP        = (1 << 4),

    MSERV_EVENT_ALL         = 0xFFFFFFFF
} miniserver_event_t;

typedef struct
{
    void*              socket;
    miniserver_event_t event_type;
} miniserver_event_message_t;

/*! . */
uint16_t miniStopSockPort;

/*!
 * module vars
 */
static MiniServerState    gMServState    = MSERV_IDLE;

static int                g_mserv_stop;
static wiced_tcp_server_t g_tcp_server;
static wiced_queue_t      g_server_queue;
static wiced_mutex_t      g_listen_mutex;
static uint32_t           g_listen_count;

static int get_listening_sockets
(
        uint16_t listen_port4, uint16_t *port4
#ifdef UPNP_ENABLE_IPV6
        ,uint16_t listen_port6, uint16_t *port6
#endif
);

#ifdef INTERNAL_WEB_SERVER
static MiniServerCallback gGetCallback = NULL;
static MiniServerCallback gSoapCallback = NULL;
static MiniServerCallback gGenaCallback = NULL;

void SetHTTPGetCallback(MiniServerCallback callback)
{
	gGetCallback = callback;
}

#ifdef INCLUDE_DEVICE_APIS
void SetSoapCallback(MiniServerCallback callback)
{
	gSoapCallback = callback;
}
#endif /* INCLUDE_DEVICE_APIS */

void SetGenaCallback(MiniServerCallback callback)
{
	gGenaCallback = callback;
}

/*!
 * \brief Based on the type pf message, appropriate callback is issued.
 *
 * \return 0 on Success or HTTP_INTERNAL_SERVER_ERROR if Callback is NULL.
 */
static int dispatch_request(
	/*! [in] Socket Information object. */
	IN SOCKINFO *info,
	/*! [in] HTTP parser object. */
	http_parser_t *hparser)
{
	MiniServerCallback callback;

	switch (hparser->msg.method) {
	/* Soap Call */
	case SOAPMETHOD_POST:
	case HTTPMETHOD_MPOST:
		callback = gSoapCallback;
		break;
	/* Gena Call */
	case HTTPMETHOD_NOTIFY:
	case HTTPMETHOD_SUBSCRIBE:
	case HTTPMETHOD_UNSUBSCRIBE:
	    UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
	               "miniserver %p: got GENA msg\n", info->socket);
	    callback = gGenaCallback;
		break;
	/* HTTP server call */
	case HTTPMETHOD_GET:
	case HTTPMETHOD_POST:
	case HTTPMETHOD_HEAD:
	case HTTPMETHOD_SIMPLEGET:
		callback = gGetCallback;
		break;
	default:
		callback = NULL;
	}
	if (callback == NULL) {
		return HTTP_INTERNAL_SERVER_ERROR;
	}
	callback(hparser, &hparser->msg, info);

	return 0;
}

/*!
 * \brief Send Error Message.
 */
static UPNP_INLINE void handle_error(
	/*! [in] Socket Information object. */
	SOCKINFO *info,
	/*! [in] HTTP Error Code. */
	int http_error_code,
	/*! [in] Major Version Number. */
	int major,
	/*! [in] Minor Version Number. */
	int minor)
{
	http_SendStatusResponse(info, http_error_code, major, minor);
}

/*!
 * \brief Free memory assigned for handling request and unitialize socket
 * functionality.
 */
static void free_handle_request_arg(
	/*! [in] Request Message to be freed. */
	void *args)
{
	struct mserv_request_t *request = (struct mserv_request_t *)args;
	sock_close(request->connfd);
	UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
	           "miniserver %p: free_handle_request_arg\n", request->connfd);
	free(request);
}

/*!
 * \brief Receive the request and dispatch it for handling.
 */
static void handle_request(
	/*! [in] Request Message to be handled. */
	void *args)
{
	SOCKINFO info;
	int http_error_code;
	int ret_code;
	int major = 1;
	int minor = 1;
	http_parser_t parser;
	http_message_t *hmsg = NULL;
	int timeout = HTTP_DEFAULT_TIMEOUT;
	struct mserv_request_t *request = (struct mserv_request_t *)args;
	SOCKET connfd = request->connfd;
	wiced_result_t result;

	UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
	           "miniserver %p: READING\n", connfd);
	/* parser_request_init( &parser ); */ /* LEAK_FIX_MK */
	hmsg = &parser.msg;
	ret_code = sock_init_with_ip(&info, connfd, &request->foreign_sockaddr);
	if (ret_code != UPNP_E_SUCCESS) {
		goto _exit;
	}
	/* read */
	ret_code = http_RecvMessage(
		&info, &parser, HTTPMETHOD_UNKNOWN, &timeout, &http_error_code);
	if (ret_code != 0) {
	    UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__, "miniserver %p: http_RecvMessage() failed !\n", connfd);
		goto error_handler;
	}

	/* UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
	              "miniserver %p: PROCESSING...\n", connfd); */

	/* dispatch */
	http_error_code = dispatch_request(&info, &parser);
	if (http_error_code != 0) {
		goto error_handler;
	}
	http_error_code = 0;

error_handler:
	if (http_error_code > 0) {
		if (hmsg) {
			major = hmsg->major_version;
			minor = hmsg->minor_version;
		}
		handle_error(&info, http_error_code, major, minor);
	}

	if ( SOCKET_STREAM_PTR_VALID(connfd) )
	{
	    wiced_tcp_stream_deinit(GET_SOCKET_STREAM(connfd));
	    CLEAR_SOCKET_STREAM_PTR(connfd);
	}
	result = wiced_tcp_server_disconnect_socket(&g_tcp_server, GET_TCP_SOCKET(connfd));
	if ( result != WICED_SUCCESS )
	{
	    UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
	               "miniserver: Error in wiced_tcp_server_disconnect(): %d\n", (int)result);
	}
	CLEAR_SOCKET_MAGIC(connfd);
	free(connfd);
	connfd = INVALID_SOCKET;

_exit:
	httpmsg_destroy(hmsg);
	free(request);

	UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
	           "miniserver: COMPLETE\n");

	wiced_rtos_lock_mutex(&g_listen_mutex);
	g_listen_count--;
	wiced_rtos_unlock_mutex(&g_listen_mutex);
}

/*!
 * \brief Initilize the thread pool to handle a request, sets priority for the
 * job and adds the job to the thread pool.
 */
static UPNP_INLINE void schedule_request_job(
	/*! [in] Socket Descriptor on which connection is accepted. */
	SOCKET connfd,
	/*! [in] Clients Address information. */
    wiced_ip_address_with_port_t *clientAddr
	)
{
	struct mserv_request_t *request;
	ThreadPoolJob job;

	memset(&job, 0, sizeof(job));

	request = (struct mserv_request_t *)malloc(
		sizeof (struct mserv_request_t));
	if (request == NULL) {
	    UpnpPrintf( UPNP_INFO, MSERV, __FILE__, __LINE__,
	                "mserv %p: out of memory\n", connfd);
		sock_close(connfd);
		return;
	}

	request->connfd = connfd;
	memcpy(&request->foreign_sockaddr, clientAddr,
		sizeof(request->foreign_sockaddr));
	TPJobInit(&job, (start_routine)handle_request, (void *)request);
	TPJobSetFreeFunction(&job, free_handle_request_arg);
	TPJobSetPriority(&job, MED_PRIORITY);
	if (ThreadPoolAdd(&gMiniServerThreadPool, &job, NULL) != 0) {
	    UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
	               "mserv %p: cannot schedule request\n", connfd);
		free(request);
		sock_close(connfd);
		return;
	}
	else
	{
	    wiced_rtos_lock_mutex(&g_listen_mutex);
	    g_listen_count++;
	    wiced_rtos_unlock_mutex(&g_listen_mutex);
	}
}
#endif /* INTERNAL_WEB_SERVER */

static void ssdp_multicast_leave(void)
{
    wiced_ip_address_t INITIALISER_IPV4_ADDRESS( mcast_addr, WICED_SSDP_IP );

    wiced_multicast_leave( g_IFACE, &mcast_addr );
}

static void ssdp_read(SOCKET rsock)
{
    if (rsock != INVALID_SOCKET)
    {
        readFromSSDPSocket(rsock);
    }
}

static void RunMiniServer(
    /*! [in] Socket Array. */
    MiniServerSockArray *miniSock)
{
    static const uint32_t      max_listen_wait = 50;
    uint32_t                   count;
    miniserver_event_message_t current_event;

    gMServState = MSERV_RUNNING;
    while (g_mserv_stop == 0)
    {
        if ( wiced_rtos_pop_from_queue( &g_server_queue, &current_event, WICED_WAIT_FOREVER) != WICED_SUCCESS )
        {
            UpnpPrintf( UPNP_PACKET, MSERV, __FILE__, __LINE__, "wiced_rtos_pop_from_queue() failed !\n");
            continue;
        }

        if (g_mserv_stop != 0)
        {
            break;
        }

        switch (current_event.event_type)
        {
            case MSERV_EVENT_RECEIVE_UDP:
                ssdp_read((SOCKET)current_event.socket);
                break;

            default:
                break;
        }
    }

    wiced_udp_unregister_callbacks(GET_UDP_SOCKET(miniSock->ssdpSock4));
#ifdef UPNP_ENABLE_IPV6
    wiced_udp_unregister_callbacks(GET_UDP_SOCKET(miniSock->ssdpSock6));
    wiced_udp_unregister_callbacks(GET_UDP_SOCKET(miniSock->ssdpSock6UlaGua));
#endif /* UPNP_ENABLE_IPV6    */
#ifdef INCLUDE_CLIENT_APIS
    wiced_udp_unregister_callbacks(GET_UDP_SOCKET(miniSock->ssdpReqSock4));
#ifdef UPNP_ENABLE_IPV6
    wiced_udp_unregister_callbacks(GET_UDP_SOCKET(miniSock->ssdpReqSock6));
#endif /* UPNP_ENABLE_IPV6    */
#endif /* INCLUDE_CLIENT_APIS */

    /*
     * Wait for listening socket processing
     */

    count = 0;
    while ((g_listen_count > 0) && (count < max_listen_wait))
    {
        imillisleep(100U);
        count++;
    }

    /* Close all sockets. */
    wiced_tcp_server_stop(&g_tcp_server);
    sock_close(miniSock->miniServerSock4);
    sock_close(miniSock->miniServerSock6);
    sock_close(miniSock->miniServerStopSock);
    sock_close(miniSock->ssdpSock4);
    sock_close(miniSock->ssdpSock6);
    sock_close(miniSock->ssdpSock6UlaGua);
#ifdef INCLUDE_CLIENT_APIS
    sock_close(miniSock->ssdpReqSock4);
    sock_close(miniSock->ssdpReqSock6);
#endif /* INCLUDE_CLIENT_APIS */
    /* Free minisock. */
    free(miniSock);
    /* Leave SSDP multicast group */
    ssdp_multicast_leave();
    gMServState = MSERV_IDLE;

    return;
}


#ifdef INTERNAL_WEB_SERVER
/*!
 * \brief Creates a STREAM socket, binds to INADDR_ANY and listens for
 * incoming connecttions. Returns the actual port which the sockets
 * sub-system returned. 
 *
 * Also creates a DGRAM socket, binds to the loop back address and 
 * returns the port allocated by the socket sub-system.
 *
 * \return
 *	\li UPNP_E_OUTOF_SOCKET: Failed to create a socket.
 *	\li UPNP_E_SOCKET_BIND: Bind() failed.
 *	\li UPNP_E_LISTEN: Listen() failed.	
 *	\li UPNP_E_INTERNAL_ERROR: Port returned by the socket layer is < 0.
 *	\li UPNP_E_SUCCESS: Success.
 */

static wiced_result_t mserv_wiced_tcp_connect_cbf( wiced_tcp_socket_t* socket, void* arg )
{
    wiced_result_t result = WICED_ERROR;

    /* UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__, "miniserver: mserv_wiced_tcp_connect_cbf(arg=%p)\n", arg); */

    if (g_mserv_stop != 0)
    {
        goto _exit;
    }

    result = wiced_tcp_server_accept( (wiced_tcp_server_t*)arg, socket );
    if ( result != WICED_SUCCESS )
    {
        UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
                   "miniserver: Error in wiced_tcp_server_accept() %d\n",
                   (int)result);
    }
    else
    {
        wiced_ip_address_with_port_t peer_address;

        result = wiced_tcp_server_peer( socket, &peer_address.addr, &peer_address.port );
        if ( result != WICED_TCPIP_SUCCESS )
        {
            UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
                       "miniserver: Error in wiced_tcp_server_peer() %d\n",
                       (int)result);
        }
        else
        {
            if ( peer_address.addr.version != WICED_IPV4 )
            {
                UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
                           "miniserver: wiced_tcp_server_accept() returns non-V4 IP address: %d\n",
                           (int)peer_address.addr.version);
                result = WICED_ERROR;
            }
            else
            {
                SOCKET listenfd4;

                /* UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
                              "miniserver: wiced_tcp_server_peer() returns IP=0x%08lx with port=%hu\n",
                              peer_address.addr.ip.v4, peer_address.port); */

                peer_address.addr.ip.v4 = htonl(peer_address.addr.ip.v4);

                listenfd4 = calloc(1, sizeof(wiced_upnp_socket_t));
                if (listenfd4 == INVALID_SOCKET)
                {
                    UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__, "miniserver: calloc(listenfd4) failed !\n");
                    result = WICED_OUT_OF_HEAP_SPACE;
                    goto _exit;
                }
                SET_SOCKET_MAGIC(listenfd4);
                SET_TCP_SOCKET_TYPE(listenfd4);
                /*
                 * wiced_tcp_server_start() creates its own TCP sockets; we just need to use them
                 */
                SET_EXT_TCP_SOCKET_PTR(listenfd4, socket);

                schedule_request_job(listenfd4, &peer_address);
            }
        }
    }

  _exit:
    return result;
}


static wiced_result_t mserv_wiced_tcp_receive_cbf( wiced_tcp_socket_t* socket, void* arg )
{
    UNUSED_PARAMETER(socket);
    UNUSED_PARAMETER(arg);
    /* UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__, "miniserver: mserv_wiced_tcp_receive_cbf(arg=%p)\n", arg); */
    return WICED_SUCCESS;
}


static wiced_result_t mserv_wiced_tcp_disconnect_cbf( wiced_tcp_socket_t* socket, void* arg )
{
    wiced_result_t result = WICED_SUCCESS;

    /* UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__, "miniserver: mserv_wiced_tcp_disconnect_cbf(arg=%p)\n", arg); */
    result = wiced_tcp_server_disconnect_socket_with_timeout( (wiced_tcp_server_t*)arg, socket, WICED_NO_WAIT);
    if ( result != WICED_SUCCESS )
    {
        UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
                   "miniserver: Error in wiced_tcp_server_disconnect(): %d\n", (int)result);
    }
    return result;
}


int get_listening_sockets
(
        uint16_t listen_port4, uint16_t *port4
#ifdef UPNP_ENABLE_IPV6
        ,uint16_t listen_port6, uint16_t *port6
#endif
)
{
    int            rc                 = UPNP_E_SUCCESS;
    uint16_t       orig_listen_port4  = listen_port4;
    wiced_result_t result;

#ifdef UPNP_ENABLE_IPV6
    rc = UPNP_E_LISTEN;
    goto _exit;
#endif

    /* As per the IANA specifications for the use of ports by applications
     * override the listen port passed in with the first available. */
    if (listen_port4 < APPLICATION_LISTENING_PORT) {
        listen_port4 = (uint16_t)APPLICATION_LISTENING_PORT;
    }

    do
    {
        result = wiced_tcp_server_start( &g_tcp_server, g_IFACE, listen_port4, MAX_LISTENING_SOCKETS,
                                         mserv_wiced_tcp_connect_cbf,
                                         mserv_wiced_tcp_receive_cbf,
                                         mserv_wiced_tcp_disconnect_cbf,
                                         &g_tcp_server );
        if ( result != WICED_SUCCESS )
        {
            UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
                       "miniserver: wiced_tcp_server_start() failed with %d, on port %hu\n",
                       (int)result, listen_port4);
            listen_port4++;
        }
    } while ( (result != WICED_SUCCESS) && (listen_port4 >= orig_listen_port4) );

    if (result != WICED_SUCCESS)
    {
        UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
                   "miniserver: error in wiced_tcp_server_start(), %d\n",
                   (int)result);
        rc = UPNP_E_LISTEN;
        goto _exit;
    }
    else
    {
        UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__,
                   "miniserver: wiced_tcp_server_start() reported success on port %hu\n",
                   listen_port4);
    }

    if (port4 != NULL)
    {
        *port4     = listen_port4;
    }

 _exit:
    return rc;
}

static int get_miniserver_sockets(
    /*! [in] Socket Array. */
    MiniServerSockArray *out,
    /*! [in] port on which the server is listening for incoming IPv4
     * connections. */
    uint16_t listen_port4
#ifdef UPNP_ENABLE_IPV6
    ,
    /*! [in] port on which the server is listening for incoming IPv6
     * connections. */
    uint16_t listen_port6
#endif
    )
{
    int rc;

    rc = get_listening_sockets(
             listen_port4, &(out->miniServerPort4)
#ifdef UPNP_ENABLE_IPV6
            ,listen_port6, &(out->miniServerPort6)
#endif
    );

    return rc;
}
#endif /* INTERNAL_WEB_SERVER */


static UPNP_INLINE void InitMiniServerSockArray(MiniServerSockArray *miniSocket)
{
    miniSocket->miniServerSock4 = INVALID_SOCKET;
    miniSocket->miniServerSock6 = INVALID_SOCKET;
    miniSocket->miniServerStopSock = INVALID_SOCKET;
    miniSocket->ssdpSock4 = INVALID_SOCKET;
    miniSocket->ssdpSock6 = INVALID_SOCKET;
    miniSocket->ssdpSock6UlaGua = INVALID_SOCKET;
    miniSocket->stopPort = 0u;
    miniSocket->miniServerPort4 = 0u;
    miniSocket->miniServerPort6 = 0u;
#ifdef INCLUDE_CLIENT_APIS
    miniSocket->ssdpReqSock4 = INVALID_SOCKET;
    miniSocket->ssdpReqSock6 = INVALID_SOCKET;
#endif /* INCLUDE_CLIENT_APIS */
}

static wiced_result_t mserv_wiced_udp_cbf( wiced_udp_socket_t* socket, void* arg )
{
    wiced_result_t             result        = WICED_SUCCESS;
    miniserver_event_message_t current_event;

    UNUSED_PARAMETER(socket);
    current_event.event_type = MSERV_EVENT_RECEIVE_UDP;
    current_event.socket     = arg;
    result = wiced_rtos_push_to_queue( &g_server_queue, &current_event, WICED_NO_WAIT );
    return result;
}

int StartMiniServer(
    /*! [in,out] Port on which the server listens for incoming IPv4
     * connections. */
    uint16_t *listen_port4,
    /*! [in,out] Port on which the server listens for incoming IPv6
     * connections. */
    uint16_t *listen_port6)
{
    static const int max_count = 10000;
    int ret_code;
    int count;
    MiniServerSockArray *miniSocket;
    ThreadPoolJob job;

    memset(&job, 0, sizeof(job));

    switch (gMServState)
    {
        case MSERV_IDLE:
            break;
        default:
            /* miniserver running. */
            return UPNP_E_INTERNAL_ERROR;
    }

    g_mserv_stop   = 0;
    g_listen_count = 0;

    if ( wiced_rtos_init_mutex(&g_listen_mutex) != WICED_SUCCESS )
    {
        UpnpPrintf( UPNP_CRITICAL, MSERV, __FILE__, __LINE__, "wiced_rtos_init_mutex() failed !\n");
        return UPNP_E_INTERNAL_ERROR;
    }

    if ( wiced_rtos_init_queue(&g_server_queue, NULL, sizeof(miniserver_event_message_t), EVENT_QUEUE_DEPTH) != WICED_SUCCESS )
    {
        UpnpPrintf( UPNP_CRITICAL, MSERV, __FILE__, __LINE__, "wiced_rtos_init_queue() failed !\n");
        wiced_rtos_deinit_mutex(&g_listen_mutex);
        return UPNP_E_INTERNAL_ERROR;
    }

    miniSocket = (MiniServerSockArray *)malloc(sizeof (MiniServerSockArray));
    if (miniSocket == NULL)
    {
        UpnpPrintf(UPNP_CRITICAL, MSERV, __FILE__, __LINE__, "mserv start: malloc() failed !\n");
        wiced_rtos_deinit_mutex(&g_listen_mutex);
        wiced_rtos_deinit_queue(&g_server_queue);
        return UPNP_E_OUTOF_MEMORY;
    }
    InitMiniServerSockArray(miniSocket);
#ifdef INTERNAL_WEB_SERVER
    /* V4 and V6 http listeners. */
    ret_code = get_miniserver_sockets(
        miniSocket, *listen_port4
#ifdef UPNP_ENABLE_IPV6
        , *listen_port6
#endif
        );
    if (ret_code != UPNP_E_SUCCESS)
    {
        free(miniSocket);
        wiced_rtos_deinit_mutex(&g_listen_mutex);
        wiced_rtos_deinit_queue(&g_server_queue);
        return ret_code;
    }
#endif /* INTERNAL_WEB_SERVER */
    /* SSDP socket for discovery/advertising. */
    ret_code = get_ssdp_sockets(miniSocket);
    if (ret_code != UPNP_E_SUCCESS)
    {
        wiced_tcp_server_stop(&g_tcp_server);
        sock_close(miniSocket->miniServerSock4);
        sock_close(miniSocket->miniServerSock6);
        sock_close(miniSocket->miniServerStopSock);
        free(miniSocket);
        wiced_rtos_deinit_mutex(&g_listen_mutex);
        wiced_rtos_deinit_queue(&g_server_queue);
        return ret_code;
    }

    TPJobInit(&job, (start_routine)RunMiniServer, (void *)miniSocket);
    TPJobSetPriority(&job, MED_PRIORITY);
    TPJobSetFreeFunction(&job, (free_routine)free);
    ret_code = ThreadPoolAddPersistent(&gMiniServerThreadPool, &job, NULL);
    if (ret_code < 0)
    {
        wiced_tcp_server_stop(&g_tcp_server);
        sock_close(miniSocket->miniServerSock4);
        sock_close(miniSocket->miniServerSock6);
        sock_close(miniSocket->miniServerStopSock);
        sock_close(miniSocket->ssdpSock4);
        sock_close(miniSocket->ssdpSock6);
        sock_close(miniSocket->ssdpSock6UlaGua);
#ifdef INCLUDE_CLIENT_APIS
        sock_close(miniSocket->ssdpReqSock4);
        sock_close(miniSocket->ssdpReqSock6);
#endif /* INCLUDE_CLIENT_APIS */
        free(miniSocket);
        /* Leave SSDP multicast group */
        ssdp_multicast_leave();
        wiced_rtos_deinit_mutex(&g_listen_mutex);
        wiced_rtos_deinit_queue(&g_server_queue);
        return UPNP_E_OUTOF_MEMORY;
    }
    /* Wait for miniserver to start. */
    count = 0;
    while ((gMServState != MSERV_RUNNING) && (count < max_count))
    {
        /* 0.05s */
        imillisleep(50U);
        count++;
    }
    if (count >= max_count)
    {
        /* Took it too long to start that thread. */
        UpnpPrintf(UPNP_INFO, MSERV, __FILE__, __LINE__, "mserv start: mserv is not running !\n");
        wiced_tcp_server_stop(&g_tcp_server);
        sock_close(miniSocket->miniServerSock4);
        sock_close(miniSocket->miniServerSock6);
        sock_close(miniSocket->miniServerStopSock);
        sock_close(miniSocket->ssdpSock4);
        sock_close(miniSocket->ssdpSock6);
        sock_close(miniSocket->ssdpSock6UlaGua);
#ifdef INCLUDE_CLIENT_APIS
        sock_close(miniSocket->ssdpReqSock4);
        sock_close(miniSocket->ssdpReqSock6);
#endif /* INCLUDE_CLIENT_APIS */
        /* Leave SSDP multicast group */
        ssdp_multicast_leave();
        wiced_rtos_deinit_mutex(&g_listen_mutex);
        wiced_rtos_deinit_queue(&g_server_queue);
        return UPNP_E_INTERNAL_ERROR;
    }
#ifdef INTERNAL_WEB_SERVER
    *listen_port4 = miniSocket->miniServerPort4;
    *listen_port6 = miniSocket->miniServerPort6;
#endif /* INTERNAL_WEB_SERVER */

    if ( miniSocket->ssdpSock4 != INVALID_SOCKET )
    {
        wiced_udp_register_callbacks( GET_UDP_SOCKET(miniSocket->ssdpSock4), mserv_wiced_udp_cbf, miniSocket->ssdpSock4 );
    }
    if ( miniSocket->ssdpSock6 != INVALID_SOCKET )
    {
        wiced_udp_register_callbacks( GET_UDP_SOCKET(miniSocket->ssdpSock6), mserv_wiced_udp_cbf, miniSocket->ssdpSock6 );
    }
    if ( miniSocket->ssdpSock6UlaGua != INVALID_SOCKET )
    {
        wiced_udp_register_callbacks( GET_UDP_SOCKET(miniSocket->ssdpSock6UlaGua), mserv_wiced_udp_cbf, miniSocket->ssdpSock6UlaGua );
    }
#ifdef INCLUDE_CLIENT_APIS
    if ( miniSocket->ssdpReqSock4 != INVALID_SOCKET )
    {
        wiced_udp_register_callbacks( GET_UDP_SOCKET(miniSocket->ssdpReqSock4), mserv_wiced_udp_cbf, miniSocket->ssdpReqSock4 );
    }
    if ( miniSocket->ssdpReqSock6 != INVALID_SOCKET )
    {
        wiced_udp_register_callbacks( GET_UDP_SOCKET(miniSocket->ssdpReqSock6), mserv_wiced_udp_cbf, miniSocket->ssdpReqSock6 );
    }
#endif /* INCLUDE_CLIENT_APIS */

    return UPNP_E_SUCCESS;
}

int StopMiniServer()
{
    miniserver_event_message_t current_event;

    current_event.event_type = MSERV_EVENT_STOP;
    current_event.socket     = NULL;

    switch (gMServState)
    {
        case MSERV_RUNNING:
            gMServState = MSERV_STOPPING;
            break;
        default:
            return 0;
    }

    while (gMServState != MSERV_IDLE)
    {
        g_mserv_stop = 1;
        wiced_rtos_push_to_queue( &g_server_queue, &current_event, 10 );
    }

    wiced_rtos_deinit_queue(&g_server_queue);
    wiced_rtos_deinit_mutex(&g_listen_mutex);
    return 0;
}

#endif /* EXCLUDE_MINISERVER */
