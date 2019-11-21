#
# $ Copyright Broadcom Corporation $
#

NAME               := Lib_UPNP

IXML_SOURCES       := \
					ixml/src/attr.c \
					ixml/src/document.c \
					ixml/src/element.c \
					ixml/src/ixml.c \
					ixml/src/ixmldebug.c \
					ixml/src/ixmlparser.c \
					ixml/src/ixmlmembuf.c \
					ixml/src/namedNodeMap.c \
					ixml/src/node.c \
					ixml/src/nodeList.c

THREADUTIL_SOURCES := \
					threadutil/src/FreeList.c \
					threadutil/src/LinkedList.c \
					threadutil/src/ThreadPool.c \
					threadutil/src/TimerThread.c
					
UPNP_SOURCES       := \
					upnp/src/ssdp/ssdp_device.c \
					upnp/src/ssdp/ssdp_ctrlpt.c \
					upnp/src/ssdp/ssdp_server.c \
					upnp/src/soap/soap_device.c \
					upnp/src/soap/soap_ctrlpt.c \
					upnp/src/soap/soap_common.c \
					upnp/src/genlib/miniserver/wiced_miniserver.c \
					upnp/src/genlib/service_table/service_table.c \
					upnp/src/genlib/util/membuffer.c \
					upnp/src/genlib/util/strintmap.c \
					upnp/src/genlib/util/upnp_timeout.c \
					upnp/src/genlib/util/util.c \
					upnp/src/genlib/client_table/client_table.c \
					upnp/src/genlib/net/sock.c \
					upnp/src/genlib/net/http/httpparser.c \
					upnp/src/genlib/net/http/httpreadwrite.c \
					upnp/src/genlib/net/http/statcodes.c \
					upnp/src/genlib/net/http/parsetools.c \
					upnp/src/genlib/net/http/webserver.c \
					upnp/src/genlib/net/uri/uri.c \
					upnp/src/gena/gena_device.c \
					upnp/src/gena/gena_ctrlpt.c \
					upnp/src/gena/gena_callback2.c \
					upnp/src/api/upnpdebug.c \
					upnp/src/api/UpnpString.c \
					upnp/src/api/upnpapi.c \
					upnp/src/api/upnptools.c \
					upnp/src/urlconfig/urlconfig.c \
					upnp/src/uuid/md5.c \
					upnp/src/uuid/sysdep.c \
					upnp/src/uuid/uuid.c \
					upnp/src/inet_pton.c

$(NAME)_SOURCES    := $(IXML_SOURCES) $(THREADUTIL_SOURCES) $(UPNP_SOURCES)

$(NAME)_INCLUDES   := \
					. \
					ixml/src/inc \
					upnp/src/inc

$(NAME)_CFLAGS     += -std=gnu99

GLOBAL_INCLUDES    := \
                    . \
					ixml/inc \
					upnp/inc \
					threadutil/inc

GLOBAL_DEFINES     := WICED

$(NAME)_COMPONENTS += utilities/linked_list

KEEP_LIST          := *