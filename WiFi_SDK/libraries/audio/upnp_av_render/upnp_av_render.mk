#
# $ Copyright Broadcom Corporation $
#
NAME               := Lib_UPnP_AV_render

$(NAME)_SOURCES    := \
						src/main.c \
						src/upnp.c \
						src/upnp_control.c \
						src/upnp_connmgr.c \
						src/upnp_transport.c \
						src/song-meta-data.c \
						src/variable-container.c \
						src/upnp_device.c \
						src/upnp_renderer.c \
						src/webserver.c \
						src/output.c \
						src/logging.c \
						src/xmldoc.c \
						src/xmlescape.c \
						src/output_wiced_audio.c

$(NAME)_INCLUDES   := \
						. \
						src

$(NAME)_DEFINES    := HAVE_CONFIG_H

$(NAME)_CFLAGS     += -std=gnu99

$(NAME)_COMPONENTS := protocols/UPNP
$(NAME)_COMPONENTS += audio/audio_client
$(NAME)_COMPONENTS += utilities/wiced_log

$(NAME)_RESOURCES  := \
						images/grender_128.png \
						images/grender_64.png

GLOBAL_INCLUDES    += .

KEEP_LIST          := *
