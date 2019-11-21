#
# $ Copyright Broadcom Corporation $
#

NAME := Lib_MQTT_Client

$(NAME)_SOURCES :=  mqtt_network.c  \
                    mqtt_frame.c    \
                    mqtt_connection.c \
                    mqtt_manager.c  \
                    mqtt_session.c \
                    mqtt_api.c
#make it visible for the applications which take advantage of this lib
GLOBAL_INCLUDES := .


$(NAME)_CFLAGS  = $(COMPILER_SPECIFIC_PEDANTIC_CFLAGS)