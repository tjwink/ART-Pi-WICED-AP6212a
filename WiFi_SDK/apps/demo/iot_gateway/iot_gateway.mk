#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#

NAME := App_Iot_Gateway

########		cloud hosts		#############
#	CLOUD_HOST_EXOSITE
#	CLOUD_HOST_CARRIOTS
#
# default cloud host is Exosite.
#############################################
#CLOUDHOST                                     := CLOUD_HOST_CARRIOTS


########		cloud protocols		##########
#	CLOUD_PROTO_COAP
#	CLOUD_PROTO_HTTP
#	CLOUD_PROTO_HTTPS
#
# default cloud protocol is CLOUD_PROTO_HTTP
##############################################
#CLOUD_PROTOCOL                                := CLOUD_PROTO_COAP
#CLOUD_PROTOCOL                                := CLOUD_PROTO_HTTPS

###########	Global Definitions  ##############
# NO_AP_INTERFACE
##############################################
GLOBAL_DEFINES		+= DISABLE_WIFI_AP_INTERFACE

GLOBAL_DEFINES += WICED_CONFIG_DISABLE_SSL_SERVER \
                  WICED_CONFIG_DISABLE_DTLS \
                  WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY \
                  WICED_CONFIG_DISABLE_DES \
                  WICED_CONFIG_DISABLE_ADVANCED_SECURITY_CURVES

$(NAME)_SOURCES := gateway.c \
                   wiced_bt_cfg.c

ifeq ($(CLOUD_PROTOCOL),CLOUD_PROTO_COAP)

$(NAME)_SOURCES    += gateway_coap.c
GLOBAL_DEFINES     += CLOUD_PROTO_COAP
GLOBAL_DEFINES     += WICED_DISABLE_TLS
$(NAME)_COMPONENTS := protocols/COAP

else

$(NAME)_SOURCES += gateway_http.c
$(NAME)_COMPONENTS := protocols/HTTP_client \
                      utilities/JSON_parser

GLOBAL_DEFINES     += CLOUD_PROTO_HTTP

ifeq ($(CLOUD_PROTOCOL),CLOUD_PROTO_HTTPS)
GLOBAL_DEFINES     += CLOUD_PROTO_HTTPS APPLICATION_STACK_SIZE=4*1024

#By default PLATFORM HEAP size is 30K. Reduce for non XIP cases
ifneq (1, $(XIP_SUPPORT))
GLOBAL_DEFINES     += PLATFORM_HEAP_SIZE=20*1024
endif

else
GLOBAL_DEFINES     += WICED_DISABLE_TLS
endif

endif


ifeq ($(CLOUDHOST),CLOUD_HOST_CARRIOTS)
GLOBAL_DEFINES	+= CLOUD_HOST_CARRIOTS
else
GLOBAL_DEFINES	+= CLOUD_HOST_EXOSITE
endif

# Enable this flag to test as a standalone app. No need of a client sensor device
#GLOBAL_DEFINES  += LOOP_TEST

APPLICATION_DCT := gateway_dct.c

USE_LIBC_PRINTF := 0

VALID_PLATFORMS := BCM920739B0_HB2 CYW9MCU7X9N364
