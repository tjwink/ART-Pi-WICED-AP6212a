#
# $ Copyright Broadcom Corporation $
#

NAME := Lib_micro_ecc

ifneq ($(WICED_SECURITY),ROM)
$(NAME)_SOURCES := uECC.c
endif

GLOBAL_INCLUDES := .

$(NAME)_ALWAYS_OPTIMISE :=1