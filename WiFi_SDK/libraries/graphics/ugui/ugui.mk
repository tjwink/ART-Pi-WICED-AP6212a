#
# $ Copyright Broadcom Corporation $
#

NAME := Lib_ugui

GLOBAL_INCLUDES := .

$(NAME)_SOURCES := ugui.c \
		   ugui_bus_i2c.c ugui_bus_rs232.c ugui_bus_spi.c ugui_bus.c \
		   ugui_driver_digole.c ugui_driver_ssd1306.c \
		   ugui_driver_virtual.c ugui_driver.c

KEEP_LIST := ugui.c ugui.h ugui_bus.h ugui_driver.h