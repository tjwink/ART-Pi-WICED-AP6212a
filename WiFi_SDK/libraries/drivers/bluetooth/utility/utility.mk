#
# $ Copyright Broadcom Corporation $
#

NAME := Lib_Bluetooth_Dynamic_GATT_DB

GLOBAL_INCLUDES  := .

$(NAME)_SOURCES  := wiced_bt_gatt_db_helper.c

$(NAME)_COMPONENTS += drivers/bluetooth/low_energy