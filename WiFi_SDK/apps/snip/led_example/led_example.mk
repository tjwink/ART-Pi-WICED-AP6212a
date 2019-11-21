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

NAME := App_LED_Example

$(NAME)_SOURCES := led_example.c

$(NAME)_COMPONENTS += utilities/command_console 	 \
                      daemons/led_service

VALID_PLATFORMS	:= BCM943340WCD1 BCM943341WAE BCM943341WCD1 BCM943362WCD4
VALID_PLATFORMS	+= BCM943362WCD6 BCM943362WCD8 BCM943364WCD1 BCM943438WCD1
VALID_PLATFORMS	+= CYW94343WWCD1_EVB BCM94343WWCD2 NEB1DX* BCM943903PS BCM943903WCD1_1 BCM943907WAE_1 BCM943907WAE2_1 CYW943907WAE*
VALID_PLATFORMS	+= BCM94390WCD1 BCM94390WCD2 BCM94390WCD3 BCM9WCDUSI09 STMDiscovery411_BCM43438 CYW943907AEVAL1F Quicksilver_EVL

