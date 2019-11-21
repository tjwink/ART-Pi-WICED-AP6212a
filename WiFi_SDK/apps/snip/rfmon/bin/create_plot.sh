#!/bin/bash
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
 # so agrees to indemnify Cypress against all liability.$
#
# Usage: total data_file ["Title"]
# Create plots based on data from rfmon.
#
# I usually put BSS type (adhoc or infra) as part of title
# Resulting .png lot will go into a 'plots' directory.

TMPFILE="c:/Temp/pass_into_gnuplot"

#Create a gnuplot from rfmon data
if [ ! -f "$1" ]; then
    echo "No such file $1"
    exit
fi

#Plot both combined and individual PER plots
`dirname $0`/create_metadata.sh  0 $1 0 "$2" > $TMPFILE
/cygdrive/c/Program\ Files\ \(x86\)/gnuplot/bin/wgnuplot  $TMPFILE

`dirname $0`/create_metadata.sh  0 $1 1 "$2" > $TMPFILE
/cygdrive/c/Program\ Files\ \(x86\)/gnuplot/bin/wgnuplot  $TMPFILE

#Create individual (multiplot) SLCPLC & FEC plot
`dirname $0`/create_metadata.sh  1 $1 1 "$2" > $TMPFILE
/cygdrive/c/Program\ Files\ \(x86\)/gnuplot/bin/wgnuplot  $TMPFILE
