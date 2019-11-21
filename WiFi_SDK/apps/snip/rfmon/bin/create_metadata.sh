#!/bin/bash

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
# Create metadata file for gnuplot
# Normally called by create_plot.sh script
# Usage:  create_metadata.sh  fec_mode  data_file  "Title"
# fec_mode: 1: Extract and plot SLCPLC & FEC data from data file.
#           0: Extract and plot PER data from data file.
# data_file: data file
# Title:  Default title is number of sinks.  This optional parameter is concatenated onto the default title.

TMPFILE="c:/Temp/renumbered"
OUTPUT_DIR=plots

if [ $1 -gt 0 ]; then
    fec=1   # Extract FEC/SLCPLC data
else
    fec=0   # Extract PER data
fi
dfile=$2    # Data file
multi=$3    # Multiplot or combined single plot
title=$4    # Title

# Create output dir if necessary
[ -d $OUTPUT_DIR ] || mkdir $OUTPUT_DIR

# Get list of peers from first line
peers=`cat $dfile | grep Time | awk '{print NF - 1}'`
format=`cat $dfile | grep Chan | cut -d':' -f 2`
channel=`cat $dfile | grep 'Cur channel'`
congest=`cat $dfile | grep 'Congestion Level'`
bsstype=`cat $dfile | grep 'BSS Type' | cut -d':' -f 2`

date=`date +'%D %T'`

# Read macaddrs into 'macs' array
IFS=' ' read -r -a macs <<< `cat $dfile | grep Time`

#First field is timestamp, renumber them so they start at 0 for a graph
#that starts at 0. (Maybe gnuplot can do this on its own?)
cat $dfile | grep '^00' | `dirname $0`/renumber.sh  | tr / ' ' > $TMPFILE

#Calculate mean for each peer
for ((i=1; i <= $peers; i++))
do
    if [ $fec -gt 0 ]; then
        col=$(((peers+2) + ((i-1)*2)))
        slc_mean[$i]=`cat $TMPFILE | awk -v col=$((col)) '{tot += $col} END {printf "%1.2f\n", tot/NR}'`
        fec_mean[$i]=`cat $TMPFILE | awk -v col=$((col+1)) '{tot += $col} END {printf "%1.2f\n", tot/NR}'`
        >&2 echo "Peer $i: slc ${slc_mean[i]} fec ${fec_mean[i]}"
    else
        col=$((i+1))
        mean[$i]=`cat $TMPFILE | awk -v col=$((col)) '{tot += $col} END {printf "%1.2f\n", tot/NR}'`
        >&2 echo "Peer $i: ${mean[i]}%"
    fi
done

#Calculate mean of all peers
if [ $fec -eq 0 ]; then
mean_all=`cat $TMPFILE | awk -v peers=$((peers)) '{for (i=2; i <= (peers + 1); i++) tot += $i} END {printf "%1.2f\n", tot/(NR*(peers))}'`
>&2 echo "Overall mean: $mean_all%"
fi

#Create gnuplot metedata
if [ $fec -gt 0 ]; then
    # FEC uses stacked histograms
    if [ $multi -gt 0 ]; then
        echo "set term pngcairo size 1800, (480 * $peers)"
        echo "set output \"${OUTPUT_DIR}/${dfile}-FEC-individual.png\""
    else
        echo "set term pngcairo size 1800, 480"
        echo "set output \"${OUTPUT_DIR}/${dfile}-FEC-combined.png\""
    fi
    echo 'set style data histogram'
    echo 'set style histogram rowstacked'
    echo 'set style fill solid'
    echo 'set ylabel "SLCPLC/FEC"'
    echo 'set yrange [0:12]'
    echo 'set boxwidth 0.5'
else
    #PER
    if [ $multi -gt 0 ]; then
        echo "set term pngcairo size 1800, (480 * $peers)"
        echo "set output \"${OUTPUT_DIR}/${dfile}-RTP-individual.png\""
    else
        echo "set term pngcairo size 1800, 480"
        echo "set output \"${OUTPUT_DIR}/${dfile}-RTP-combined.png\""
    fi
    echo 'set ylabel "RTP PER Rate (4 PT Moving Avg)"'
    echo 'set yrange [0:1.4]'
    echo "set format y '%2.1f%%'"
fi
echo 'set xlabel "Seconds"'
echo "set key font \",8\""


if [ $fec -gt 0 ]; then
    #
    # FEC mode
    #
    if [ $multi -gt 0 ]; then
        echo "set multiplot layout $peers, 1 title \"$title: Apollo FEC/SLC concealment (Individual)\n${format}, $peers Peers\n${channel}, ${bsstype}, ${congest}\n${date}\n\""
        for ((i = 1; i <= $peers; i++))
        do
            echo "set title \"Sink #$i, MAC ${macs[i]} \""
            col=$(((peers+2) + ((i-1)*2)))
            echo 'plot \'
            echo \"$TMPFILE\" using $((col)) t \"SLCPLC ${slc_mean[i]}\" , \'\' using $((col + 1)) t \"FEC ${fec_mean[i]}\"
        done
        echo "unset multiplot"
    else
        echo "set title \"$title: Apollo FEC (Combined)\n${format}, $peers Peers\n${channel}, ${bsstype}, ${congest}\n${date}\n\""
        echo 'plot \'
        for ((i = 1; i < $peers; i++))
        do
            col=$(((peers+2) + ((i-1)*2)))
            echo \"$TMPFILE\" using $((col)) t \"SLCPLC ${macs[i]}  \(${slc_mean[i]}\)\" , \'\' using $((col + 1)) t \"FEC ${macs[i]} \(${fec_mean[i]}\)\", \\
        done
        # No trailing comma on last line
        col=$(((peers+2) + ((i-1)*2)))
        echo \"$TMPFILE\" using $((col)) t \"SLCPLC ${macs[i]}  \(${slc_mean[i]}\)\" , \'\' using $((col + 1)) t \"FEC ${macs[i]} \(${fec_mean[i]}\)\" \\
    fi
else
    #
    # PER mode
    #
    if [ $multi -gt 0 ]; then
        echo "set multiplot layout $peers, 1 title \"$title: Apollo RTP PER (Individual)\n${format}, $peers Peers\nOverall RTP PER: $mean_all%\n${channel}, ${bsstype}, ${congest}\n${date}\n\""
        for ((i = 1; i <= $peers; i++))
        do
            echo "set title \"Sink #$i, MAC ${macs[i]} \""
            echo 'plot \'
            echo \"$TMPFILE\" u 1:$((i + 1)) lw 3 t \"${macs[i]} \(${mean[i]}%\)\" with lines, \\
            echo $mean_all lc black dt 2 t \"Overall Mean \($mean_all%\)\"
        done
    else
        echo "set title \"$title  Apollo RTP PER (Combined)\n$format, $peers Peers\nOverall RTP PER: $mean_all%\n${channel}, ${bsstype}, ${congest}\n${date}\n\""
        echo 'plot \'
        for ((i = 1; i <= $peers; i++))
        do
            echo \"$TMPFILE\" u 1:$((i + 1)) lw 3 t \"${macs[i]} \(${mean[i]}%\)\" with lines, \\
        done
        echo $mean_all lc black t \"Overall Mean \($mean_all%\)\"
    fi
fi
