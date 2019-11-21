#!/usr/bin/python

"""
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

Filename: report_server.py
Date: 2011-06-23
Author: tbriers, jasonrc

A simple "report server" for demonstrating usage of the WICED canned_send app.
The server prints data received in UDP frames to the console.

"""

import socket
import sys
import time
import ctypes


HTML_START = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.01 Frameset//EN\"\n\
\"http://www.w3.org/TR/html4/frameset.dtd\">\n\
<HTML>\n\
<HEAD>\n\
<TITLE>A simple frameset document</TITLE>\n\
</HEAD>\n\
<FRAMESET cols=\"20%, 80%\">\n\
  <FRAME src=\"frame_a.htm\">\n\
  <FRAME src=\""
HTML_END = "\">\n\
  <NOFRAMES>\n\
      <P>This page needs frames\n\
  </NOFRAMES>\n\
</FRAMESET>\n\
</HTML>\n"

HOST = ''
PORT = 50007
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    s.bind((HOST, PORT))
except socket.error, msg:
    sys.exit('Error binding UDP server to port %d : %s' % (PORT, msg))

print
print "UDP server started. Waiting for data ..."

remote_frame = "http://www.google.com"
# Generate initial index.htm file
index_htm = open("index.htm", 'w')
index_htm.write(HTML_START + remote_frame + HTML_END)
index_htm.close()

while 1:
    data, address = s.recvfrom(1024)
    if not data:
        break
    index_htm = open("index.htm", 'w')
    index_htm.write(HTML_START + data + HTML_END)
    index_htm.close()
    print data

s.close()


