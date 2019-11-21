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

#!/usr/bin/python

"""
A simple "udp echo server" for demonstrating UDP usage.
The server listens for UDP frames and echoes any received
frames back to the originating host.

"""

import socket
import string
import sys
import time
import getopt
import select


try:
    this_file = __file__
except NameError:
    this_file = "udp_echo_server.py"


def main(argv):

    HOST = ''
    PORT = 50007
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    no_echo = False

    try:
        opts, args = getopt.getopt(argv, "nhp:", ["no_echo", "help", "port="])
    except getopt.GetoptError:
        # print help information and exit:
        usage()
        sys.exit(2)

    for o, a in opts:
        if o in ("-n", "--no_echo"):
            no_echo = True
        if o in ("-h", "--help"):
            usage()
            sys.exit()
        if o in ("-p", "--port"):
            print "port: %s => %s"%(PORT, a)
            PORT = string.atoi(a)

    try:
        s.bind((HOST, PORT))
    except socket.error, msg:
        sys.exit('Error binding UDP server to port %d : %s' % (PORT, msg))

    print "UDP echo server started."

    while 1:
        a,b,c = select.select([s], [], [], 0.01)
        if s in a:
            data, address = s.recvfrom(65536)   # This is the max. UDP frame size
            #print "dlen=", len(data)
            if not data:
                break
            if not no_echo:
                sent = 0;
                while sent < len( data ):
                    sent += s.sendto("echo: "+data[sent:], address)
            print time.strftime("%b %d %H:%M:%S ", time.localtime()), address[0], ":", repr(data.split("\x00")[0])
    s.close()


def usage():
    print "Report Server Usage:"
    print
    print "Options:"
    print "  -n or --no_echo    Optional.  When given, no data will be echoed."
    print "                     Data-echoing is enabled on default."
    print "  -h or --help       Display this help message and exit."
    print "  -p or --port       Change the port that the report server binds to."
    print

if __name__ == "__main__":
    main(sys.argv[1:])
