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

Filename: webserver.py
Date: 2011-06-23
Author: chad

A simple web server that allows demonstration of IP discovery in the
Appliance app without a connection to the internet

"""

import sys
import BaseHTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler


HandlerClass = SimpleHTTPRequestHandler
ServerClass  = BaseHTTPServer.HTTPServer
Protocol     = "HTTP/1.0"

port = 80
server_address = ('127.0.0.1', port)

HandlerClass.protocol_version = Protocol
httpd = ServerClass(server_address, HandlerClass)

sa = httpd.socket.getsockname()
print "Serving HTTP on", sa[0], "port", sa[1], "..."
httpd.serve_forever()


###################################################
#import SimpleHTTPServer
#import SocketServer

#PORT = 80

#Handler = SimpleHTTPServer.SimpleHTTPRequestHandler

#httpd = SocketServer.TCPServer(("", PORT), Handler)

#print "serving at port", PORT
#httpd.serve_forever()




##################################################
#import time
#import BaseHTTPServer


#HOST_NAME = '192.168.0.6' # !!!REMEMBER TO CHANGE THIS!!!
#PORT_NUMBER = 80 # Maybe set this to 9000.


#class MyHandler(BaseHTTPServer.BaseHTTPRequestHandler):
#    def do_HEAD(s):
#        s.send_response(200)
#        s.send_header("Content-type", "text/html")
#        s.end_headers()
#    def do_GET(s):
#        """Respond to a GET request."""
#        s.send_response(200)
#        s.send_header("Content-type", "text/html")
#        s.end_headers()
#        mainfile = open("main.htm", 'r')
#        mainfile_html = mainfile.read()
#        print mainfile_html
#        s.wfile.write(mainfile_html)
#        s.wfile.write("<frameset cols=\"25%,75%\"><frame src=\"frame_a.htm\" /><frame src=\"frame_b.htm\" /></frameset>")
        # If someone went to "http://something.somewhere.net/foo/bar/",
        # then s.path equals "/foo/bar/".
#        s.wfile.write("<p>You accessed path: %s</p>" % s.path)
#        s.wfile.write("</html>")

#if __name__ == '__main__':
#    server_class = BaseHTTPServer.HTTPServer
#    httpd = server_class((HOST_NAME, PORT_NUMBER), MyHandler)
#    print time.asctime(), "Server Starts - %s:%s" % (HOST_NAME, PORT_NUMBER)
#    try:
#        httpd.serve_forever()
#    except KeyboardInterrupt:
#        pass
#    httpd.server_close()
#    print time.asctime(), "Server Stops - %s:%s" % (HOST_NAME, PORT_NUMBER)
