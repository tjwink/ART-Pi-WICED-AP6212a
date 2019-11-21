#!/usr/bin/python

#
# $ Copyright Broadcom Corporation $
#

"""

Description: The DUT CA stands between the Test Manager and WICED, converting CAPI commands from TCP/IP link to WICED console
commands through the serial link.

"""

import serial
import io
import getopt
import sys
import time
import glob
import os
import select
import termios
import fcntl
import traceback
import socket
import string

# define global variables
HOST = ''
PORT = 9000
endl = "\r\n"
connlist = []
dut_current_command = None
last_command = ''
DEBUG_PRINTS = False

INTERFACE_ID = "wlan0"
MODEL_NAME = "WICED"
VENDOR_NAME = "Broadcom"
CA_VERSION = "4.2"               # DUT_CA Version
HARDWARE_VERSION = "1.0"         # DUT Hardware Version
SOFTWARE_VERSION = "1.1"         # SDK Version

try:
    import warnings
    warnings.filterwarnings("ignore", "", FutureWarning, "", 0)
except NameError:
    pass


class UUartTimestampError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class WicedError(Exception):
    pass

class uuart_timestamp:

    def __init__(self, tty, format, dump_file, verbose, baud=115200):
        self.debug_print = True
        self.dump_file = dump_file
        self.format = format
        #self.interactive_mode = interactive_mode
        self.ser = serial.Serial()
        self.ser.baudrate = baud
        self.ser.port = tty
        self.ser.timeout = 0  # Read timeout.
        self.ser.writeTimeout = 0  # Write timeout.
        self.stdin_fd = sys.stdin.fileno()
        self.stdout_fd = sys.stdout.fileno()
        self.termios_settings = []
        # Buffer that contains the data read from the serial port and is to be written to stdout
        self.ser_in_buf = ""
        # Buffer that contains the data read from stdin and is to be written to the serial port
        self.ser_out_buf = ""

        self.start()

    def start(self):

        # Open the tty port.
        try:
            self.ser.open()
        except serial.SerialException, x:
            # Repackage the exception as a UUartTimestampError.
            raise UUartTimestampError(x)

        # Clear the receive buffer.
        self.ser.flushInput()
        self.ser.flushOutput()
        # Write some sync bytes to the port.
        #self.ser.write('\x55\x55\x55\x55')

        # Get a copy of the terminal settings.
        new = termios.tcgetattr(self.stdin_fd)
        self.termios_settings = termios.tcgetattr(self.stdin_fd)
        # Set the terminal to non-canonical (unbuffered) input handling.
        new[3] = new[3] & ~termios.ICANON
        new[3] = new[3] & ~termios.ECHO
        termios.tcsetattr(self.stdin_fd, termios.TCSANOW, new)

        new = termios.tcgetattr(self.stdout_fd)
        new[3] = new[3] & ~termios.ICANON
        termios.tcsetattr(self.stdout_fd, termios.TCSANOW, new)

        # Currently we don't change the serial's termios settings. Just leaving this code here in case we need
        # to in the future.
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self.ser.fd)
        termios.tcsetattr(self.ser.fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

    def stop(self):
        """ This method closes the tty and cleans up the lock file.
        """
        termios.tcsetattr(self.stdin_fd, termios.TCSANOW, self.termios_settings)
        # Close the serial port.
        self.ser.close()

    # Non blocking receive_line, return '' immediately if no new line received
    def receive_line(self):
        rx_string = ''
        in_fd_list = []
        out_fd_list = []

        # If there is any serial out data, try to write it before we attempt another read
        if len(self.ser_in_buf) > 0:
            if self.ser_in_buf.find('\n') != -1:
                n = 0
                '''
                if self.interactive_mode:
                    n = os.write(self.stdout_fd, self.ser_in_buf[0:self.ser_in_buf.index('\n') + 1])
                else:
                    n = len(self.ser_in_buf[0:self.ser_in_buf.index('\n') + 1])
                '''
                # FIXME
                #n = os.write(self.stdout_fd, self.ser_in_buf[0:self.ser_in_buf.index('\n') + 1])
                n = len(self.ser_in_buf[0:self.ser_in_buf.index('\n') + 1])

                rx_string += self.ser_in_buf[0:n]
                # Shorten buffer
                self.ser_in_buf = self.ser_in_buf[n:]
            '''
            if self.interactive_mode:
                n = os.write(self.stdout_fd, self.ser_in_buf)
                rx_string += self.ser_in_buf[0:n]
                # Shorten buffer
                self.ser_in_buf = self.ser_in_buf[n:]
            '''

        # There is data to write to the serial port, check if we can write it
        if len(self.ser_out_buf) > 0:
            out_fd_list.append(self.ser.fd)

        # Check to see if our output buffer isn't too full. If it isn't we can attempt to read from the keyboard.
        #if len(self.ser_out_buf) < 1024:
        #   in_fd_list.append(self.stdin_fd)

        # Check to see if our input buffer isn't too full. If not, we can attempt to read some more.
        if len(self.ser_in_buf) < 1024:
            in_fd_list.append(self.ser.fd)

        # Don't bother testing to see if we can write to the stdout - just assume we always can.
        #print "DEBUG: fd lists:", in_fd_list, out_fd_list
        # Await for 0.1 seconds
        (in_fds, out_fds, except_fds) = select.select(in_fd_list, out_fd_list, [], 0.1)
        #print "DEBUG: after lists:", in_fds, out_fds

        if self.ser.fd in in_fds:
            self.ser_in_buf += os.read(self.ser.fd, 1024)

        '''
        if interactive_mode:
            if self.stdin_fd in in_fds:
                self.ser_out_buf += os.read(self.stdin_fd, 1024)
        '''
        if self.ser.fd in out_fds:
            self.ser_out_buf = self.ser_out_buf.replace('\n', '\r')
            self.ser_out_buf = self.ser_out_buf.replace('\x7f', '\b')
            n = os.write(self.ser.fd, self.ser_out_buf)
            self.ser_out_buf = self.ser_out_buf[n:]
            termios.tcflush(self.ser.fd, termios.TCOFLUSH)
            self.ser.flushOutput()

            #print "DEBUG: ser_in_buf", [self.ser_in_buf]
            #print "DEBUG: ser_out_buf", [self.ser_out_buf]
        #else:

        # FIXME:
        '''
        if self.stdin_fd in in_fds:
            self.ser_out_buf += os.read(self.stdin_fd, 1024)
        n = os.write(self.stdout_fd, self.ser_out_buf)
        rx_string += self.ser_out_buf[0:n]
        self.ser_out_buf = self.ser_out_buf[n:]
        '''

        return rx_string

    def send(self, msg):
        self.ser_out_buf += msg

def usage(exec_name):
    print "Usage:"
    print "    %s -l <IP address of local interface> -p <port number> -t <terminal> [-b <baud>] [-h] [--help]"%exec_name
    print "        -l <interface IP address>"
    print "            The IP address of a specific netwrok interface"
    print "        -p <port number>"
    print "            The port number to listen on"
    print "        -t <terminal>"
    print "            Path to a uart terminal device for connecting to the user UART."
    print "        -b <baud>"
    print "            Optional bit rate parameter for configuring the serial port."
    print "        -i"
    print "            Interactive mode. Use this mode with console applications."
    print "            No timestamping of screen output occurs in this mode. File output can be timestamped."
    print "        -o, --output=FILE"
    print "            Optional output file."
    print "        -r"
    print "            Overwrite output file if it already exists."
    print "        -a"
    print "            Append to output file if it already exists."
    print "        -f [h|f|i|d|b|n]"
    print "            Format of timestamp: human, float, integer, diff, float+diff(b), none."
    print "        -q"
    print "            Don't prepend output with a brief banner."
    print "        --help | -h"
    print "            This help message."

def parse_args():
    try:
        opts, args = getopt.getopt(sys.argv[1:], 't:b:f:o:qiharl:p:v', ['help','listen','port'])
    except getopt.GetoptError:
        # print help information and exit:
        usage(sys.argv[0])
        sys.exit(2)

    # The local IP address to listen on
    host = ''
    # The CAPI server port number
    port = PORT
    terminal = "/dev/ttyUSB0"
    baud = None
    dump_file = "dump_file.txt"
    format = 'n'
    verbose = True
    banner = 'n'
    overwrite = '-a'

    # Decode the options.
    for o, a in opts:
        if o == "-t":
            terminal = a
        elif o == "-b":
            baud = a
        elif o == "-f":
            if a in ('h', 'f', 'i', 'd', 'b', 'n'):
                format = a
        elif o in ('-o', '--output'):
            dump_file = a
        elif o in ("-q"):
            banner = 'n'
        elif o in ("-a", "-r"):
            overwrite = o
        elif o in ("-l", "--listen"):
            ip = a
        elif o in ("-p", "--port"):
            port = int(a)
        elif o in ("-h", "--help"):
            usage(sys.argv[0])
            sys.exit(0)

    # Error checking on command line parameters.
    if terminal == None:
        sys.stderr.write("Error: required parameter 'terminal' not provided.\n")
        usage(sys.argv[0])
        sys.exit(2)

    return (host, port, terminal, baud, format, dump_file, verbose, overwrite)

class CAPI:
    """class to process CAPI commands from Testing Engine
    """
    def __init__(self, conn, addr, uart, outfile):
        self.conn, self.addr, self.uart, self.outfile = conn, addr, uart, outfile
        # The supported commands
        self.command_list = (
                             ('sta_verify_ip_connection', self.sta_verify_ip_connection),
                             # CAPI commands accomplished by CA
                             ('ca_get_version', self.ca_get_version),
                             # Unsupported CAPI commands
                             ('sta_get_stats', self.not_implemented),
                             ('sta_set_ibss', self.not_implemented),
                             ('sta_set_eaptls', self.not_implemented),
                             ('sta_set_eapttls', self.not_implemented),
                             ('sta_set_eapsim', self.not_implemented),
                             ('sta_set_peap', self.not_implemented),
                             ('sta_set_uapsd', self.not_implemented),
                            )

    # Processing CAPI commands (coming from UCC)
    def UCC_Read(self, wiced_cmd):
        processed = None
        cmd = wiced_cmd.strip()
        print "From UCC> %s" % cmd
        if self.outfile:
            self.outfile.write("%s%s  From UCC> %s\n" % (time.strftime("%b %d %H:%M:%S", time.localtime()), (".%03d" % ((time.time()-int(time.time()))*1000)), cmd))
        parts = cmd.split(",")
        if not len(parts) % 2:
            self.UCC_Write("status,INVALID,errorCode,syntax_error")
            return
        for command, function in self.command_list:
            capi_cmd = parts[0].lower()
            if capi_cmd == command:
                try:
                    function(parts[1:])
                except WicedError:
                    self.UCC_Write("status,ERROR,errorCode,Wiced_watchdogged")
                processed = True
                break
        if not processed:
            #wiced_cmd = wiced_cmd
            self.UCC_Write("status,RUNNING")
            self.WicedWrite(cmd)
            wiced_reply = self.WicedRead() # Throw away the echo
#            time.sleep(1)
            wiced_reply = self.WicedRead()
            self.UCC_Write(wiced_reply)

    #        self.UCC_Write("status,INVALID,errorCode,unknown_command")

    # Send message to UCC
    def UCC_Write(self, msg):
        print "To UCC<", msg
        if self.outfile:
            self.outfile.write("%s%s  To UCC< %s\n" % (time.strftime("%b %d %H:%M:%S", time.localtime()), (".%03d" % ((time.time()-int(time.time()))*1000)), msg))
        self.conn.send(msg + endl)

    # Blocking while receiving Wiced's status reply, debug message will be filtered
    def WicedRead(self):
        Wiced_reply = ''
        while len(Wiced_reply) == 0:
            Wiced_reply = self.uart.receive_line()
            Wiced_reply = Wiced_reply.strip()
            if Wiced_reply == '':
                time.sleep(0.01)
                continue
            # Get rid of prompt
            if len(Wiced_reply) > 2:
                if Wiced_reply[0:2] == "> ":
                    Wiced_reply = Wiced_reply[2:]
                    print "From WICED>", Wiced_reply
                else:
                    print Wiced_reply
            if self.outfile:
                self.outfile.write("%s%s From WICED> %s\n" % (time.strftime("%b %d %H:%M:%S", time.localtime()), (".%03d" % ((time.time()-int(time.time()))*1000)), Wiced_reply))
        return Wiced_reply

    def WicedWrite(self, msg):
        # CA makes sure that all commands sent to DUT are in lowercase
        # msg = msg.lower()
        print "To WICED<", msg
        if self.outfile:
            self.outfile.write("%s%s To WICED< %s" % (time.strftime("%b %d %H:%M:%S", time.localtime()), (".%03d" % ((time.time()-int(time.time()))*1000)), msg))
        # Our Wiced's maximum uart receive buffer is set to 256
        if len(msg) > 256:
            print "Error: Due to limited buffer size, Wiced can not receive more than 256 bytes at once"
        else:
            msg = msg + "\n"
            self.uart.send(msg)

    def capi_parse(self, param_name_list, args):
        pairs = []
        Wiced_cmd = ''
        param_dict = {} # Used to validate input values
        ignored_args = ["interface", "streamid"]
        for i in xrange(len(args)/2):
            pairs.append((args[i*2], args[i*2+1]))
        for param in param_name_list:
            # Initialize the parameter dictionry with empty string
            param_dict[param] = ''
            for name, value in pairs:
                name = name.lower()
                # Don't lowercase SSID and Passphrase as they may case sensitive
                if name not in ("ssid", "passphrase"):
                    value = value.lower()
                if name == param:
                    # Add the parameter into parameter dictionary
                    param_dict[param] = value
                    # Ignore these arguments as we do not support them
                    if name in ignored_args:
                        continue
                    # STA_SET_ENCRYPTION
                    if name == "encptype":
                        if value == "none" or value == "open":
                            value = "0"
                        elif value ==  "wep":
                            value = "1"
                    # STA_SET_PSK
                    if name == "keymgmttype":
                        if value == "wpa1" or value == "wpa":
                            value = "2"
                        elif value == "wpa_mixed": # this is not in the spec
                            value == "3"
                        elif value == "wpa2":
                            value = "4"
                    if name == "flag": # Debug command: echo
                        if value in ("on", "1"):
                            value = "1"
                        elif value in ("off", "0"):
                            value = "0"
                        else:
                            value = "0"
                    Wiced_cmd += ",%s" % value
        return Wiced_cmd, param_dict

    def capi_format(self, return_value_list, Wiced_reply):
        tm_reply = ''
        if Wiced_reply == "$PowerON":
            tm_reply = "===== Wiced Rebooted ====="
            # FIXME: multiple returns
            return tm_reply

        Wiced_reply = Wiced_reply.upper()
        values = Wiced_reply.split(",")

        if return_value_list is None:
            return_value_list = []

        if values[0] != '$COMPLETE':
            tm_reply = "status," + values[0][1:]  # Get rid of the $ sign
        else:
            tm_reply = "status,COMPLETE"
            # Ignore status
            values = values[1:]
            # The offset of skipped parameters
            skip = 0
            for i in xrange(len(return_value_list)):
                name = return_value_list[i]
                if len(values) >= i-skip+1:
                    value = values[i-skip]
                else:
                    value = None
                # Filling in those parameters with fixed values
                if name == 'interface':
                    value = INTERFACE_ID
                    skip += 1
                # StreamID is fixed as we support only one stream at this moment
                elif name == 'streamID':
                    value = '1'
                    skip += 1
                # secondary-dns is not supported
                elif name == 'secondary-dns':
                    value = '0.0.0.0'
                    skip += 1
                if value == None:
                     print ">>>>>>>>>>>>Error: The last Wiced response doesn't comply to CA control protocol"
                     break
                tm_reply = tm_reply + ',' + name + ',' + value
        return tm_reply

    # CAPI commands implementation
    def ca_get_version(self, args):
        self.UCC_Write("status,RUNNING")
        self.UCC_Write("status,COMPLETE,version,%s" % (CA_VERSION))

    def sta_verify_ip_connection(self, args):
        self.UCC_Write("status,RUNNING")
        input_param_list = ["interface", "destination", "timeout"]
        return_value_list = ["connected"]
        Wiced_cmd, param_dict = self.capi_parse(input_param_list, args)
        Wiced_cmd = "verify_ip_conn" + Wiced_cmd
        self.WicedWrite(Wiced_cmd + "\n")
        Wiced_reply = self.WicedRead()
        tm_reply = self.capi_format(return_value_list, Wiced_reply)
        self.UCC_Write(tm_reply)

    def not_implemented(self, args):
        self.UCC_Write("status,ERROR,errorCode,not_implemented")
        pass

class CA:
    def __init__(self):
        self.outfile = None
        pass

    def pollNewConn(self, s, uart):
        """Routine to check for incoming connections. Accepts a socket object 's'
        as argument and returns a User object 'user'
        """
        try:
            conn, addr = s.accept()
        except socket.error:
            return None

        out_str = "Connection from %s\n" % str(addr)
        print "="*40
        print out_str,
        print "="*40

        if self.outfile:
            #self.outfile.write("%s%s  CA: %s\n" % (time.strftime("%b %d %H:%M:%S", time.localtime()), (".%03d" % ((time.time()-int(time.time()))*1000)), out_str))
            self.outfile.write(out_str)

        conn.setblocking(0)      # check this
        client = CAPI(conn, addr, uart, self.outfile)
        return client

    def main(self):
        global dut_current_command
        '''pycom main() start from here'''
        (host, port, terminal, baud, format, dumpfile, verbose, overwrite) = parse_args()
        if baud == None:
            baud = 115200
        if port == None:
            port = 4000

        if dumpfile != "":
            if glob.glob(dumpfile):
                if overwrite not in ('-a', '-r'):
                    overwrite = raw_input("%s already exists. y[es] to overwrite, n[o] to append? [yN] "%dumpfile)
                if overwrite in ('-r', 'y', 'yes', 'Y', 'o', 'overwrite', 'O'):
                    print("Overwriting %s" % (dumpfile,))
                    os.remove(dumpfile)
                    self.outfile = open(dumpfile,'w')
                elif overwrite in ('-a', 'N', 'n', 'no', 'a', 'A', 'append', ''):
                    self.outfile = open(dumpfile,'a')
                else:
                    print("Invalid input. Aborting...")
                    sys.exit(1)
            else:
                self.outfile = open(dumpfile,'w')

        # set up the server
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.setblocking(0)
        s.listen(5)
        client = None    # The telnet client, Test Manager

        try:
            uart = uuart_timestamp(terminal, format, dumpfile, verbose, baud)
        except UUartTimestampError:
            print "Could not open serial port"
            sys.exit(1)

        last_time = time.time()
        first_time = last_time
        last_dump_time = last_time

        if verbose != 'n':
            print ("\n============== %s ==============")%(time.strftime("%b %d %H:%M:%S ", time.localtime()))
        if self.outfile:
            self.outfile.write("\n============== %s ==============\n"%(time.strftime("%b %d %H:%M:%S ", time.localtime())))

        try:
            while True:
                time.sleep(0.01) # WARNING: big sleep time will affect response time
                if client is None:
                    client = self.pollNewConn(s, uart)      # check for incoming connections
                else:
                    try:
                        data = client.conn.recv(2048) # The maximum length of a CAPI command is 2048
                        if not data:
                            client = None
                        else:
                            client.UCC_Read(data)
                    except socket.error, e:
                        if e[0] == 32: # Broken pipe
                            print "Disconnected with Test Manager"
                        elif e[0] == 11: # Resource temporarily unavailable (No data available
                            pass
                        else:
                            print "Socket error::", e
                            client = None

                read_line = uart.receive_line().strip()  # May encounter the I/O error exception here
                if read_line != '':
                    global DEBUG_PRINTS
                    if DEBUG_PRINTS:
                        print "From WICED> %s" % read_line
                    pass

                out_str = ""
                this_time = time.time()
                #print out_str
                if dumpfile != "":
                    #outfile.write(out_str + "\n")
                    # I get bored waiting for the output file to flush.
                    # So manually flush it every 10 seconds...
                    if (this_time-last_dump_time) > 10:
                        self.outfile.flush() # flush every 10 seconds
                        last_dump_time = this_time

        except KeyboardInterrupt:
            if verbose != 'n':
                print ("=========== Exit: %s ===========\n\n")%(time.strftime("%b %d %H:%M:%S ", time.localtime()))
            if self.outfile:
                self.outfile.write("\n=========== Exit: %s ===========\n\n"%(time.strftime("%b %d %H:%M:%S ", time.localtime())))

            print "Shutting down ..."
            uart.stop()
            s.shutdown(2)
            s.close()
            if self.outfile:
                self.outfile.flush()
            client = None

            if client: # Close the connection
                client.close()


if __name__ == "__main__":
    ca = CA()
    ca.main()
