--------------------------------------------
BUILD INSTRUCTIONS
--------------------------------------------

a) Release build test.nsh-NuttX-NuttX_NS-BCM943909WCD1_3
b) Debug build test.nsh-NuttX-NuttX_NS-BCM943909WCD1_3-debug
c) Put application, WLAN firmware and configuration into serial flash: test.nsh-NuttX-NuttX_NS-BCM943909WCD1_3-debug download
   Than power cycle board and enjoy NSH running.

--------------------------------------------
DEBUGGING
--------------------------------------------

Debug as usually using Wiced-SDK via Eclipse.

--------------------------------------------
NSH AND NETWORKING
--------------------------------------------

It is IMPORTANT to put WLAN firmware and configuration (please check above build instructions) into serial flash to be a able to run application.

If tried to build another NuttX application (e.g. test.nuttx_bringup) please clean the build ("clean" target, "make clean") before building NSH.

---

1) STA interface

There are two ways to bring-up STA network interface:
a) 'wifi_up sta' (command to be typed in NSH console) - this will connect device to AP programmed in flash-based DCT
b) 'wiced_console "join SSID wpa2 PASSWORD"' - this forwards command to WICED console (same as test.console application).
   Command programs flash-based DCT and than do same as 'wifi_up sta'.
   So simplest way is to use (b) once, and than use (a) all the times, even after power cycle.

Now device is associated with AP, network interface is registered but not yet brought up (not yet IP address assigned).

To bring-up and assign IP settings 'ifconfig' command to be used.
Syntax is 'ifconfig [nic_name [<ip-address>|dhcp] [dr|gw|gateway <dr-address>] [netmask <net-mask>] [dns <dns-address>]]',
where nic_name is ethN. Please run 'ifconfig' to get info about interfaces.

Interface can be brought down/up any time, without changing IP settings using 'ifup nic_name' / 'ifdown nic_name' commands.

---

2) AP interface

To bring-up AP network interface:
a) 'wifi_up ap' - this will start AP using parameters programmed in flash-based DCT.
b) 'wiced_console "start_ap SSID open 1 6"' - this forwards command to WICED console (same as test.console application).
   Command programs flash-based DCT and than do same as 'wifi_up ap'.
   So simplest way is to use (b) once, and than use (a) all the times, even after power cycle.

To bring-up and assign IP settings same 'ifconfig'/'ifup'/'ifdown' as in STA case to be used.

Please pay attention that ethN name is assigned in order of interface registering.
So if STA brought-up first, it will be eth0. But if AP first it will be eth0, and STA will be eth1.
Each wifi_down/wifi_up assigns new subsequent N to ethN.

---

Opposite to 'wifi_up ap|sta' is 'wifi_down ap|sta' command.

To test networking 'ping', 'ping6' and 'iperf' commands are available.

For more information 'help' and 'wiced_console help' commands are available.

--------------------------------------------
NSH AND OTHER BUILTIN APPS
--------------------------------------------

1) RTOS test ostest
   Steps to run ostest:
    
   a) create SmartFS (RAM base)
      nsh> flash_eraseall /dev/smart1
      nsh> mksmartfs /dev/smart1
      nsh> mount -t smartfs /dev/smart1 /mnt
    
   b) start ostest
      nsh> ostest
    
   Note: step (a) can be skipped if you don't care AIO test result.
    
2) Audio nxplayer
    
   To test build and download nsh app, connect earphones to CODEC OUT/IN on eval board.
    
   nsh>nxplayer
   nxplayer>play
