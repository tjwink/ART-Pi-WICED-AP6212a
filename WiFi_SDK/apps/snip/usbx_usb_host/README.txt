snip/usbx_usb_host:
===================
- USB v2.0 Host Application Group
- USBX is the only supported USB stack/driver in current Wiced SDK

This snip collects various USB Host Applications based on different USBX USB Host Class drivers.


Global Requirements:
====================
- BCM94390x platform sets (only this platform sets have USB Hareware support)
- Specific requirements for each different application are described inside application


Board Setting:
==============
- Strapping: Check your board strapping is fixed to be USB PHY
- USB cord for Host mode: Get one "micro-A" USB cord connected on your board "uAB USB DRD" port
  (Without correct micro-A USB cord, 4390x USB Host will not be recognized. If using micro-B USB cord, 4390x USB will be recognized as Device mode)
- Connect all the other necessary components like UART on your board


Build Targets:
==============
- Various USB Host class drivers defined different USB applications

So we use different build targets to seperate various applications:
1. USB Host CDC-ACM class read/write application build:
   make snip.usbx_usb_host.usb_host_cdc_acm_read_write-<your platform>-ThreadX

2. USB Host Prolific class read/write application build:
   make snip.usbx_usb_host.usb_host_pl2303_read_write-<your platform>-ThreadX

3. USB Host Storage class application build:
   make snip.usbx_usb_host.usb_host_storage_read_write-<your platform>-ThreadX

4. USB Host HID Keyboard class application build:
   make snip.usbx_usb_host.usb_host_hid_keyboard-<your platform>-ThreadX

5. USB Host HID Mouse class application build:
   make snip.usbx_usb_host.usb_host_hid_mouse-<your platform>-ThreadX


Application Introduction:
=========================
- usb_host_cdc_acm_read_write:
  * USB Host Controller: OHCI/EHCI
  * USB Host Class: CDC-ACM
  * USB device: CDC-ACM USB device
  >> To run cdc-acm console read/write command string.
  >> To run cdc-acm get file functionality with sha1sum check.

- usb_host_pl2303_read_write:
  * USB Host Controller: OHCI/EHCI
  * USB Host Class: Prolific
  * USB device: Prolific PL2303 USB to RS232 convertor cable
  >> To run prolific console read/write loopback.
  >> To run prolific get file functionality with sha1sum check.

- usb_host_storage_read_write:
  * USB Host Controller: OHCI/EHCI
  * USB Host Class: Storage
  * USB device: USB flash disk
  >> To run single file read/write functionality with sha1sum check.
  >> To run file read/write throughput measurement.

- usb_host_hid_keyboard:
  * USB Host Controller: OHCI/EHCI
  * USB Host Class: HID (with keyboard client)
  * USB device: USB HID keyboard
  >> To run hid keyboard key input detection.

- usb_host_hid_mouse:
  * USB Host Controller: OHCI/EHCI
  * USB Host Class: HID (with mouse client)
  * USB device: USB HID mouse
  >> To run hid mouse key/movement detection.


