snip/usbx_usb_device:
===================
- USB v2.0 Device Application Group
- USBX is the only supported USB stack/driver in current Wiced SDK

This snip collects various USB Device Applications based on different USBX USB Device Class drivers.


Global Requirements:
====================
- BCM94390x platform sets (only this platform sets have USB Hareware support)
- Specific requirements for each different application are described inside application


Board Setting:
==============
- Strapping: Check your board strapping is fixed to be USB PHY
- USB cord for Device mode: Get one "micro-B" USB cord connected on your board "uAB USB DRD" port
  (Without correct micro-B USB cord, 4390x USB Device will not be recognized. If using micro-A USB cord, 4390x USB will be recognized as Host mode)
- Connect all the other necessary components like UART on your board


Build Targets:
==============
- Various USB Device class drivers defined different USB applications

So we use different build targets to seperate various applications:
1. USB Device CDC-ACM class read/write application build:
   make snip.usbx_usb_device.usb_device_cdc_acm_read_write-<your platform>-ThreadX

2. USB Device HID Keyboard class application build:
   make snip.usbx_usb_device.usb_device_hid_keyboard-<your platform>-ThreadX


Application Introduction:
=========================
- usb_device_cdc_acm_read_write:
  * USB Device Controller: BCM4390X
  * USB Device Class: CDC-ACM
  * USB Host: Win7/Linux PC
  >> To run cdc-acm device connect.
  >> To run cdc-acm device shutdown.
  >> To run cdc-acm device loopback.
  >> To run cdc-acm device console read/write.
  
- usb_device_hid_keyboard:
  * USB Device Controller: BCM4390X
  * USB Device Class: HID
  * USB Host: Win7/Linux PC
  >> To run hid keyboard device key input.


