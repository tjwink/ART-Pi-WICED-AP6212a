#
# $ Copyright Broadcom Corporation $
#

echo USB ports before initialising FTDI
sudo ls -l /dev/ttyUSB*
sudo ifconfig eth0 down
sudo ifconfig eth0 192.168.250.40 netmask 255.255.0.0 up
sudo rmmod ftdi_sio
sudo modprobe ftdi_sio vendor=0xa5c product=0x43fa
dmesg | grep FTDI | grep attach
#echo USB ports after initialising FTDI
#sudo ls -l /dev/ttyUSB*

