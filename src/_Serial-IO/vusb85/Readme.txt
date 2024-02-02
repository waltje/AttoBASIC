This is a modified version of vusb to work on an attiny85.
The original source is from http://www.obdev.at/products/vusb/download.html

The changes have been done to get the examples hid-mouse and hid-data working on an attiny85.
The other examples have not got tested, but should also work once you modify the Makefile and usbconf.h
The required hardware and a working firmware can be seen in the project http://www.obdev.at/products/vusb/easylogger.html

The following files got modified:
 libs-device/osccal.h
 examples/hid-mouse/Makefile
 examples/hid-mouse/usbconf.h
 examples/hid-data/Makefile
 examples/hid-data/usbconf.h


The steps to do the modifications yourselfs is documented here:
http://www.ruinelli.ch/how-to-use-v-usb-on-an-attiny85

(c) 2011 by George Ruinelli
www.ruinelli.ch
--------------------------------

Original code:
(c) 2010 by OBJECTIVE DEVELOPMENT Software GmbH.
http://www.obdev.at/