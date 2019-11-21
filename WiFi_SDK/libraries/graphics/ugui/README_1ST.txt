
NOTE: for uGUI test app see:
/apps/snip/graphics/uguitest/


NOTE: 
uGUI can be used to control multiple displays, either on the same bus (i2c) or different bus (spi/serial) 
but please note that it is *NOT* thread safe.

always make sure that the display control is performed in a single thread of your WICED application.

one single thread to control all the displays by selecting them using the call

UG_S16 UG_SelectGUI( UG_GUI* g );
