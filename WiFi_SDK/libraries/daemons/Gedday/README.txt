=====================================================================
Gedday : The WICED Network Discovery Library
=====================================================================

What is Gedday?
-------------------------------------------
Gedday is a network service discovery library that incorporates various
components of Zeroconf.

Zeroconf (Zero configuration networking) is a set of techniques that
automatically creates a usable Internet Protocol (IP) network without
manual operator intervention or special configuration servers.
 -- http://en.wikipedia.org/wiki/Zero_configuration_networking


Why do I need it? What is it used for?
-------------------------------------------
Adding network discovery to your application enables the application
to advertise services such as a web server. A client on the same
network (eg. a smartphone or tablet) can use a network discovery client
such as a Bonjour browser to find the service.


 What protocols are supported?
-------------------------------------------
- Multicast DNS / DNS Service Discovery (similar to the Apple 'Bonjour' protocol)
  A lightweight version of mDNS/DNS-SD is provided.
