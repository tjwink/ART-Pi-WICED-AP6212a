===============================================================================
Cypress WICED Studio Software Development Kit - README
===============================================================================

WICED Studio provides a full complement of application level APIs, libraries
and tools needed to design and implement secure embedded wireless networking
applications.

For details on supported WICED Studio platforms and features, refer to
"WICED Studio 6.x Technical Brief" at https://community.cypress.com/docs/DOC-14251

WICED Studio is structured as follows:
<WICED Platform>   : High level folder for each supported platform
common             : Common apps, libraries, headers across bluetooth platforms
doc                : API & Reference Documentation, Eval Board & Module Schematics
drivers            : Drivers for USB serial converter
test               : Tools provided for automation testing
wiced_tools        : Build tools, compilers, debugger, makefiles, programming tools etc.
README.txt         : WICED Studio high level README file (this file)
version.txt        : Version of WICED Studio


Getting Started
-------------------------------------------------------------------------------
If you are unfamiliar with WICED Studio, please refer to the WICED Studio
Quick Start Guide or Kit Guide for your WICED Platform under the Eclipse IDE
'Project Explorer', located here: <WICED Platform>/doc/
For example:
20706-A2_Bluetooth/doc/CYW920706WCDEVAL-Kit-Guide.pdf or
43xxx_Wi-Fi/doc/WICED-QSG.pdf

The WICED Studio Quick Start or Kit Guide documents the process to setup a computer
for use with the WICED Studio IDE and WICED Evaluation Board.

The currently active project, including associated support files and make
targets specific to each platform, may be switched using the "WICED Platform"
drop down menu of the Eclipse IDE.

Please see the  <WICED Platform>/README.txt for each WICED Platform to get
detailed descriptions for each platform.

WICED Studio includes lots of sample applications in the <WICED Platform>/apps
directory.  See the <WICED Platform>/apps/README.txt for more detailed
descriptions of the apps.


Tools
-------------------------------------------------------------------------------
The GNU ARM toolchain is from Yagarto, http://yagarto.de

WICED Studio also supports ARM RealView 4.1 and above compiler toolchain:
http://www.arm.com

The standard WICED Evaluation board provides single USB-serial port for programming.

The debug interface is ARM Serial Wire Debug (SWD) and shares pins with download
serial lines TXd (SWDCLK) and RXd (SWDIO).

Building, programming and debugging of applications is achieved using either a
command line interface or the WICED Studio IDE as described in the Quick Start
Guide or Kit Guide.


WICED Technical Support
-------------------------------------------------------------------------------
WICED support is available on the Cypress forum at
https://community.cypress.com/welcome
Access to the WICED forum is restricted to bona-fide WICED customers only.

Cypress provides customer access to a wide range of additional
information, including technical documentation, schematic diagrams,
product bill of materials, PCB layout information, and software
updates. Please contact your Cypress Sales or Engineering support
representative or Cypress support at http://www.cypress.com/support.


Further Information
-------------------------------------------------------------------------------
Further information about WICED and the WICED Development System is
available on the WICED website at
http://www.cypress.com/products/wireless-connectivity
or by contacting Cypress support at http://www.cypress.com/support
