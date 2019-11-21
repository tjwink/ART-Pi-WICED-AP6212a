
Wi-Fi MESH demo application

This application snippet demonstrates using WICED Wi-Fi MESH.
In an infrastructure network, an AP talks to directly to the STA's associated to it.
In an adhoc network, each STA talks directly to its peers in the network.
In a WICED Wi-Fi Mesh network, each node can talk indirectly, through the other node, to reach an otherwise unreachable node.

In WICED Wi-Fi Mesh network, one of the nodes can be designated to also connect to your router (and hence the outside internet).  This is the Gateway node.

To demonstrate the app, work through the following steps:

 1. Edit the wifi_config_dct.h file:
     a. Describe the Mesh Network:
        CONFIG_AP_SSID       => Name of the Mesh network you are creating.
        CONFIG_AP_SECURITY   => For 6.1 release this must be WICED_SECURITY_OPEN.
        CONFIG_AP_PASSPHRASE => Ignored for this release.
        CONFIG_AP_CHANNEL    => Channel to setup Mesh network. If using a Gateway, for 6.1 release, this
                                must be the same channel that the Gateway router (below) is on. 
                                That is, the Mesh network needs to be on the same channel as the Gateway.

        These lines should be identical on all nodes of this mesh network.

     b.  Describe the Gateway node to the internet:
         CLIENT_AP_2_SSID        => SSID of router
         CLIENT_AP_2_SECURITY    => As appropriate for router
         CLIENT_AP_2_PASSPHRASE  => As appropriate for router
         CLIENT_AP_2_BSS_TYPE    => WICED_BSS_TYPE_INFRASTRUCTURE
         CLIENT_AP_2_BAND/CHANNEL=> Ignored

 2. Plug the WICED eval board into your computer
 3. Open a terminal application and connect to the WICED eval board
 4. Build and download the application to each of the WICED boards in the network.

After the download completes, the terminal displays WICED startup information.


Default mode is not a gateway. To make a node also operate as a Gateway, run this command and reboot:
> dct_misc_wifi mesh gate 1

Default is static IP addressing, to enable DHCP addressing, run this command and reboot:
> dct_misc_wifi mesh ip dhcp

iperf works normally. On WICED, Ping does not do name resolution and accepts only IP addresses. If
you want to ping an outside address like google.com through the gateway, you will need to manually
convert google.com into an IP address and ping that address.

Rebooting a mesh node may take up to a minute to get its address populated throughout all
the nodes in a network.

Some useful commands:
mesh_status: Shows status on directly connected nodes.
mesh_route:  Shows next-hop to get to indirectly connected nodes.

