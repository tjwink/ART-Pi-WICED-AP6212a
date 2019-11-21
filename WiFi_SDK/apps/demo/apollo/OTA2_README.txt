

 OTA2 Support notes:

 First, build the ota2_extraction application
   - This is a generic extractor needed for all OTA2 applications

      snip.ota2_extract-<PLATFORM>

 To build OTA2 version of Apollo use one of these build command lines:
  (adding ota2_factory_download, ota2_factory_image, or ota2_image indicates that it is an OTA2 build)

First time (and each time you want to test Factory Reset):
 demo.apollo-<PLATFORM> ota2_factory_download download download_apps run
    - builds the application and creates /build/demo.apollo-<PLATFORM>/OTA2_factory_reset_file.bin
    - downloads the factory reset image to SFLASH - This takes extra time
    - downloads and runs the application

Subsequent builds, use this build string:
 demo.apollo-<PLATFORM> ota2_image download download_apps run
    - builds the application and creates /build/demo.apollo-<PLATFORM>/OTA2_image_file.bin
    - Does NOT download the factory reset image -or- the downloadable image to SFLASH
    - downloads and runs the application

Use this to create an OTA2 image suitable to put on your server for updating.
You can change the OTA2 Application version # to be sure of the version you are running.
 demo.apollo-<PLATFORM> ota2_image APP_VERSION_FOR_OTA2_MAJOR=x APP_VERSION_FOR_OTA2_MINOR=y
    - one of the ota2_xxx args must be present to affect an OTA2 build
    - set 'x' and 'y' to decimal values for your Application version
      - This is output to the console with 'config' so you can check which version is in FLASH
    - builds the application and creates /build/demo.apollo-<PLATFORM>/OTA2_image_file.bin
      - This is the update file to put on your server
    - does not download the image to the SFLASH
    - does not download the application to the platform

To use PUSH mode for updates, reset the board and hold the Factory Reset button for ~5 seconds
    You will see that the OTA2 extract application is running.
    Connect your PC's wifi to the SOFT_AP_SSID of your board defined in apps/demo/apollo/wifi_dct_config.h
    The SSID is printed to the console as well.
    Use your Web Browser to go to 192.168.10.1

 #define SOFT_AP_SSID         "apollo"
 #define SOFT_AP_PASSPHRASE   "abcd1234"
 #define SOFT_AP_SECURITY     WICED_SECURITY_OPEN
 #define SOFT_AP_CHANNEL      149

 To PULL updates from the Web
     - adjust these lines in apps/demo/apollo/wifi_config_dct.h
     - Rebuild, then from the command line, type:
      > get_update <Server_Name | Server_IP>/<file_name>

 #define CLIENT_AP_SSID       "AP_For_Upgrade_Connection"
 #define CLIENT_AP_PASSPHRASE "AP_password"
 #define CLIENT_AP_BSS_TYPE   WICED_BSS_TYPE_INFRASTRUCTURE
 #define CLIENT_AP_SECURITY   WICED_SECURITY_WPA2_AES_PSK
 #define CLIENT_AP_CHANNEL    44
 #define CLIENT_AP_BAND       WICED_802_11_BAND_5GHZ

For timed PULL operation, adjust these fields in apollo_start_bg_update()
     NOTE: The system uses relative times for checking updates.

     apollo->ota2_bg_params.initial_check_interval   = 5;                   <- initial wait time (in seconds)
     apollo->ota2_bg_params.check_interval           = SECONDS_PER_HOUR;    <- interval after initial wait for next check (1 hour)
                                                                               use SECONDS_PER_DAY for 1 day interval
     apollo->ota2_bg_params.retry_check_interval     = SECONDS_PER_MINUTE;  <- retry check if check fails (minimum 1 minute)

Set the default PULL address in the Application DCT (apollo_dct.c):
     .ota2_default_update_uri        = "192.168.1.100/ota2_firmware/OTA2_image_file.bin",

     NOTE: You can change this value using code by reading and writing the Application DCT
           You can change the uri by using "config ota2_uri <new uri>" (remember to "config save")
           You can over-ride the uri when you use the command console "get_update <uri>"

 Set the default values for these flags in the Application DCT (apollo_dct.c) based on your requirements:
    .ota2_stop_playback_to_update   = 1,
    .ota2_reboot_after_download     = 1,


To Factory Reset, reset the board and hold the Factory Reset button for ~ 10 seconds


You can test the Version number of your software when checking for updates by building the update image with:

    demo.apollo-<PLATFORM> ota2_image APP_VERSION_FOR_OTA2_MAJOR=y APP_VERSION_FOR_OTA2_MINOR=y

    set 'x' and 'y' to decimal values for your Application version.
    These values are saved in the Application DCT and the OTA2_image_file.bin.

    They are compared in the ota2 callback if you enable the check by defining CHECK_OTA2_UPDATE_VERSION=1
    in the apollo.mk file. Default is commented out so that it does not check the update version during development.

    #GLOBAL_DEFINES      += CHECK_OTA2_UPDATE_VERSION=1

