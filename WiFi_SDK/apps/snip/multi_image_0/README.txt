snip/multi_image_0 and snip/multi_image_1 work together.

They demonstrate how to put one application into deepsleep and have the system wake into another application.
This will allow for application swapping while maintaining a WiFi connection.
This can be extended to a third application.

Requirements:
- BCM94390x platform (this platform has Always On RAM and tiny_bootloader support)
- All Application DCT structures/fields for all applications must match
- All Always on RAM  variables for all applications must match

Concept:
The Tiny bootloader is used to allow the system to be put in deepsleep.
Upon awalening, the tiny bootloader powers up the system and re-loads the application into RAM.
Normal cold boot sequence (not used for warm boot) sets the Application LUT address for the boot_client
       (from platform_dct_header_t.boot_detail) into the config.app_address in AON for tiny_bootloader
       to find and load the application.
Using the API call platform_deepsleep_set_boot() you can instruct the tiny_bootloader to load an
       alternate application upon waking up.
  Note that in this example we also use wiced_framework_set_boot() so that on the next cold boot,
      the bootloader will load the same application.

NOTE: When the hardware is connected to the IDE (like just after downloading the applications), the system is in an "Always on" state,
      will not go into deep sleep, and will not perform a warm boot. In order to test the functionality, you can leave the Console connected
      but you must power cycle the board to get it out of the *always on* mode.

Using this test:

1 - build snip/multi_image_1

    make snip.multi_image_1-<platform>

2 - build and run snip/multi_image_0

    make snip.multi_image_0-<platform> download download_apps run

To switch applications:

    In the command console, type "switch"
