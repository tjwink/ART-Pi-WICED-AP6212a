---------------
IMPORTANT NOTE:
if you are testing uGUI on BCM94307_WAE boards with DIGOLE displays 
make sure you change the platform definition to use software drivers 
for i2c or spi mode. Currenlty gsio does not support few
modes for spi and other details of i2c and so it won't work unless
you set the sw drivers 

platforms/BCM943907WAE_1/platform.c

const platform_spi_t platform_spi_peripherals[] =
{
    [WICED_SPI_1]  =
    {
        .port                    = BCM4390X_SPI_0,
        .pin_mosi                = &platform_gpio_pins[WICED_GPIO_21],
        .pin_miso                = &platform_gpio_pins[WICED_GPIO_19],
        .pin_clock               = &platform_gpio_pins[WICED_GPIO_20],
        .pin_cs                  = &platform_gpio_pins[WICED_GPIO_22],
        //.driver                  = &spi_gsio_driver,
        .driver                  = &spi_bb_driver,
    },

    [WICED_SPI_2]  =
    {
        .port                    = BCM4390X_SPI_1,
        //.driver                  = &spi_gsio_driver,
        .driver                  = &spi_bb_driver,
    },
};


---------------
IMPORTANT NOTE:
if you are using a DIGOLE display make sure to check for R2/R3 or any
resistor placed serial to the SS, CLK, DATA lines for 5V compatibility.
you need to replace those resistor with a 0-ohm resistor to be able to
drive the display with 3.3V of main power.

---------------
IMPORTANT NOTE:
if you are using a DIGOLE display and you need to convert an image to 
the native RGB332, RGB656, RGB888 binary format see the ./resources 
folder for a perl tool to automatically resize and format the image
using IMAGEMAGICK.

---------------
IMPORTANT NOTE:
if you are using SPI on WAE1 or WAE2 make sure you remove the resistor R69
from the board:
- on WAE1 it is close to SW7 and R70
- on WAE2 is on the back side, close to DMIC
