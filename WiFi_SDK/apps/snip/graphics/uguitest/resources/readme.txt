DIGOLE display requires/support raw images formatted as RGB 888,565 and 332 format.

this tool will convert any input picture to the correct byte array in a .h file.

this tool leverages IMAGEMAGICK to reduce the colormap with the appropriate threshold and dithering.
for quantization+dither see the link:  http://www.imagemagick.org/Usage/quantize/

---

From a PNG/BMP/JPG file you can convert it to a DIGOLE format using this script,
it will create 3 files (.h) with a byte array listed that you can upload to the display.

the tool syntax follows these examples:

 ./convert_img.pl -i ./demo_cover.png (convert and mantain the picture size)

 ./convert_img.pl -i ./demo_cover2.png -x 64 -y 64 (cnvert and resample)
