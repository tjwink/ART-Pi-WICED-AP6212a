#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#
#!/usr/bin/perl

## Cypress (c) 2017
##
## tool to convert any image to all the DIGOLE supported rgb formats
## note: IMAGEMAGICK package is needed, see https://www.imagemagick.org/script/index.php
use strict;
use warnings;
use Getopt::Std;

my $CONVERT = "/usr/bin/convert";


## #################################################################################### ##
## HELP
## #################################################################################### ##

## help
sub help
{
    print "\nusage: convert_img.pl -f file -w width -h height\n";
    print "  -i : image input filename\n";
    print "  -x : width (pixels) for output image (optional)\n";
    print "  -y : height (pixels) for output image (optional)\n";
    print "\n";
}


## #################################################################################### ##
## MAIN
## #################################################################################### ##

my %options=();
my $fh;
my $fh_888;
my $fh_565;
my $fh_332;

my $w=0;
my $h=0;

getopts("hi:x:y:", \%options);

# SANITY CHECKS
if (defined $options{h})
  {
    help();
    exit(0);
  }

if (!(defined $options{i}))
  {
    print "\nERROR: input filename is mandatory. \n";
    help();
    exit(0);
  }

if ( (defined $options{x}) && (defined $options{y}) )
  {
    $w = $options{x};
    $h = $options{y};
  }

my $infile = $options{i};

# compute filname for output files
my $tmpname = $infile;
$tmpname =~ s/\.\w+$//;

my $outfile_RGB888 = $tmpname.'.RGB888.h';
my $outfile_RGB565 = $tmpname.'.RGB565.h';
my $outfile_RGB332 = $tmpname.'.RGB332.h';


print "RGB332: $outfile_RGB332\n";
print "RGB565: $outfile_RGB565\n";
print "RGB888: $outfile_RGB888\n";


open ($fh,     '<:raw', $infile) or die qq{opening '$infile': $!};
open ($fh_332, '>', $outfile_RGB332) or die qq{opening '$outfile_RGB332': $!};
open ($fh_565, '>', $outfile_RGB565) or die qq{opening '$outfile_RGB565': $!};
open ($fh_888, '>', $outfile_RGB888) or die qq{opening '$outfile_RGB888': $!};


#
# use IMAGEMAGICK to convert input to rgb888
my $cmd;
my $res;

if ($w>0)
  {
    $cmd = $CONVERT." ".$infile." -depth 8 +dither -resize \"".$w."x".$h."!\" -set colorspace RGB tmp.888.rgb";
    $res = `$cmd`;
  }
else
  {
    $cmd = $CONVERT." ".$infile."  -depth 8 +dither -set colorspace RGB  tmp.888.rgb";
    $res = `$cmd`;
  }

if ($w>0)
  {
    $cmd = $CONVERT." ".$infile." -depth 8 +dither -resize \"".$w."x".$h."!\" -set colorspace RGB -ordered-dither o8x8,32,64,32 tmp.565.rgb";
    $res = `$cmd`;
  }
else
  {
    $cmd = $CONVERT." ".$infile."  -depth 8 +dither -set colorspace RGB -ordered-dither o8x8,32,64,32 tmp.565.rgb";
    $res = `$cmd`;
  }

if ($w>0)
  {
    $cmd = $CONVERT." ".$infile." -depth 8 +dither -resize \"".$w."x".$h."!\" -set colorspace RGB -ordered-dither o8x8,8,8,4 tmp.332.rgb";
    $res = `$cmd`;
  }
else
  {
    $cmd = $CONVERT." ".$infile."  -depth 8 +dither -set colorspace RGB -ordered-dither o8x8,8,8,4 tmp.332.rgb";
    $res = `$cmd`;
  }


close $fh;

if(-f "tmp.888.rgb")
  {
    open ($fh, '<:raw', "tmp.888.rgb") or die qq{opening rgb file: $!};

    my $i = 0;
    my @rgb;
    my $c = 0;

    print $fh_888 "const uint8_t img_rgb888[] ={\n";

    while (defined(my $buffer = <$fh>))
      {
        for (0 .. length($buffer) - 1)
          {
            $rgb[$i] = ord substr $buffer, $_, 1;;
            $i = $i+1;

            if($i >= 3){
              $i=0;

              ## RGB PACK 888 for Digole specs
              printf $fh_888 '0x%02x,0x%02x,0x%02x, ', $rgb[0]>>2, $rgb[1]>>2, $rgb[2]>>2;

              ## 16 columns
              $c=$c+1;
              if( $c >= 16 ){
                $c=0;
                print $fh_888 "\n";
              }
            }
          }
      }

    print $fh_888 "\n};\n";
    close $fh;
    unlink("tmp.888.rgb");
  }

if(-f "tmp.565.rgb")
  {
    open ($fh, '<:raw', "tmp.565.rgb") or die qq{opening rgb file: $!};

    my $i = 0;
    my @rgb;
    my $c = 0;

    print $fh_565 "const uint8_t img_rgb565[] ={\n";

    while (defined(my $buffer = <$fh>))
      {
        for (0 .. length($buffer) - 1)
          {
            $rgb[$i] = ord substr $buffer, $_, 1;;
            $i = $i+1;

            if($i >= 3){
              $i=0;

              ## RGB PACK 565
              my $rgb565_MSB = ( ($rgb[0] & 0xF8)      | ($rgb[1]>>5) ) & 0xFF;
              my $rgb565_LSB = ( (($rgb[1] & 0x18)<<3) | ($rgb[2]>>3) ) & 0xFF;
              printf $fh_565 '0x%02x,0x%02x, ', $rgb565_MSB, $rgb565_LSB;

              ## 16 columns
              $c=$c+1;
              if( $c >= 16 ){
                $c=0;
                print $fh_565 "\n";
              }
            }
          }
      }

    print $fh_565 "\n};\n";
    close $fh;
    unlink("tmp.565.rgb");
  }


if(-f "tmp.332.rgb")
  {
    open ($fh, '<:raw', "tmp.332.rgb") or die qq{opening rgb file: $!};

    my $i = 0;
    my @rgb;
    my $c = 0;

    print $fh_332 "const uint8_t img_rgb332[] ={\n";

    while (defined(my $buffer = <$fh>))
      {
        for (0 .. length($buffer) - 1)
          {
            $rgb[$i] = ord substr $buffer, $_, 1;;
            $i = $i+1;

            if($i >= 3){
              $i=0;

              ## RGB PACK 332
              my $rgb332 = ($rgb[0] & 0xE0) | (($rgb[1] & 0xE0)>>3) | ($rgb[2]>>6);
              #printf '(0x%02x 0x%02x 0x%02x)0x%02x,', $rgb[0],$rgb[1],$rgb[2],$rgb332;
              printf $fh_332 '0x%02x,', $rgb332;


              ## 16 columns
              $c=$c+1;
              if( $c >= 16 ){
                $c=0;
                print $fh_332 "\n";
              }
            }
          }
      }

    print $fh_332 "\n};\n";

    unlink("tmp.332.rgb");
  }


close $fh;
close $fh_332;
close $fh_565;
close $fh_888;
