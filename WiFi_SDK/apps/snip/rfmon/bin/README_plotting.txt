
Scripts to create plots based on stats from rfmon.

Develop and run in cygwin environment.

Depends on gnuplot and is called as:
/cygdrive/c/Program\ Files\ \(x86\)/gnuplot/bin/wgnuplot
edit location as needed.

On rfmon:
  1) rf_style 1         # Set output style to charting
  2) rf_continuous 1    # Set to keep outputting.
  3) Capture the output into a file.   # Terminal emulator may have logging feature or copy N paste into a file.
     The first line should have the MAC addresses of all sinks.  No blank lines or extra characters in lines.

On charting machine:
$ ./create_plot.sh log_file "IBSS, in the lab"
Usage:  ./create_plot.sh log_file ["Title"]

Scripts parse data for number and mac addresses of all sinks.

Output is a .png file in ./plot directory
