#!/opt/local/bin/perl
#
# Copyright (C) 2007, 2008 Niccolo Rigacci
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
# Author:       Niccolo Rigacci <niccolo@rigacci.org>
#               Russell Cattelan <cattelan@thebarn.com> 2008
#
#
# Control program for GPS units using the MediaTek (MTK) chipset.
# Tested to work with i-Blue 747 and Holux M-241 GPS data loggers.
#

use strict;

use mtk;

# Use the basename() function.
use File::Basename;
# Use the getopts() function.
use Getopt::Std;
use vars qw($opt_a $opt_b $opt_d $opt_E $opt_f $opt_h
	    $opt_l $opt_m $opt_o $opt_p $opt_R $opt_r
	    $opt_s $opt_t $opt_v $opt_w $opt_q);

my $NAME = basename($0);

#my $port     = '/dev/ttyUSB0';   # Default communication port.
my $port     = '/dev/tty.SLAB_USBtoUART';
my $mtk;
my $debug = 0;

#-------------------------------------------------------------------------
# Get options from command line.
#-------------------------------------------------------------------------
if (! getopts('ab:d:Ef:hl:m:o:p:Rr:s:tvwq') or $opt_h) {
    my $str1 = mtk::Device::describe_log_format(0x00fff);
    my $str2 = mtk::Device::describe_log_format(0xff000);
    print <<HELP;
Usage: $NAME [options]
Options:
    -a                       Read all the log memory (overlapped data)
    -b filename.bin          Do not read device, read a previously saved .bin file
    -d debug_level           Debug level: 0..7
    -E                       Erase data log memory
    -f filename              Base name for saved files (.bin and .gpx)
    -h                       Print this message and exit
    -l {on|off}              Turn loggin ON/OFF
    -m {stop|overlap}        Set STOP/OVERLAP recording method on memory full
    -o log_format            Enable or disable log fields (FIELD1,-FIELD2,...), available fields:
                             $str1
                             $str2
    -p port                  Communication port, default: $port
    -R                       Recover from disabled log: erase data and reset recording criteria
    -r time:distance:speed   Set logging criteria (zero to disable):
                             every 1-999 seconds, every 10-9999 meters, over 10-999 km/h
    -s speed                 Serial port speed
    -t                       Create a gpx file with tracks
    -v                       Print MTKBabel version and exit
    -w                       Create a gpx file with waypoints

Example:
    Download traks and waypoints from GPS device, creating the
    following files: gpsdata.bin, gpsdata_trk.gpx and gpsdata_wpt.gpx.

    mtkbabel -s 115200 -f gpsdata -t -w

HELP
    exit(1)
}


if ($opt_p) {
  $port = $opt_p;
}


if ($opt_d) {
  $debug = $opt_d;
}

if ($opt_b) {
  # just open the record bin data file, parse, and write to gpx file
  # gpx file name will default to the time stamp of the first record.
  #
  $mtk = mtk::Device->new(debug => $debug);
  my $records = $mtk->parse_log_data($opt_b);
  $mtk->gpx_print_files($records,
			tzone => "CST",
		       );
  exit 0;
} else {
  $mtk = mtk::Device->new(port => $port,
			  debug => $debug,
			 );
}

if ($opt_q) {
  $mtk->query_device();
}

if ($opt_f) {
  $mtk->query_device();
  $mtk->get_log_data(file => $opt_f);
}

if ($opt_o) {
  $mtk->set_log_format($opt_o);
}

if ($opt_m) {
  $mtk->set_record_method($opt_m);
}

if ($opt_r) {
  $mtk->set_recording_criteria($opt_r);
}

if ($opt_R) {
  $mtk->recover_log();
}

if ($opt_l) {
  $mtk->set_log_enable($opt_l);
}

if ($opt_E) {
  $mtk->erase_log();
}
