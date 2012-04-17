#
# Copyright (C) 2007, 2008 Niccolo Rigacci
# Copyright (C) 2008 Russell Cattelan
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
# Author (module base): Russell Cattelan <cattelan@thebarn.com>
#
#
# Control program for GPS units using the MediaTek (MTK) chipset.
# Tested to work with i-Blue 747 and Holux M-241 GPS data loggers.
#
#
#
package mtk::GPX;

use warnings;
use strict;
use Carp;

use IO::File;
use Date::Format;

use Data::Dumper;

my $EOL = "\n";

use enum qw(BITMASK:VALID_
	    NOFIX
	    SPS
	    DGPS
	    PPS
	    RTK
	    FRTK
	    ESTIMATED
	    MANUAL
	    SIMULATOR
);
use enum qw(BITMASK:RCR_
	    TIME
	    SPEED
	    DISTANCE
	    BUTTON);

sub new {
  my $class = shift;
  my $filename;
  my $self = {};

  my %args = @_;

  if (exists $args{file}) {
    $filename = $args{file};
    print "$args{'file'} open file name \n";
  }

  bless($self, $class);

  $self->{trk_number} = 0;
  $self->{wpt_number} = 0;
  $self->{trk_minlat} =   90.0;
  $self->{trk_minlon} =  180.0;
  $self->{trk_maxlat} =  -90.0;
  $self->{trk_maxlon} = -180.0;
  $self->{wpt_minlat} =   90.0;
  $self->{wpt_minlon} =  180.0;
  $self->{wpt_maxlat} =  -90.0;
  $self->{wpt_maxlon} = -180.0;

  $self->{fh} = new IO::File $filename, "w" or croak "open failed: $filename $!\n";

  return $self;
}

#-------------------------------------------------------------------------
# Print the header of a GPX file.
#-------------------------------------------------------------------------
sub gpx_print_gpx_begin {
  my $self   = shift;
  my $time = time();

  my $fp = $self->{fh};
  print $fp sprintf('<?xml version="1.0" encoding="UTF-8"?>%s', $EOL);
  print $fp '<gpx' . $EOL;
  print $fp '  version="1.1"' . $EOL;
  print $fp '  creator="MTKBabel - http://www.rigacci.org/"' . $EOL;
  print $fp '  xmlns="http://www.topografix.com/GPX/1/1"' . $EOL;
  print $fp '  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"' . $EOL;
  print $fp '  xmlns:mtk="http://www.rigacci.org/gpx/MtkExtensions/v1"' . $EOL;
  print $fp '  xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd' . $EOL;
  print $fp '                      http://www.rigacci.org/gpx/MtkExtensions/v1 ',
    'http://www.rigacci.org/gpx/MtkExtensions/v1/MtkExtensionsv1.xsd">' . $EOL;
  print $fp sprintf('<metadata>%s', $EOL);
  print $fp sprintf('  <time>%s</time>%s', time2str('%Y-%m-%dT%H:%M:%SZ', $time, 'GMT'), $EOL);;
  print $fp sprintf('  <bounds minlat="%.9f" minlon="%.9f" maxlat="%.9f" maxlon="%.9f"/>%s',
		    $self->{trk_minlat}, $self->{trk_minlon}, $self->{trk_maxlat}, $self->{trk_maxlon}, $EOL);
  print $fp sprintf('</metadata>%s', $EOL);
}

sub gpx_print_gpx_end {
  my $self = shift;
  #local
  my $fp = $self->{fh};
  print $fp sprintf('</gpx>%s', $EOL);
}

#-------------------------------------------------------------------------
# Open and close the GPX <trk> tag.
#-------------------------------------------------------------------------
sub gpx_print_trk_begin {
  my $self = shift;
  my $record = shift;
  #local
  my $fp = $self->{fh};

  print $fp sprintf('<trk>%s', $EOL);
  print $fp sprintf('  <name>%s</name>%s', $record->{utc}, $EOL);
  print $fp sprintf('  <number>%u</number>%s', $self->{trk_number}, $EOL) if ($self->{trk_number} > 0);;
  print $fp sprintf('<trkseg>%s', $EOL);
  $self->{trk_number}++;
}
sub gpx_print_trk_end {
  my $self = shift;
  my $fp = $self->{fh};
  print $fp sprintf('</trkseg>%s', $EOL);
  print $fp sprintf('</trk>%s', $EOL);
}

#-------------------------------------------------------------------------
# Print a GPX <trkpt>.
#-------------------------------------------------------------------------
sub gpx_print_trkpt {
  my $self = shift;
  my $record = shift;
  my $fp = $self->{fh};

  print $fp sprintf('<trkpt lat="%.9f" lon="%.9f">%s', $record->{latitude}, $record->{longitude}, $EOL);
  print $fp sprintf('  <ele>%.6f</ele>%s', $record->{height}, $EOL) if (defined($record->{height}));
  print $fp sprintf('  <time>%s</time>%s', $record->{utc},    $EOL) if (defined($record->{utc}));
  gpx_print_pt_attributes($self,$record);
  print $fp sprintf('</trkpt>%s', $EOL);
  $self->{trk_minlat} = $record->{latitude} if ($record->{latitude}  < $self->{trk_minlat});
  $self->{trk_maxlat} = $record->{latitude}  if ($record->{latitude}  > $self->{trk_maxlat});
  $self->{trk_minlon} = $record->{longitude} if ($record->{longitude} < $self->{trk_minlon});
  $self->{trk_maxlon} = $record->{longitude} if ($record->{longitude} > $self->{trk_maxlon});
}

#-------------------------------------------------------------------------
# Print a GPX <wpt>.
#-------------------------------------------------------------------------
sub gpx_print_wpt {
  my $self = shift;
  my $record = shift;
  my $fp = $self->{fh};

    $self->{wpt_number}++;
    print $fp sprintf('<wpt lat="%.9f" lon="%.9f">%s', $record->{latitude}, $record->{longitude}, $EOL);
    print $fp sprintf('  <ele>%.6f</ele>%s',   $record->{height},$EOL)
      if (defined($record->{height}));
    print $fp sprintf('  <time>%s</time>%s',   $record->{utc},$EOL)
      if (defined($record->{utc}));
    print $fp sprintf('  <name>%03d</name>%s', $self->{wpt_number},$EOL);
    print $fp sprintf('  <cmt>%03d</cmt>%s',   $self->{wpt_number},$EOL);
    print $fp sprintf('  <desc>%s</desc>%s',   $record->{utc},$EOL)
      if (defined($record->{utc}));
    print $fp sprintf('  <sym>Flag</sym>%s',$EOL);
    gpx_print_pt_attributes($self,$record);
    print $fp sprintf('</wpt>%s', $EOL);

    $self->{wpt_minlat} = $record->{latitude}
      if ($record->{latitude}  < $self->{wpt_minlat});
    $self->{wpt_maxlat} = $record->{latitude}
      if ($record->{latitude}  > $self->{wpt_maxlat});
    $self->{wpt_minlon} = $record->{longitude}
      if ($record->{longitude} < $self->{wpt_minlon});
    $self->{wpt_maxlon} = $record->{longitude}
      if ($record->{longitude} > $self->{wpt_maxlon});
}

#-------------------------------------------------------------------------
# Print <trkpt> and <wpt> common attributes.
#-------------------------------------------------------------------------
sub gpx_print_pt_attributes {
  my $self = shift;
  my $record = shift;
  my $fp = $self->{fh};

    print $fp sprintf('  <type>%s</type>%s',  describe_rcr_gpx($record->{rcr}),         $EOL)
      if (describe_rcr_gpx($record->{rcr}));
    print $fp sprintf('  <fix>%s</fix>%s',  describe_valid_gpx($record->{valid}),       $EOL)
      if (describe_valid_gpx($record->{valid}));
    print $fp sprintf('  <sat>%u</sat>%s',                     $record->{nsat_in_use},  $EOL)
      if (defined($record->{nsat_in_use}));
    print $fp sprintf('  <hdop>%.2f</hdop>%s',                 $record->{hdop},         $EOL)
      if (defined($record->{hdop}));
    print $fp sprintf('  <vdop>%.2f</vdop>%s',                 $record->{vdop},         $EOL)
      if (defined($record->{vdop}));
    print $fp sprintf('  <pdop>%.2f</pdop>%s',                 $record->{pdop},         $EOL)
      if (defined($record->{pdop}));
    print $fp sprintf('  <ageofdgpsdata>%u</ageofdgpsdata>%s', $record->{dage},         $EOL)
      if (defined($record->{dage}));
    print $fp sprintf('  <dgpsid>%u</dgpsid>%s',               $record->{dsta},         $EOL)
      if (defined($record->{dsta}));

    if (defined($record->{speed}) or
        defined($record->{heading}) or
        defined($record->{nsat_in_view}) or
        defined($record->{millisecond}) or
        defined($record->{distance}) or
        defined($record->{satdata})) {

    print $fp sprintf('  <extensions>%s', $EOL);
    print $fp sprintf('    <mtk:wptExtension>%s', $EOL);
    print $fp sprintf('      <mtk:valid>%s</mtk:valid>%s',  describe_valid_mtk($record->{valid}),       $EOL)
      if (defined($record->{valid}));
    print $fp sprintf('      <mtk:speed>%.6f</mtk:speed>%s',                   $record->{speed},        $EOL)
      if (defined($record->{speed}));
    print $fp sprintf('      <mtk:heading>%.6f</mtk:heading>%s',               $record->{heading},      $EOL)
      if (defined($record->{heading}));
    print $fp sprintf('      <mtk:satinview>%u</mtk:satinview>%s',             $record->{nsat_in_view}, $EOL)
      if (defined($record->{nsat_in_view}));

    foreach my $sat (@{$record->{satdata_array}}) {
      print $fp sprintf('      <mtk:satdata sid="%u" inuse="%u">%s',   $sat->{sid}, $sat->{inuse}, $EOL);
      print $fp sprintf('        <mtk:elevation>%d</mtk:elevation>%s',
			$sat->{elevation},$EOL) if (exists($sat->{elevation}));
      print $fp sprintf('        <mtk:azimuth>%u</mtk:azimuth>%s',
			$sat->{azimuth},$EOL) if (exists($sat->{azimuth}));
      print $fp sprintf('        <mtk:snr>%u</mtk:snr>%s',
			$sat->{snr},$EOL) if (exists($sat->{snr}));
      print $fp sprintf('      </mtk:satdata>%s', $EOL);
    }

    print $fp sprintf('      <mtk:msec>%u</mtk:msec>%s',           $record->{millisecond},  $EOL)
      if (defined($record->{millisecond}));
    print $fp sprintf('      <mtk:distance>%.9f</mtk:distance>%s', $record->{distance},     $EOL)
      if (defined($record->{distance}));
    print $fp sprintf('    </mtk:wptExtension>%s', $EOL);
    print $fp sprintf('  </extensions>%s', $EOL);

    }
}

# Description suitable for GPX <trkpt> <fix> element.
sub describe_valid_gpx {
    my $valid = shift;
    return('none')         if ($valid == VALID_NOFIX);
    return('3d')           if ($valid == VALID_SPS);
    return('dgps')         if ($valid == VALID_DGPS);
    return('pps')          if ($valid == VALID_PPS);
    return(undef);
}

# Description suitable for GPX <trkpt> <type> element.
sub describe_rcr_gpx {
    my $rcr = shift;
    my $str = '';

    $str .= ',TIME'     if ($rcr & RCR_TIME);
    $str .= ',SPEED'    if ($rcr & RCR_SPEED);
    $str .= ',DISTANCE' if ($rcr & RCR_DISTANCE);
    $str .= ',BUTTON'   if ($rcr & RCR_BUTTON);

    return(undef) if ($str eq '');
    return(substr($str, 1));
}

#-------------------------------------------------------------------------
# Return a string describing some field.
#-------------------------------------------------------------------------
sub describe_valid_mtk {
    my $valid = shift;
    return('nofix')     if ($valid == VALID_NOFIX);
    return('sps')       if ($valid == VALID_SPS);
    return('dgps')      if ($valid == VALID_DGPS);
    return('pps')       if ($valid == VALID_PPS);
    return('rtk')       if ($valid == VALID_RTK);
    return('frtk')      if ($valid == VALID_FRTK);
    return('estimated') if ($valid == VALID_ESTIMATED);
    return('manual')    if ($valid == VALID_MANUAL);
    return('simulator') if ($valid == VALID_SIMULATOR);
    return('Unknown');
}

1;
__END__
