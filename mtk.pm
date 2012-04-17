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
package mtk::Device;

use vars qw ($VERSION);
$VERSION = '0.1';

use warnings;
use strict;
use Carp;

use Data::Dumper;
use Date::Format;

use Device::SerialPort;

use IO::File;

use mtkGPX;

# Debug levels.
use constant LOG_EMERG   => 0;
use constant LOG_ALERT   => 1;
use constant LOG_CRIT    => 2;
use constant LOG_ERR     => 3;
use constant LOG_WARNING => 4;
use constant LOG_NOTICE  => 5;
use constant LOG_INFO    => 6;
use constant LOG_DEBUG   => 7;

# Default timeout for packet wait (sec).
use constant TIMEOUT => 6;
# Timeout for activity on device port (msec).
use constant TIMEOUT_IDLE_PORT => 5000;

# Log status is stored as a bitmask field;
use constant LOG_STATUS_AUTOLOG        => 0x0002; # AUTO_LOG mode (by criteria) ON/OFF
use constant LOG_STATUS_STOP_WHEN_FULL => 0x0004; # STOP/OVERLAP method when memory full
use constant LOG_STATUS_ENABLE         => 0x0100; # Device is in ENABLE_LOG (normal) state
use constant LOG_STATUS_DISABLE        => 0x0200; # Device entered DISABLE_LOG state (fail sectors >= 16)
use constant LOG_STATUS_NEED_FORMAT    => 0x0400; # Flash memory need format
use constant LOG_STATUS_FULL           => 0x0800; # Flash memory is full


#
# A record separator has one of the following types.
use constant SEP_TYPE_CHANGE_LOG_BITMASK    => 0x02;
use constant SEP_TYPE_CHANGE_LOG_PERIOD     => 0x03;
use constant SEP_TYPE_CHANGE_LOG_DISTANCE   => 0x04;
use constant SEP_TYPE_CHANGE_LOG_SPEED      => 0x05;
use constant SEP_TYPE_CHANGE_OVERLAP_STOP   => 0x06;
use constant SEP_TYPE_CHANGE_START_STOP_LOG => 0x07;my $SEP_TYPE_CHANGE_START_STOP_LOG = 0x07;
# Is there a char "*" separator between data and checksum?
use constant LOG_HAS_CHECKSUM_SEPARATOR => 1;

use enum qw(BITMASK:LOG_FORMAT_
	    UTC
	    VALID
	    LATITUDE
	    LONGITUDE
	    HEIGHT
	    SPEED
	    HEADING
	    DSTA
	    DAGE
	    PDOP
	    HDOP
	    VDOP
	    NSAT
	    SID
	    ELEVATION
	    AZIMUTH
	    SNR
	    RCR
	    MILLISECOND
	    DISTANCE);

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
	    BUTTON
	   );

my @LOG_TYPE = ( {
		  name => 'UTC',
		  size => 4,
		  decode => sub {
		    my $r = shift;
		    my $buffer = shift;
		    $r->{utc} = time2str('%Y-%m-%dT%H:%M:%SZ',unpack('L',$buffer) , 'GMT');
		    $r->{time} = unpack('L',$buffer);
		    #print "Hi from LOG_TYPE UTS $r->{utc}\n";
		  },
		},{
		    name => 'VALID',
		    size => 2,
		},{
		    name => 'LATITIDE',
		    size => 8
		},{
		    name => 'LONGITUDE',
		    size => 8
		},{
		    name => 'HEIGHT',
		    size => 4,
		},{
		    name => 'SPEED',
		    size => 4,
		},{
		    name => 'HEADING',
		    size => 4,
		},{
		    name => 'DSTA',
		    size => 2,
		},{
		    name => 'DAGE',
		    size => 4,
		},{
		    name => 'PDOP',
		    size => 2,
		},{
		    name => 'HDOP',
		    size => 2
		},{
		    name => 'VDOP',
		    size => 2
		},{
		    name => 'NSAT',
		    size => 2
		},{
		    name => 'SID',
		    size => 4, # sid(1) + sidinuse(1) + sidinview(2)
		},{
		    name => 'ELEVATION',
		    size => 2,
		},{
		    name => 'AZIMUTH',
		    size => 2,
		},{
		    name => 'SNR',
		    size => 2,
		},{
		    name => 'RCR',
		    size => 2,
		},{
		    name => 'MILLISECOND',
		    size => 1,
		},{
		    name => 'DISTANCE',
		    size => 8,
		},
	       );

my @valid_tbl = (
		 {
		  name => 'nofix',
		 }, {
		  name => 'sps',
		 }, {
		  name => 'dgps',
		 }, {
		  name => 'pps',
		 }, {
		  name => 'rtk',
		 }, {
		  name => 'frtk',
		 }, {
		  name => 'estimated',
		 }, {
		  name => 'manual',
		 }, {
		  name => 'simulator',
		 },
		);


use enum qw(BITMASK:RCR_ TIME SPEED DISTANCE BUTTON);

#
# Default data types.
use constant SIZEOF_BYTE            => 1;
use constant SIZEOF_FLOAT           => 4;
use constant SIZEOF_FLOAT3          => 3;
use constant SIZEOF_DOUBLE          => 8;
#
use constant SIZEOF_LOG_UTC         => 4; # long
use constant SIZEOF_LOG_VALID       => 2; # word
use constant SIZEOF_LOG_LATITUDE    => 8; # double
use constant SIZEOF_LOG_LONGITUDE   => 8; # double
use constant SIZEOF_LOG_HEIGHT      => 4; # float
use constant SIZEOF_LOG_SPEED       => 4; # float
use constant SIZEOF_LOG_HEADING     => 4; # float
use constant SIZEOF_LOG_DSTA        => 2; # word;
use constant SIZEOF_LOG_DAGE        => 4; # long;
use constant SIZEOF_LOG_PDOP        => 2; # word;
use constant SIZEOF_LOG_HDOP        => 2; # word;
use constant SIZEOF_LOG_VDOP        => 2; # word;
use constant SIZEOF_LOG_NSAT        => 2; # byte * 2;
use constant SIZEOF_LOG_SID         => 1; # byte;
use constant SIZEOF_LOG_SIDINUSE    => 1; # byte;
use constant SIZEOF_LOG_SATSINVIEW  => 2; # word;
use constant SIZEOF_LOG_ELEVATION   => 2; # word;
use constant SIZEOF_LOG_AZIMUTH     => 2; # word;
use constant SIZEOF_LOG_SNR         => 2; # word;
use constant SIZEOF_LOG_RCR         => 2; # word;
use constant SIZEOF_LOG_MILLISECOND => 2; # word;
use constant SIZEOF_LOG_DISTANCE    => 8; # double;

# Log data is retrieved in chunks of this size.
use constant SIZEOF_CHUNK           => 0x800;
use constant SIZEOF_SECTOR          => 0x10000;
use constant SIZEOF_SECTOR_HEADER   => 0x200;
use constant SIZEOF_SEPARATOR       => 0x10;

use constant RCD_METHOD_OVP         => 1;
use constant RCD_METHOD_STP         => 2;

# temp hack for now
my $device;
my $fp_log;

BEGIN {
}

END {
}

my $debug = 0;

sub new {
  my $class = shift;
  my $self = {};
  bless($self, $class);
  my $port;
  my $filename;

  my $baudrate = 115200;

  my %args = @_;

  if (exists $args{debug}) {
    $debug = $args{debug};
  }
  if (exists $args{baudrate}) {
    $baudrate = $args{baudrate};
  }
  if (exists $args{port}) {
    $self->{port} = $args{port};
    print "$args{'port'}  port name\n";

    #carp("Cannot open $port. Did you turn on the GPS device?\n") if (! -c $self->{port});
    $self->{device} = Device::SerialPort->new($self->{port})
      or croak "failed to open device $self->{port}";
    $self->{device}->baudrate($baudrate) or carp "fail setting baud rate";
    $self->{device}->parity('none')      or carp "fail setting parity";
    $self->{device}->databits(8)         or carp "fail setting databits";
    $self->{device}->stopbits(1)         or carp "fail setting stopbits";
    $self->{device}->handshake('none')   or carp "fail setting handshake";
    $self->{device}->write_settings      or carp "no settings";

    $device = $self->{device};
  }

  return $self;
}

#-------------------------------------------------------------------------
# Erase memory.
#-------------------------------------------------------------------------
sub erase_log {
  my $self = shift;
  print ">> Erasing log memory...\n";
  #  $self->query_device();
  $self->command('PMTK182,6,1',60);
}

#-------------------------------------------------------------------------
# Recover from disable log: ENABLE LOG and FORMAT LOG ALL.
# Also reset recording criteria.
#-------------------------------------------------------------------------
sub recover_log {
  my $self = shift;

  print ">> Recover from disable log: ENABLE LOG and FORMAT LOG ALL...\n";
  $self->command('PMTK182,10');
  $self->command('PMTK182,6,1');
}

#-------------------------------------------------------------------------
# Turn ON or OFF data logging.
#-------------------------------------------------------------------------
sub set_log_enable {
  my $self = shift;
  my $opt_l = shift;

  if ($opt_l eq 'on') {
    print ">> Switch recording to ON\n";
    $self->command('PMTK182,4');
  }
  if ($opt_l eq 'off') {
    print ">> Switch recording to OFF\n";
    $self->command('PMTK182,5');
  }
}

#-------------------------------------------------------------------------
# Set recording criteria: TIME, DISTANCE, SPEED.
#-------------------------------------------------------------------------
sub set_recording_criteria {
  my $self = shift;
  my $opt_r = shift;

  print ">> Setting recording criteria: time, distance, speed\n";
  my ($time, $distance, $speed) = split(/:/, $opt_r);
  print ">> Setting recording criteria: time ($time) distance ($distance) speed ($speed)\n";
  undef($time)     if ($time     eq '');
  undef($distance) if ($distance eq '');
  undef($speed)    if ($speed    eq '');
  $time     = int($time    ) if (defined($time));
  $distance = int($distance) if (defined($distance));
  $speed    = int($speed   ) if (defined($speed));
  if (defined($time) and (($time >= 1 and $time <= 999) or ($time == 0))) {
    $self->command(sprintf('PMTK182,1,3,%u', $time * 10));
  }
  if (defined($distance) and (($distance >= 10 and $distance <= 9999) or ($distance == 0))) {
    $self->command(sprintf('PMTK182,1,4,%u', $distance * 10));
  }
  if (defined($speed) and (($speed >= 10 and $speed <= 999) or ($speed == 0))) {
    $self->command(sprintf('PMTK182,1,5,%u', $speed * 10));
  }
}

#-------------------------------------------------------------------------
# Set recording method: OVERLAP or STOP (PMTK_LOG_REC_METHOD).
#-------------------------------------------------------------------------
sub set_record_method {
  my $self = shift;
  my $opt_m = shift;
  my $ret;

  if ((lc($opt_m) eq 'overlap') or (lc($opt_m) eq 'stop')) {
    if (lc($opt_m) eq 'overlap') {
      print ">> Setting method OVERLAP on memory full\n";
      $ret = $self->command('PMTK182,1,6,1');
    } else {
      print ">> Setting method STOP on memory full\n";
      $ret = $self->command('PMTK182,1,6,2');
    }
    if ($ret =~ m/PMTK001,182,1,(\d)/) {
      if ($1 ne '3') {
	printf(">> ERROR: Cannot set recording method\n");
      }
    }
  }
}

#-------------------------------------------------------------------------
# Set log format (PMTK_LOG_SETFORMAT).
#-------------------------------------------------------------------------
sub set_log_format {
  my $self = shift;
  my $opt_o = shift;
  my $ret;
  my $log_format;

  # Get current log format.
  $ret = $self->command('PMTK182,2,2');
  if ($ret =~ m/PMTK182,3,2,([0-9A-Za-z]+)\*/) {
    $log_format = hex($1);
    print ">> Setting log format: ",sprintf(" %08X\n",$log_format);
    $log_format = encode_log_format($log_format, $opt_o);
    $self->command(sprintf('PMTK182,1,2,%08X', $log_format));
  }
}

#-------------------------------------------------------------------------
# Query device current setting and log status
#-------------------------------------------------------------------------
sub query_device {

  my $self = shift;
  my $ret;
  my $next_write_address;
  my $log_status;
  my $expected_records_total;
  my $log_format;
  my $rec_method;
  my $fail_sectors;

print "Dude\n";
  # Query log format (PMTK_LOG_QUERY).
  $ret = $self->command('PMTK182,2,2');
  if ($ret =~ m/PMTK182,3,2,([0-9A-Za-z]+)\*/) {
    $log_format = hex($1);
    my ($size_wpt, $size_sat) = sizeof_log_format($log_format);
    printf("Log format: (%s) %s\n", $1, describe_log_format($log_format));
    printf("Size in bytes of each log record: %u + (%u * sats_in_view)\n", $size_wpt + 2, $size_sat);
  }

  # Query recording criteria: time, distance, speed (PMTK_LOG_QUERY).
  $ret = $self->command('PMTK182,2,3');
  if ($ret =~ m/PMTK182,3,3,([0-9]+)\*/) {
    printf("Logging TIME interval:     %6.2f s\n", $1 / 10);
  }
  $ret = $self->command('PMTK182,2,4');
  if ($ret =~ m/PMTK182,3,4,([0-9]+)\*/) {
    printf("Logging DISTANCE interval: %6.2f m\n", $1 / 10);
  }
  $ret = $self->command('PMTK182,2,5');
  if ($ret =~ m/PMTK182,3,5,([0-9]+)\*/) {
    printf("Logging SPEED limit:       %6.2f km/h\n", $1 / 10);
  }
  # Query recording method when full (OVERLAP/STOP).
  $ret = $self->command('PMTK182,2,6');
  if ($ret =~ m/PMTK182,3,6,([0-9]+)\*/) {
    $rec_method = $1;
    printf("Recording method on memory full: (%u) %s\n",
	   $rec_method, describe_recording_method($rec_method));
  }
  # Query LOG_STATUS (recording ON/OFF, ...).
  $ret = $self->command('PMTK182,2,7');
  if ($ret =~ m/PMTK182,3,7,([0-9A-Za-z]+)\*/) {
    $log_status = $1;
    printf("Log status: (%012b) %s\n", $log_status, describe_log_status($log_status));
    if ($log_status & LOG_STATUS_NEED_FORMAT) {
      printf("WARNING! Log status NEED_FORMAT, log data is not valid!\n");
    }
    if ($log_status & LOG_STATUS_DISABLE) {
      printf("WARNING! Log status DISABLE_LOG, may too many failed sectors!\n");
    }
  }
  # Query the RCD_ADDR (data log Next Write Address).
  # If device is in STOP mode, this is also the total memory used.
  # If it is in OVERLAP mode, there is old data beyond the NWA.
  $ret = $self->command('PMTK182,2,8');
  if ($ret =~ m/PMTK182,3,8,([0-9A-Za-z]+)\*/) {
    $next_write_address = hex($1);
    printf("Next write address: %u (0x%08X)\n", $next_write_address, $next_write_address);
  }
  # Query number of records stored in the log.
  $ret = $self->command('PMTK182,2,10');
  if ($ret =~ m/PMTK182,3,10,([0-9A-Za-z]+)\*/) {
    $expected_records_total = hex($1);
    printf("Number of records: %u\n", $expected_records_total);
  }
  # Query failed sectors (PMTK_RCD_FSECTOR).
  $ret = $self->command('PMTK182,2,11');
  if ($ret =~ m/PMTK182,3,11,([0-9A-Za-z]+)\*/) {
    $fail_sectors = $1;
    printf("Memory health status (failed sectors mask): %s\n", $fail_sectors);
  }
  # Compute the memory used by data log, round-up to the entire sector.
  #    if (($rec_method == $RCD_METHOD_OVP) or $opt_a) {
  #        # In OVERLAP mode we don't know where data ends; read the entire memory.
  #        $bytes_to_read = flash_memory_size($model_id);
  #    } else {
  # In STOP mode read from zero to Next Write Address.
  my $sectors  = int($next_write_address / SIZEOF_SECTOR);
  $sectors += 1 if (($next_write_address % SIZEOF_SECTOR) != 0);
  $self->{bytes_to_read} = $sectors * SIZEOF_SECTOR;
  #    }

  printf(">> %u (0x%08X) bytes of log data on device\n", $self->{bytes_to_read}, $self->{bytes_to_read});

}

sub command($$) {
  my $self = shift;
  my $send = shift;
  my $wait = shift;
  my $pkt;

  for (my $i=0; $i < 2; $i++) {
    packet_send($send);
    for (my $i=0; $i < 2; $i++) {
      $pkt = packet_wait($wait);
      if (defined($pkt)) {
	return $pkt;
      } else {
	warn "packet_wait failed retrying\n";
      }
    }
    warn "Resending packet $send\n";
  }
  # Ok just can't seem to get a response
  return undef;
}

sub get_log_data {
  my $self = shift;
  my %args = @_;
  my $pkt;
  #my $bytes_to_read = 1396768;
  #  my $bytes_to_read = 4 * 0x800;
  my $bytes_to_read = $self->{bytes_to_read};
  my $offset;
  my $non_written_sector_found = 0;
  my $filename;

  if ($args{file}) {
    $filename =  $args{file}.".bin";
  } else {
    $filename = time2str('%Y-%m-%d-%Z',time()).'.bin';
  }

  print "Using $filename to store binary data\n";
  $self->{mtk_fh} = new IO::File "> $filename" or croak "open failed $filename $!\n";

  $offset = 0;
  while ($offset < $bytes_to_read) {
    # Request log data (PMTK_LOG_REQ_DATA) from $offset to $bytes_to_read.

    $pkt = $self->command(sprintf('PMTK182,7,%08X,%08X', $offset, SIZEOF_CHUNK));

    # Resend the last packet request if we did not get a responce.
    next if(!defined($pkt));

    write_log_packet($pkt,$bytes_to_read,$self->{mtk_fh});

    $offset += SIZEOF_CHUNK;
    last if ($non_written_sector_found);
  }
}

sub log_time {
  time2str('%H:%M:%S', time());
}

#-------------------------------------------------------------------------
# Return the flash memory size upon model ID.
#-------------------------------------------------------------------------
sub flash_memory_size {
    my $model = shift;
    return( 8 * 1024 * 1024 / 8) if ($model == 0x1388); # 757/ZI v1        8 Mbit = 1 Mb
    return( 8 * 1024 * 1024 / 8) if ($model == 0x5202); # 757/ZI v2        8 Mbit = 1 Mb
    return(32 * 1024 * 1024 / 8) if ($model == 0x8300); # Qstartz BT-1200 32 Mbit = 4 Mb
    # 0x0051    i-Blue 737, Qstartz 810, Polaris iBT-GPS, Holux M1000
    # 0x0002    Qstartz 815
    # 0x001b    i-Blue 747
    # 0x001d    BT-Q1000 / BGL-32
    # 0x0131    EB-85A
    return(16 * 1024 * 1024 / 8); # 16Mbit -> 2Mb
}

#-------------------------------------------------------------------------
# Calculate the packet checksum: bitwise XOR of string's bytes.
#-------------------------------------------------------------------------
sub packet_checksum {
  my $pkt   = shift;
  my $len   = length($pkt);
  my $check = 0;
  my $i;

  for ($i = 0; $i < $len; $i++) {
    $check ^= ord(substr($pkt, $i, 1));
  }
  #printf("0x%02X\n", $check);
  return($check);
}

sub packet_send {
  my $pkt = shift;
  my $n;
  # Add the checksum to the packet.
  $pkt = $pkt . '*' . sprintf('%02X', packet_checksum($pkt));
  ### $ptk
  warn log_time(), sprintf(" TX packet(%04d) => %s\n",length($pkt),$pkt) if ($debug >= LOG_NOTICE);
  # Add the preamble and <CR><LF>.
  $pkt = '$' . $pkt . "\r\n";
  $n =  $device->write($pkt);

  #  if ($debug >= LOG_DEBUG) {
  #    printf("Writing %u bytes to device; actually written %u bytes\n", length($pkt), $n)
  #  }

  croak ("ERROR: Writing to device: $!") if ($n != length($pkt));
}

#-------------------------------------------------------------------------
# Read a packet from the device.
# Return the packet with PktType, DataField, "*" and Checksum.
#
#   Example: PMTK182,3,8,0004E69C*13
#
# The packet received has a leading Preample and a trailing <CR><LF>,
# example: $PMTK182,3,8,0004E69C*13<CR><LF>
#-------------------------------------------------------------------------
sub packet_read($) {

  my $timeout = shift;
  my $c;
  my $n;
  my $n_tot;
  my $pkt;
  my $previous_c;
  my $payload;
  my $checksum;

  # Timeout (in milliseconds) for activity on the port.
  $timeout = TIMEOUT_IDLE_PORT if (!defined($timeout));
  $device->read_const_time($timeout);

  # Wait packet preamble.
  $c = '';
  while ($c ne '$') {
    ($n, $c) = $device->read(1);
    die("ERROR: Reading from device: $!") if ($n != 1);
  }

  # Read until End Of Packet.
  $pkt = '';
  $previous_c = '';
  $n_tot = 0;
  while (1) {
    ($n, $c) = $device->read(1);
    #	print "n: $n c $c\n";
    die("ERROR: Reading from device: $!") if ($n != 1);
    $n_tot += $n;
    if ($c eq '$') {
      $pkt = '';
    } else {
      $pkt .= $c;
    }
    if (($c eq "\n") and ($previous_c eq "\r")) {
      last;
    }
    $previous_c = $c;
  }

  # Remove trailing <CR><LF>.
  $pkt = substr($pkt, 0, -2);
  if ($pkt !~ /^GP/ && $debug >= LOG_NOTICE) {
    warn log_time(), sprintf(" RX packet(%04d) <= %s\n", $n_tot, substr($pkt,0,40)) if ($debug >= LOG_NOTICE);
  }

  # Extract packet payload and checksum.
  $payload  = substr($pkt,  0, -3);
  $checksum = hex(substr($pkt, -2,  2));

  # Verify pcket checksum.
  if ($checksum ne packet_checksum($payload)) {
    printf("Packet checksum error: expected 0x%02X, computed 0x%02X\n",
	   $checksum, packet_checksum($payload)) if ($debug >= LOG_ERR);
    return undef;
  } else {
    return $pkt;
  }
}

#-------------------------------------------------------------------------
# Read packets from the device, wait for a cmd response packet (PMTK0001)
#-------------------------------------------------------------------------
sub packet_wait($) {
  #  my $self = shift;
  my $timeout  = shift;
  my $max_time;
  my $pkt;
  my $len;
  my $i;
  my $ret_pkt = undef;

  # Timeout (in seconds) for packet wait.
  $timeout = TIMEOUT if (!defined($timeout));
  $max_time = time() + $timeout;

  while (1) {

    $pkt = packet_read($timeout * 1000);

    if (!defined($pkt)) {
      print "No pkt timeout? retry read\n";
      $ret_pkt = undef;
      next;
    }

    if (my ($status,$csum) = $pkt =~ m{ PMTK001,[(\d+),]+(\d+)\*(\d+) }xms) {
      if ($status == 3) {
	return (defined($ret_pkt)? $ret_pkt : $pkt);
      } elsif ($status == 2) {
	warn "Valid command: action failed\n";
      } elsif ($status == 1) {
	warn "Invaid command\n";
      }
    }
    $ret_pkt = $pkt;
    last if (time() > $max_time);
  }

  warn " ERROR: packet_wait() timedout ",log_time(), "\n"
    if ($debug >= LOG_ERR);
  return undef;
}


#-------------------------------------------------------------------------
# Append received packets of log data to a file.
#
# Packet format is PMTK182,8,SSSSSSSS,PacketData*CC where:
#   SSSSSSSS   = Offset of first byte (hex value, 8 chars)
#   PacketData = Packet data (hex values, 4096 chars)
#   *          = Separator (1 char)
#   CC         = Checksum of data (hex value, 2 chars)
#-------------------------------------------------------------------------
sub write_log_packet($$$) {

  my $pkt = shift;
  my $bytes_to_read = shift;
  my $fp_log = shift;

  my $pkt_len;
  my $pkt_offset;
  my $log_offset;
  my $percent;
  my $i;

  # Save only datalog packets (PMTK_LOG_RESP_DATA).
  return if (substr($pkt, 0, 10) ne 'PMTK182,8,');

  # Check if packet length is OK and if chunk is in sequence order.
  $log_offset = tell($fp_log);
  $pkt_offset = hex(substr($pkt, 10, 8));
  $pkt_len    = (SIZEOF_CHUNK * 2) + 22;

  # Check sector headers for non-written pattern, set a flag to avoid reading unused memory.
  if (($log_offset % SIZEOF_SECTOR) == 0) {
    if (uc(substr($pkt, 19, SIZEOF_SEPARATOR * 2)) eq ('FF' x SIZEOF_SEPARATOR)) {
      printf("WARNING: Sector header at offset 0x%08X is non-written data\n", $log_offset);
      # $non_written_sector_found = 1;
    }
  }

  # We request $pkt_len at time, check how much we got.
  if (length($pkt) != $pkt_len) {
    printf("WARNING: Packet size error: expected %04X, got %04X\n", $pkt_len, length($pkt)) if ($debug >= LOG_WARNING);
  }

  if ($pkt_offset != $log_offset) {
    printf("ERROR: Chunk out of sequence: expected %08X, got %08X\n", $log_offset, $pkt_offset) if ($debug >= LOG_ERR);
  } else {
    printf("Saving log data, offset: 0x%08X\n", $log_offset) if ($debug >= LOG_INFO);
    # Convert the string of hex values into binary data.
    for ($i = 19; $i < ($pkt_len - 3); $i += 2) {
      #printf $fp_log chr(hex(substr($pkt, $i, 2)));
      print $fp_log chr(hex(substr($pkt, $i, 2)));
    }
    if (tell($fp_log) != $pkt_offset + SIZEOF_CHUNK) {
      print "Hmm bad write ", tell($fp_log), " offset: $pkt_offset SIZEOF_CHUNK\n";
    }
    $percent = ($log_offset / $bytes_to_read) * 100;
    printf("Saved log data: %6.2f%% file off 0x%x packet 0x%x\n", $percent,tell($fp_log),$pkt_offset);
  }
}

#-------------------------------------------------------------------------
# Parse raw binary log data and write GPX files.
#-------------------------------------------------------------------------
sub parse_log_data($) {

  my $self = shift;
  my $file = shift;

  my %args = @_;

  my $record_array = ();

  my $model_id;
  my $i;
  my $fp;
  my $buffer;
  my $log_len;
  my $new_offset;
  my $log_format;
  my $checksum;

  my $expected_records_sector  = 0;
  my $record_count_sector      = 0;
  my $record_count_total       = 0;
  my $gpx_in_trk               = 0;
  my $next_data_force_waypoint = 0;

  my $expected_records_total = 0xffffffff;

  my $record;

  use constant LEN_SECTOR_HEADER => 0x200;

  # Open the binary log file for reading.

  my $fh = new IO::File;
  $fh->open($file, "r") or croak "Error opending $file $!\n";
  $fh->seek(0,0) or croak "seek failed $!\n";

  $log_len = ($fh->stat)[7];	# extract the size from stat

  print "log_len $log_len\n";

  while (1) {

    $record = {};
    # Print current file position.
    printf("Reading offset %08X\n", $fh->tell()) if ($debug >= LOG_INFO);

    #-----------------------------------------
    # Process the begin of a sector (header).
    #-----------------------------------------
    if (($fh->tell() % SIZEOF_SECTOR) == 0) {
      # Reached the begin of a log sector (every 0x10000 bytes). Get the header (0x200 bytes).
      $fh->read($buffer, SIZEOF_SECTOR_HEADER) or carp "Read error $!\n";
      # What we will find in this log sector:
      ($expected_records_sector, $log_format) = parse_sector_header($buffer);
      next if (!defined($expected_records_sector) or !defined($log_format));
      $record_count_sector = 0;
    }

    #-----------------------------------------
    # Check if all the records has been read.
    #-----------------------------------------
    if ($record_count_total >= $expected_records_total) {
      printf("Total record count: %u\n", $record_count_total);
      last;
    }
    if ($record_count_sector >= $expected_records_sector) {
      $new_offset = SIZEOF_SECTOR * (int($fh->tell() / SIZEOF_SECTOR) + 1);
      if ($new_offset < $log_len) {
	# Log file contains more data (that is old data, being overwritten).
	$fh->seek($new_offset, 0);
	next;
      } else {
	# End Of File.
	#                if (!defined($opt_b)) {
	if (0) {
	  printf("ERROR: End of log file! Total record count: %u, expected %u\n",
		 $record_count_total, $expected_records_total);
	  last;
	} else {
	  printf("Total record count: %u\n", $record_count_total);
	  last;
	}
      }
    }

    #------------------------------------------------------------------
    # Check for:
    # - record separator:        AAAAAAAAAAAAAAXXYYYYYYYYBBBBBBBB
    # - non written space:       FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
    # - Holux M-241 separators : HOLUXGR241LOGGER
    #                            HOLUXGR241WAYPNT
    #------------------------------------------------------------------
    if (($log_len - $fh->tell()) >= SIZEOF_SEPARATOR) {

      $fh->read($buffer,  SIZEOF_SEPARATOR) or croak "Read failed $!\n";

      if ((substr($buffer, 0, 7) eq (chr(0xaa) x 7)) and (substr($buffer, -4) eq (chr(0xbb) x 4))) {
	#----------------------------------------------------------
	# Found a record separator.
	#----------------------------------------------------------

	my $separator_type = unpack('@7C',$buffer);
	my $separator_arg  = unpack('@8L',$buffer);

	if ($debug >= LOG_INFO) {
	  printf("Separator: %s, type: %s\n", uc(unpack('H*', $buffer)),
		 describe_separator_type($separator_type));
	}

	if ($separator_type == SEP_TYPE_CHANGE_LOG_BITMASK) {
	  $log_format = $separator_arg;
	  printf("New log bitmask: %s (0x%08X = %s)\n",
		 $separator_arg, $log_format, describe_log_format($log_format))
	    if ($debug >= LOG_INFO);
	}
	$record->{separator} = $separator_type;
	push (@{$record_array},$record);
	next;			# Search for the next record or record separator.

      } elsif ($buffer eq 'HOLUXGR241LOGGER') {
	#----------------------------------------------------------
	# Found Holux M-241 separator
	#----------------------------------------------------------
	printf("Separator: %s\n", $buffer) if ($debug >= LOG_INFO);
	$model_id = '0021';
	set_data_types($model_id);
	next;

      } elsif ($buffer eq 'HOLUXGR241WAYPNT') {
	#----------------------------------------------------------
	# Found Holux M-241 separator for waypoint.
	#----------------------------------------------------------
	printf("Separator: %s\n", $buffer) if ($debug >= LOG_INFO);
	$model_id = '0021';
	set_data_types($model_id);
	$next_data_force_waypoint = 1;
	next;

      } elsif ($buffer eq (chr(0xff) x SIZEOF_SEPARATOR)) {
	#----------------------------------------------------------
	# Found non-written space.
	#----------------------------------------------------------

	if ($expected_records_sector == 0xffff) {
	  # Sector record count = 0xffff means this is the currently writing sector.
	  # We found empty space, so we skip to sector end.
	  $new_offset = SIZEOF_SECTOR * (int($fh->tell() / SIZEOF_SECTOR) + 1);
	  if ($new_offset < $log_len) {
	    # Log file contains more data (that is old data, being overwritten).
	    $fp->seek($new_offset, 0);
	    #
	    # note the fact in the record array
	    $record->{unwritten} = 1;
	    push (@{$record_array},$record);
	    next;		# Search for the next record or record separator.
	  } else {
	    # End Of File.
	    printf("ERROR: End of log file. Total number of read records: %u, expected %u\n",
		   $record_count_total, $expected_records_total);
	    last;
	  }
	} else {
	  # ERROR! Non written space, but this is not the writing sector.
	  printf("ERROR: Non written space! Read %u records, expected %u\n",
		 $record_count_sector, $expected_records_sector);
	  last;
	}

      } else {
	# None of above, should be record data: rewind the file pointer so we can read it.
	$fh->seek(-(SIZEOF_SEPARATOR), 1);
      }
    }

    #-----------------------------------------
    # Read a log record.
    #-----------------------------------------
    $record_count_sector++;
    $record_count_total++;
    $checksum = 0;
    printf("Reading log sector: record %u (%u/%u total)\n",
	   $record_count_sector, $record_count_total, $expected_records_total)
      if ($debug >= LOG_INFO);

    # use this loop evenutally to decode log data
    #    my $record_size = 0;
    #    for (my $i=0; $i <=$#LOG_TYPE; $i++) {
    ##      printf ("0x%x %d\n",$log_format, $i);
    #      if ($log_format & (1 << $i)) {
    #	$record_size += $LOG_TYPE[$i]->{size};
    #	print "Record bit $i ", $LOG_TYPE[$i]->{name},"\n";
    #      }
    #      if (($log_format & LOG_FORMAT_SID) and ((1 << $i) & LOG_FORMAT_SID)) {
    #	do_end: while (1) {
    #	  for (my $si=14; $si <= 16; $si++) {
    #	    printf "\tSID: bit %d\n",$si;
    #	  }
    #	  $i = 16;
    #	  last do_end;
    #	}
    #      }
    #    }


    # Read each record field.
    if ($log_format & LOG_FORMAT_UTC) {
      $fh->read($buffer, SIZEOF_LOG_UTC);
      $checksum ^= packet_checksum($buffer);
      #$record->{utc} = time2str('%Y-%m-%dT%H:%M:%SZ',unpack('L',$buffer) , 'GMT');
      #my $record_utc = time2str('%Y-%m-%dT%H:%M:%SZ',unpack('L',$buffer) , 'GMT');
      #print "\trecord $record_utc\n";

      $LOG_TYPE[0]->{decode}($record,$buffer);

      printf("Record UTC: %s %s\n", uc(unpack('H*', $buffer)), $record->{utc})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_VALID) {
      $fh->read($buffer, SIZEOF_LOG_VALID);
      $checksum ^= packet_checksum($buffer);
      $record->{valid} = unpack('S',$buffer);
      printf("Record VALID: %s (0x%04X = %s)\n", uc(unpack('H*', $buffer)), $record->{valid},
	     describe_valid_mtk($record->{valid}))
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_LATITUDE) {
      $fh->read($buffer, SIZEOF_LOG_LATITUDE);
      $checksum ^= packet_checksum($buffer);
      $record->{latitude} = mtk2number($buffer, SIZEOF_LOG_LATITUDE);
      printf("Record LATITUDE: %s (%.9f)\n", uc(unpack('H*', $buffer)), $record->{latitude})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_LONGITUDE) {
      $fh->read($buffer, SIZEOF_LOG_LONGITUDE);
      $checksum ^= packet_checksum($buffer);
      $record->{longitude} = mtk2number($buffer, SIZEOF_LOG_LONGITUDE);
      printf("Record LONGITUDE: %s (%.9f)\n", uc(unpack('H*', $buffer)), $record->{longitude})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_HEIGHT) {
      $fh->read($buffer, SIZEOF_LOG_HEIGHT);
      $checksum ^= packet_checksum($buffer);
      $record->{height} = mtk2number($buffer, SIZEOF_LOG_HEIGHT);
      printf("Record HEIGHT: %s (%.6f)\n", uc(unpack('H*', $buffer)), $record->{height})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_SPEED) {
      $fh->read($buffer, SIZEOF_LOG_SPEED);
      $checksum ^= packet_checksum($buffer);
      $record->{speed} = mtk2number($buffer, SIZEOF_LOG_SPEED);
      printf("Record SPEED: %s (%.6f)\n", uc(unpack('H*', $buffer)), $record->{speed})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_HEADING) {
      $fh->read($buffer, SIZEOF_LOG_HEADING);
      $checksum ^= packet_checksum($buffer);
      $record->{heading} = mtk2number($buffer, SIZEOF_LOG_HEADING);
      printf("Record HEADING: %s (%.6f)\n", uc(unpack('H*', $buffer)), $record->{heading})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_DSTA) {
      $fh->read($buffer, SIZEOF_LOG_DSTA);
      $checksum ^= packet_checksum($buffer);
      $record->{dsta} = unpack('S',$buffer);
      printf("Record DSTA: %s (%u)\n", uc(unpack('H*', $buffer)), $record->{dsta})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_DAGE) {
      $fh->read($buffer, SIZEOF_LOG_DAGE);
      $checksum ^= packet_checksum($buffer);
      $record->{dage} = unpack('L',$buffer);
      printf("Record DAGE: %s (%u)\n", uc(unpack('H*', $buffer)), $record->{dage})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_PDOP) {
      $fh->read($buffer, SIZEOF_LOG_PDOP);
      $checksum ^= packet_checksum($buffer);
      $record->{pdop} = unpack('S',$buffer) / 100;
      printf("Record PDOP: %s (%.2f)\n", uc(unpack('H*', $buffer)), $record->{pdop})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_HDOP) {
      $fh->read($buffer, SIZEOF_LOG_HDOP);
      $checksum ^= packet_checksum($buffer);
      $record->{hdop} = unpack('S',$buffer) / 100;
      printf("Record HDOP: %s (%.2f)\n", uc(unpack('H*', $buffer)), $record->{hdop})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_VDOP) {
      $fh->read($buffer, SIZEOF_LOG_VDOP);
      $checksum ^= packet_checksum($buffer);
      $record->{vdop} = unpack('S',$buffer) / 100;
      printf("Record VDOP: %s (%.2f)\n", uc(unpack('H*', $buffer)), $record->{vdop})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_NSAT) {
      $fh->read($buffer, 2);
      $checksum ^= packet_checksum($buffer);
      ($record->{nsat_in_view},
       $record->{nsat_in_use}) = unpack('UU',$buffer);
      # $fh->read($buffer, SIZEOF_BYTE);
      # $checksum ^= packet_checksum($buffer);
      # $record->{nsat_in_use} = ord($buffer);
      printf("Record NSAT: in view %u, in use %u\n",
	     $record->{nsat_in_view},
	     $record->{nsat_in_use})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_SID) {
      while (1) {
	my $satdata= undef;
	# Even with zero satellites in view, we have at least one bunch of data.
	$fh->read($buffer, SIZEOF_LOG_SID);
	$checksum ^= packet_checksum($buffer);
	$satdata->{sid} = ord($buffer);
	### my $tmp_sid = unpack('C',$buffer);
	$fh->read($buffer, SIZEOF_LOG_SIDINUSE);
	$checksum ^= packet_checksum($buffer);
	$satdata->{inuse} = ord($buffer);
	### my $tmp_inuse = unpack('C', $buffer);
	$fh->read($buffer, SIZEOF_LOG_SATSINVIEW);
	$checksum ^= packet_checksum($buffer);
	$satdata->{inview} = unpack('S',$buffer);

	### warn " sid $satdata->{sid} inuse $satdata->{inuse} inview $satdata->{inview} \n";
	### warn " sid $tmp_sid inuse $tmp_inuse inview $satdata->{inview} \n";
	if ($satdata->{inview} == 0) {
	  printf("\tNo satellites in view\n") if ($debug >= LOG_DEBUG);
	  last;
	} else {
	  # Read data for this satellite.
	  printf("\tSats in view: %u, SatID %u in use: %u\n",
		 $satdata->{inview},
		 $satdata->{sid},
		 $satdata->{inuse})
	    if ($debug >= LOG_DEBUG);

	  if ($log_format & LOG_FORMAT_ELEVATION) {
	    $fh->read($buffer, SIZEOF_LOG_ELEVATION);
	    $checksum ^= packet_checksum($buffer);
	    $satdata->{elevation} = unpack('S',$buffer);
	    printf("\tSatellite ELEVATION: %s (%d)\n", uc(unpack('H*', $buffer)),
		   $satdata->{elevation})
	      if ($debug >= LOG_DEBUG);
	  }

	  if ($log_format & LOG_FORMAT_AZIMUTH) {
	    $fh->read($buffer, SIZEOF_LOG_AZIMUTH);
	    $checksum ^= packet_checksum($buffer);
	    $satdata->{azimuth} = unpack('S',$buffer);
	    printf("\tSatellite AZIMUTH:   %s (%u)\n", uc(unpack('H*', $buffer)), $satdata->{azimuth})
	      if ($debug >= LOG_DEBUG);
	  }

	  if ($log_format & LOG_FORMAT_SNR) {
	    $fh->read($buffer, SIZEOF_LOG_SNR);
	    $checksum ^= packet_checksum($buffer);
	    $satdata->{snr} = unpack('S',$buffer);
	    printf("\tSatellite SNR:       %s (%u)\n", uc(unpack('H*', $buffer)), $satdata->{snr})
	      if ($debug >= LOG_DEBUG);
	  }
	  push(@{$record->{satdata_array}},$satdata);
	}
	#	warn "sat array count ", $#{$record->{satdata_array}}+1 ,"\n";
	#	warn "sat array count ", scalar(@{$record->{satdata_array}}) ,"\n";
	#last if ($record->{satdata_count} >= $satdata->{inview});
	last if (scalar(@{$record->{satdata_array}}) >= $satdata->{inview});
      }
    }

    if ($log_format & LOG_FORMAT_RCR) {
      $fh->read($buffer, SIZEOF_LOG_RCR);
      $checksum ^= packet_checksum($buffer);
      $record->{rcr} = unpack('S',$buffer);
      printf("Record RCR: %s (%s)\n", uc(unpack('H*', $buffer)), describe_rcr_mtk($record->{rcr}))
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_MILLISECOND) {
      $fh->read($buffer, SIZEOF_LOG_MILLISECOND);
      $checksum ^= packet_checksum($buffer);
      $record->{millisecond} = unpack('S',$buffer);
      printf("Record MILLISECOND: %s (%u)\n", uc(unpack('H*', $buffer)), $record->{millisecond})
	if ($debug >= LOG_DEBUG);
    }

    if ($log_format & LOG_FORMAT_DISTANCE) {
      $fh->read($buffer, SIZEOF_LOG_DISTANCE);
      $checksum ^= packet_checksum($buffer);
      $record->{distance} = mtk2number($buffer, SIZEOF_LOG_DISTANCE);
      printf("Record DISTANCE: %s (%.9f)\n", uc(unpack('H*', $buffer)), $record->{distance})
	if ($debug >= LOG_DEBUG);
    }

    if (LOG_HAS_CHECKSUM_SEPARATOR) {
      # Read separator between data and checksum.
      $fh->read($buffer, SIZEOF_BYTE);
      if ($buffer ne '*') {
	printf("ERROR: Checksum separator error: expected char 0x%02X, found 0x%02X\n", ord('*'), ord($buffer));
	last;
      }
    }
    # Read and verify checksum.
    $fh->read($buffer, SIZEOF_BYTE);
    if ($checksum != ord($buffer)) {
      printf("ERROR: Record checksum error: expected 0x%02X, computed 0x%02X\n", ord($buffer), $checksum);
      last;
    }

    push (@{$record_array},$record);
    #    print Dumper($record);
  }				# while(1)

  $fh->close();

  return $record_array;
}


#-------------------------------------------------------------------------
# Read some bytes from the device and return them.
#-------------------------------------------------------------------------
#sub my_read {
#    my $handle   = shift;
#    my $length   = shift;
#    my $variable;
#    my $n = read($handle, $variable, $length);
#    printf("ERROR: Reading file: read %u bytes, expected %u\n", $n, $length) if ($n != $length);
#    return($variable);
#}


#-------------------------------------------------------------------------
# Parse the header (0x200 bytes) of a datalog sector (every 0x10000 bytes).
# Return the number of records in the sector and the log format.
#-------------------------------------------------------------------------
sub parse_sector_header {

  my $sector_header = shift;

  if ($debug >= LOG_NOTICE) {
    printf("\n");
    printf("Sector header:      %s\n", uc(unpack('H*', $sector_header))) if ($debug >= LOG_DEBUG);
  }

  # Check validity of sector header.
  my $separator   =     substr($sector_header, -6, 1); # Should be '*'
  my $checksum    = ord(substr($sector_header, -5, SIZEOF_BYTE)); # WARNING: It's not the XOR checksum!!!
  my $header_tail =     substr($sector_header, -4, 4); # Should be 0xBBBBBBBB

  my $ack = substr($sector_header, -4, 16);

  if (($separator ne '*') or ($header_tail ne (chr(0xBB) x 4))) {
    printf("ERROR: Invalid datalog sector header ->$separator<- ->$header_tail<- ->$ack<-\n");
    return(undef, undef);
  }

  my ($log_count,$log_format, $log_status,$log_period,$log_distance,$log_speed,$log_failsect1) =
    unpack('@0A2 @2A4 @6A2 @8A4 @12A4 @16A4 @20A32', $sector_header);

  my ($count,$format,$status,$period,$distance,$speed,$log_failsect) =
    unpack('S L S L L L A32',$sector_header);

  if ($debug >= LOG_NOTICE) {
    printf("Record count:        %s %s %u records\n",  uc(unpack('H*', $log_count)),    ' 'x6, $count);
    printf("Log format mask:     %s %s %032bb (%s)\n", uc(unpack('H*', $log_format)),   ' 'x2, $format, describe_log_format($format));
    printf("Log mode mask:       %s %s %016bb (%s)\n", uc(unpack('H*', $log_status)),   ' 'x6, $status, describe_log_status($status));
    printf("Log period:          %s %s %6.2f s\n",     uc(unpack('H*', $log_period)),   ' 'x2, $period   / 10);
    printf("Log distance:        %s %s %6.2f m\n",     uc(unpack('H*', $log_distance)), ' 'x2, $distance / 10);
    printf("Log speed:           %s %s %6.2f km/h\n",  uc(unpack('H*', $log_speed)),    ' 'x2, $speed    / 10);
    printf("Failed sectors mask: %s\n",                uc(unpack('H*', $log_failsect)));
    printf("\n");
  }

  return($count, $format);
}



#-------------------------------------------------------------------------
# Given a log format value (bitmask), return a description string.
#-------------------------------------------------------------------------
sub describe_log_format {

  my $log_format = shift;
  my $str = undef;

  for (my $i=0; $i <= $#LOG_TYPE; $i++) {
    if ($log_format & (1 << $i)) {
      $str .= $LOG_TYPE[$i]->{name}.',';
    }
  }

  chop($str);			# strip off the last comma
  return $str;
}

#-------------------------------------------------------------------------
#
#-------------------------------------------------------------------------
sub encode_log_format {

  my $log_format = shift;
  my $changes = shift;

  $log_format |= LOG_FORMAT_UTC          if ($changes =~ m/\bUTC\b/);
  $log_format |= LOG_FORMAT_VALID        if ($changes =~ m/\bVALID\b/);
  $log_format |= LOG_FORMAT_LATITUDE     if ($changes =~ m/\bLATITUDE\b/);
  $log_format |= LOG_FORMAT_LONGITUDE    if ($changes =~ m/\bLONGITUDE\b/);
  $log_format |= LOG_FORMAT_HEIGHT       if ($changes =~ m/\bHEIGHT\b/);
  $log_format |= LOG_FORMAT_SPEED        if ($changes =~ m/\bSPEED\b/);
  $log_format |= LOG_FORMAT_HEADING      if ($changes =~ m/\bHEADING\b/);
  $log_format |= LOG_FORMAT_DSTA         if ($changes =~ m/\bDSTA\b/);
  $log_format |= LOG_FORMAT_DAGE         if ($changes =~ m/\bDAGE\b/);
  $log_format |= LOG_FORMAT_PDOP         if ($changes =~ m/\bPDOP\b/);
  $log_format |= LOG_FORMAT_HDOP         if ($changes =~ m/\bHDOP\b/);
  $log_format |= LOG_FORMAT_VDOP         if ($changes =~ m/\bVDOP\b/);
  $log_format |= LOG_FORMAT_NSAT         if ($changes =~ m/\bNSAT\b/);
  $log_format |= LOG_FORMAT_SID          if ($changes =~ m/\bSID\b/);
  $log_format |= LOG_FORMAT_ELEVATION    if ($changes =~ m/\bELEVATION\b/);
  $log_format |= LOG_FORMAT_AZIMUTH      if ($changes =~ m/\bAZIMUTH\b/);
  $log_format |= LOG_FORMAT_SNR          if ($changes =~ m/\bSNR\b/);
  $log_format |= LOG_FORMAT_RCR          if ($changes =~ m/\bRCR\b/);
  $log_format |= LOG_FORMAT_MILLISECOND  if ($changes =~ m/\bMILLISECOND\b/);
  $log_format |= LOG_FORMAT_DISTANCE     if ($changes =~ m/\bDISTANCE\b/);

  $log_format &= ~LOG_FORMAT_UTC         if ($changes =~ m/-UTC\b/);
  $log_format &= ~LOG_FORMAT_VALID       if ($changes =~ m/-VALID\b/);
  $log_format &= ~LOG_FORMAT_LATITUDE    if ($changes =~ m/-LATITUDE\b/);
  $log_format &= ~LOG_FORMAT_LONGITUDE   if ($changes =~ m/-LONGITUDE\b/);
  $log_format &= ~LOG_FORMAT_HEIGHT      if ($changes =~ m/-HEIGHT\b/);
  $log_format &= ~LOG_FORMAT_SPEED       if ($changes =~ m/-SPEED\b/);
  $log_format &= ~LOG_FORMAT_HEADING     if ($changes =~ m/-HEADING\b/);
  $log_format &= ~LOG_FORMAT_DSTA        if ($changes =~ m/-DSTA\b/);
  $log_format &= ~LOG_FORMAT_DAGE        if ($changes =~ m/-DAGE\b/);
  $log_format &= ~LOG_FORMAT_PDOP        if ($changes =~ m/-PDOP\b/);
  $log_format &= ~LOG_FORMAT_HDOP        if ($changes =~ m/-HDOP\b/);
  $log_format &= ~LOG_FORMAT_VDOP        if ($changes =~ m/-VDOP\b/);
  $log_format &= ~LOG_FORMAT_NSAT        if ($changes =~ m/-NSAT\b/);
  $log_format &= ~LOG_FORMAT_SID         if ($changes =~ m/-SID\b/);
  $log_format &= ~LOG_FORMAT_ELEVATION   if ($changes =~ m/-ELEVATION\b/);
  $log_format &= ~LOG_FORMAT_AZIMUTH     if ($changes =~ m/-AZIMUTH\b/);
  $log_format &= ~LOG_FORMAT_SNR         if ($changes =~ m/-SNR\b/);
  $log_format &= ~LOG_FORMAT_RCR         if ($changes =~ m/-RCR\b/);
  $log_format &= ~LOG_FORMAT_MILLISECOND if ($changes =~ m/-MILLISECOND\b/);
  $log_format &= ~LOG_FORMAT_DISTANCE    if ($changes =~ m/-DISTANCE\b/);

  return($log_format);
}

#-------------------------------------------------------------------------
# Given a log format value (bitmask), return the record size in bytes.
#-------------------------------------------------------------------------
sub sizeof_log_format {

  my $log_format = shift;
  my $size_wpt = 0;
  my $size_sat = 0;

  $size_wpt += SIZEOF_LOG_UTC         if ($log_format & LOG_FORMAT_UTC);
  $size_wpt += SIZEOF_LOG_VALID       if ($log_format & LOG_FORMAT_VALID);
  $size_wpt += SIZEOF_LOG_LATITUDE    if ($log_format & LOG_FORMAT_LATITUDE);
  $size_wpt += SIZEOF_LOG_LONGITUDE   if ($log_format & LOG_FORMAT_LONGITUDE);
  $size_wpt += SIZEOF_LOG_HEIGHT      if ($log_format & LOG_FORMAT_HEIGHT);
  $size_wpt += SIZEOF_LOG_SPEED       if ($log_format & LOG_FORMAT_SPEED);
  $size_wpt += SIZEOF_LOG_HEADING     if ($log_format & LOG_FORMAT_HEADING);
  $size_wpt += SIZEOF_LOG_DSTA        if ($log_format & LOG_FORMAT_DSTA);
  $size_wpt += SIZEOF_LOG_DAGE        if ($log_format & LOG_FORMAT_DAGE);
  $size_wpt += SIZEOF_LOG_PDOP        if ($log_format & LOG_FORMAT_PDOP);
  $size_wpt += SIZEOF_LOG_HDOP        if ($log_format & LOG_FORMAT_HDOP);
  $size_wpt += SIZEOF_LOG_VDOP        if ($log_format & LOG_FORMAT_VDOP);
  $size_wpt += SIZEOF_LOG_NSAT        if ($log_format & LOG_FORMAT_NSAT);

  # Variable part, for each satellite:
  if ($log_format & LOG_FORMAT_SID) {
    $size_sat += SIZEOF_LOG_SID;
    $size_sat += SIZEOF_LOG_SIDINUSE;
    $size_sat += SIZEOF_LOG_SATSINVIEW;
    $size_sat += SIZEOF_LOG_ELEVATION   if ($log_format & LOG_FORMAT_ELEVATION);
    $size_sat += SIZEOF_LOG_AZIMUTH     if ($log_format & LOG_FORMAT_AZIMUTH);
    $size_sat += SIZEOF_LOG_SNR         if ($log_format & LOG_FORMAT_SNR);
  }

  $size_wpt += SIZEOF_LOG_RCR         if ($log_format & LOG_FORMAT_RCR);
  $size_wpt += SIZEOF_LOG_MILLISECOND if ($log_format & LOG_FORMAT_MILLISECOND);
  $size_wpt += SIZEOF_LOG_DISTANCE    if ($log_format & LOG_FORMAT_DISTANCE);

  return($size_wpt, $size_sat);

}

#-------------------------------------------------------------------------
# Return a string describing the log record separator type.
#-------------------------------------------------------------------------
sub describe_separator_type {
  my $sep_type = shift;
  return('CHANGE_LOG_BITMASK')    if ($sep_type == SEP_TYPE_CHANGE_LOG_BITMASK);
  return('CHANGE_LOG_PERIOD')     if ($sep_type == SEP_TYPE_CHANGE_LOG_PERIOD);
  return('CHANGE_LOG_DISTANCE')   if ($sep_type == SEP_TYPE_CHANGE_LOG_DISTANCE);
  return('CHANGE_LOG_SPEED')      if ($sep_type == SEP_TYPE_CHANGE_LOG_SPEED);
  return('CHANGE_OVERLAP_STOP')   if ($sep_type == SEP_TYPE_CHANGE_OVERLAP_STOP);
  return('CHANGE_START_STOP_LOG') if ($sep_type == SEP_TYPE_CHANGE_START_STOP_LOG);
  return('Unknown');
}

#-------------------------------------------------------------------------
# Return a string describing some field.
#-------------------------------------------------------------------------
sub describe_valid_mtk {
  my $valid = shift;

  for (my $i = 0; $i < 9; $i++) {
    if ((1 << $i) & $valid) {
      ### print "valid $valid bit $i ", $valid_tbl[$i]->{name},"\n";
      return $valid_tbl[$i]->{name};
    }
  }
  return('Unknown');
}

sub describe_rcr_mtk {
  my $rcr = shift;
  my $str = '';

  $str .= ',TIME'     if ($rcr & RCR_TIME);
  $str .= ',SPEED'    if ($rcr & RCR_SPEED);
  $str .= ',DISTANCE' if ($rcr & RCR_DISTANCE);
  $str .= ',BUTTON'   if ($rcr & RCR_BUTTON);

  return('Unknown') if ($str eq '');
  return(substr($str, 1));
}

#
#
#
sub describe_log_status {
  my $log_status = shift;
  my $str = '';
  if ($log_status & LOG_STATUS_AUTOLOG) {
    $str .= ',AUTOLOG_ON';
  } else {
    $str .= ',AUTOLOG_OFF';
  }
  if ($log_status & LOG_STATUS_STOP_WHEN_FULL) {
    $str .= ',STOP_WHEN_FULL';
  } else {
    $str .= ',OVERLAP_WHEN_FULL';
  }
  if ($log_status & LOG_STATUS_ENABLE) {
    $str .= ',ENABLE_LOG';
  }
  if ($log_status & LOG_STATUS_DISABLE) {
    $str .= ',DISABLE_LOG';
  }
  if ($log_status & LOG_STATUS_NEED_FORMAT) {
    $str .= ',NEED_FORMAT';
  }
  if ($log_status & LOG_STATUS_FULL) {
    $str .= ',FULL';
  }
  return(substr($str, 1));
}

sub describe_recording_method {
  my $val = shift;
  # Recording method: OVERLAP or STOP.
  return('OVERLAP')   if ($val == RCD_METHOD_OVP);
  return('STOP')      if ($val == RCD_METHOD_STP);
  return('Unknown');
}

#-------------------------------------------------------------------------
# Convert 24, 32 or 64 bit binary data into a number.
#-------------------------------------------------------------------------
sub mtk2number {
  my $buffer = shift;
  my $size = shift;
  if ($size == SIZEOF_FLOAT3) {
    return unpack('f', chr(0) . $buffer);
  } elsif ($size == SIZEOF_FLOAT) {
    return unpack('f', $buffer);
  } elsif ($size == SIZEOF_DOUBLE) {
    return unpack('d', $buffer);
  }
}

#
#
sub gpx_print_files {
  my $self = shift;
  my $r_pts = shift;

  my %args = @_;

  my $record;
  my $gpx_in_trk = 0;
  my $trk_out = undef;
  my $wpt_out = undef;
  my $next_data_force_waypoint;
  my $tz = undef;
  my $file_name;
  my $smart_trk = 0;

  if (exists $args{tzone}) {
    $tz = $args{tzone};
  }

  if (exists $args{trk_out}) {
    $trk_out = $args{trk_out};
    $trk_out->gpx_print_gpx_begin();
  } else {
    # use the first valid record time for the filename based on local timezone
    # not utc time, pass in tz for anything other than local time zone
    foreach $record (@{$r_pts}) {
      #      print Dumper($record);
      if (exists($record->{time})) {
	$file_name = time2str('%Y-%m-%d-%Z',$record->{time},$tz).'.gpx';
	if (-e $file_name) {
	  warn "File $file_name exists using ";
	  $file_name = time2str('%Y-%m-%d-%Z',$record->{time},$tz).'.gpx-2';
	  warn $file_name,"\n";
	}
	$trk_out = mtk::GPX->new(file => $file_name);
	last;
      }
    }
    $trk_out->gpx_print_gpx_begin();
  }

  # If we really want to separate out the waypoint pass in the file
  # otherwise we will just add them to the trk info
  if (exists $args{wpt_out}) {
    $wpt_out = $args{wpt_out};
    $wpt_out->gpx_print_gpx_begin();
  } else {
    $wpt_out = $trk_out;
  }

  foreach $record (@{$r_pts}) {

    if (exists($record->{time})) {
      my $name = time2str('%Y-%m-%d-%Z',$record->{time},$tz).'.gpx';
      if ($file_name !~ $name) {
	# warn "possible new file $name old file $file_name\n";
      }
    }

    # Start a new GPX <trkseg> on satellite lost.
    if (exists($record->{separator}) or
	exists($record->{unwritten})) {
      if ($gpx_in_trk and $smart_trk) {
	#warn Dumper($record);
	$trk_out->gpx_print_trk_end();
	$gpx_in_trk = 0;
      }
      next;
    }
    if (!exists($record->{valid})) {
      warn Dumper($record);
    }
    if (($record->{valid} == VALID_NOFIX)and $gpx_in_trk and $smart_trk) {
      $trk_out->gpx_print_trk_end();
      $gpx_in_trk = 0;
    }

    if (defined($record->{latitude}) and defined($record->{longitude})) {
      if ($next_data_force_waypoint) {
	### XXX
	print "XXX $next_data_force_waypoint\n";
	$wpt_out->gpx_print_wpt($record);
	$next_data_force_waypoint = 0;
      } else {
	# Write <trkpt> data in GPX file.
	if (($record->{valid} != VALID_NOFIX) and !($record->{rcr} & RCR_BUTTON)) {
	  if (! $gpx_in_trk) {
	    $trk_out->gpx_print_trk_begin($record);
	    $gpx_in_trk = 1;
	  }
	  $trk_out->gpx_print_trkpt($record);
	}
	# Write <wpt> data in GPX file.
	if (($record->{rcr} & RCR_BUTTON) and ($record->{valid} != VALID_NOFIX)) {
	  $wpt_out->gpx_print_wpt($record);
	}
      }
    }
  }

  # Eventually close the <trk> GPX tags.
  if ($gpx_in_trk) {
    $trk_out->gpx_print_trk_end();
    $gpx_in_trk = 0;
  }
  $trk_out->gpx_print_gpx_end();
  if ($trk_out != $wpt_out) {
    $wpt_out->gpx_print_gpx_end();
  }
}

1;
__END__
