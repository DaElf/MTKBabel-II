use strict;
use warnings;
use ExtUtils::MakeMaker;

eval 'use ExtUtils::MakeMaker::Coverage';
warn "Optional ExtUtils::MakeMaker::Coverage not available\n" if $@;

WriteMakefile(
	      ( MM->can( 'signature_target' ) ? ( SIGN => 1 ) : () ),
	      NAME          => 'mtk::Device',
	      AUTHOR        => 'Russell Cattelan <cattelan@thebarn.com>',
	      VERSION_FROM  => 'mtk.pm',
	      ABSTRACT_FROM => 'mtk.pm',
	      LICENSE       => 'GPL',
	      PL_FILES      => {},
	      PREREQ_PM     => {
				'Date::Format'   => 0,
				'Device::SerialPort' => 0,
				'IO::File' => 0,
			       },
	      dist  => { COMPRESS => 'gzip -9f', SUFFIX => 'gz', },
	      clean => { FILES    => '' },
	     );
