#!/usr/bin/env perl


#loads fig file and convert polyline data (data type is 2) to 'iso' format.

my $usage = << "ENDUS";
usage: $0 <figFile> <maskFile> <scale>
converts map in xfig format to map in 'iso' format
objects at depth=49 are considered as border 
ENDUS

if (@ARGV < 3) {
	print STDERR $usage;
	exit;
}	

my ($infile, $outfile, $scale) = @ARGV;

open($fi,"<$infile") || die "$! $infile\n";
my @data = <$fi>;
close $fi;

#figure produced by xfig format sometimes contain multiple lines with polygon data. These lines begin with \t char.
my @tmp=();
chomp(@data);
my $wasTab = 0;
for(my $i=0;$i<@data;$i++) {
	my $l = $data[$i];
	if ($l =~ m/^\t.*/) {
		if ($wasTab == 0) {
			$wasTab = 1;
			push(@tmp,$l);
		} else {
			$l =~ s/\t+//g;
			$tmp[$#tmp] .= $l;
		}
	} else {
		push(@tmp,$l);
		$wasTab = 0;
	}
}

#$"="|\n";
#print "New data:\n";
#print "@tmp\n\n";
@data = @tmp;

my $isoFile = "$outfile.iso";
my $txtFile = "$outfile.txt";

open($foi,">$isoFile") || die "$! $isoFile\n";
open($fot,">$txtFile") || die "$! $txtFile\n";

for(my $i=9;$i<@data;$i++) {
	$data[$i] =~ s/^\s+//;
	my @s = split(/\s+/,$data[$i]);
	if ($s[0] == 2) { #polyline
        my $depth = $s[6];

		my $d = $data[$i+1];
		$d =~ s/^\s+//;
		my @c = split(/\s+/,$d);
        if ($depth == 49) {
            print $foi "[border]\n";
        }   else {
		    print $foi "[obstacle]\n";
        }
		for($j=0;$j<@c;$j+=2) {
			print $foi $scale*$c[$j]. " ".  -$scale*$c[$j+1] . "\n";
			print $fot $scale*$c[$j]. " ".  -$scale*$c[$j+1] . "\n";
		}
		print $fot "\n\n";
		$i++;
	} elsif ($s[0] == 1) { #elipse

	} elsif ($s[0] == 4) { #text

	}

}

close $fo;


