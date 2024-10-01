#!/usr/bin/env perl

if (@ARGV<  3) {
	print STDERR "usage: $0 <numberFile> <headerFile> <variableName\n";
	print STDERR "converts numbers in numberFile to const int array and saves it to headerFile\n";
	print STDERR "<numerFile> ..   file with numbers\n";
	print STDERR "<headerFile> ..  name of generated header file \n";
	print STDERR "<variableName>   name of the variable that will stores the values\n";
	exit;
}

my ($nf, $hf,$vn) = @ARGV;

open(my $fi,"<$nf") || die "$! $nf\n";
my @data = <$fi>;
close $fi;
chomp @data;

open(my $fo,">$hf") || die "$! $hf\n";

print $fo "#ifndef _COLOR_${vn}_MAP_\n";
print $fo "#define _COLOR_${vn}_MAP_\n";
print $fo "// file generated " . localtime() . "by $0\n";
print $fo "// file contains numbers loaded from $hf\n";

$tmp = $data[0];
$tmp=~s/^\s+//;
my @a = split(/\s+/,$tmp);

print $fo "const double $vn"."[".scalar(@data) . "][".scalar(@a)."] = {\n";

for(my $i=0;$i<@data;$i++) {
	$data[$i]=~s/^\s+//;
	my @c = split(/\s+/,$data[$i]);
	$"=',';
	print $fo "{ @c }";
	if ($i < @data-1) {
		print $fo ",";
	}
	print $fo "\n";
}
print $fo "};\n";
print $fo "#endif\n\n";


