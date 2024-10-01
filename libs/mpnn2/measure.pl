#!/usr/bin/env perl

my $out = "run.time";

system "rm $out";
foreach my $r (1..40) {
    system "/usr/bin/time -a -o $out ./testmpnn ";
}

