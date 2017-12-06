#! /usr/bin/perl -w

use strict;
use warnings;

open(my $fin, '<', $ARGV[0]) or die "Cannot open $ARGV[0]: $!";
binmode($fin);
open(my $fout, '>', $ARGV[1]) or die "Cannot create $ARGV[1]: $!";
binmode($fout);

my $n;
my $bytes_in;
my $bytes_out;
while (($n = read($fin, $bytes_in, 8)) == 8) {
    my @c = split('', $bytes_in);
    my $bytes_out = join('', $c[7], $c[6], $c[5], $c[4], $c[3], $c[2], $c[1], $c[0]);
    print $fout $bytes_out;
}

if ($n > 0) {
    print $fout $bytes_in;
}

close($fout);
close($fin);