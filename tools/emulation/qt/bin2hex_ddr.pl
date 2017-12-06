#! /usr/bin/perl -w

($in, $out, $addrh) = @ARGV;
die "Missing input file name.\n" unless $in;
die "Missing output file name.\n" unless $out;
$addrh = "0x61008000" unless $addrh;
$addr = hex($addrh);
if ($addr >= 0x60000000) {
    $addr -=0x60000000;
}
$addr /= 2;
$byteCount = 0;
open(IN, "< $in");
binmode(IN);
open(OUT, "> tmp.hex");
$mem="";
while (read(IN,$b,1)) {
    $n = length($b);
    $byteCount += $n;
    $byte = unpack("H2", $b);
    $mem = $byte.$mem;
    if(($byteCount % 2) == 0) {
        print OUT "$mem\n";
        $mem = "";
    }
}
if(($byteCount % 2) != 0) {
    while(($byteCount % 2) != 0) {
        $mem = "00".$mem;
        $byteCount++;
    }
    print OUT "$mem\n";
}
close(IN);
close(OUT);
#print "Number of bytes converted = $byteCount\n";

open(IN, "< tmp.hex");
open(OUT, "> $out");
printf OUT ("\$INSTANCE ddr_device_0_0.memcore\n");
printf OUT ("\$RADIX   HEX\n");
#printf OUT ("\$ADDRESS   $addr    BFFF\n");
while (<IN>) {
    printf OUT ("%x $_",$addr);
    $addr++;
}
#while($addr < 0xC000) {
#   printf OUT ("%x 0000\n",$addr);
#   $addr++;
#}

system("rm tmp.hex");
