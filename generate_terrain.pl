#!/usr/bin/perl -w
#
# Script to create a 2D terrain of nodes

use strict;
use Math::Random;

(@ARGV==3) || die "usage: $0 <terrain_side_size_(m)> <num_of_nodes> <data_variance>\ne.g. $0 100 20 1000\n";

my $tx = $ARGV[0];
my $nodes = $ARGV[1];
my $var = $ARGV[2]; # use 0 for fixed data sizes

($tx < 1) && die "grid side must be higher than 1 meters!\n";
($nodes < 1) && die "number of nodes must be higher than 1!\n";

my @sensors;
my %coords;

for(my $i=1; $i<=$nodes; $i++){
	my ($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
	($x, $y) = ($x/10, $y/10);
	while (exists $coords{$x}{$y}){
		($x, $y) = (int(rand($tx*10)), int(rand($tx*10)));
		($x, $y) = ($x/10, $y/10);
	}
	
	my $data = random_uniform_integer(1, 10000-$var/2, 10000+$var/2);

	$coords{$x}{$y} = 1;
	push(@sensors, [$x, $y, $data]);
}

printf "# terrain map [%i x %i]\n", $tx, $tx;
print "# node coords:";
my $n = 1;
foreach my $s (@sensors){
	my ($x, $y, $data) = @$s;
	printf " %s [%.1f %.1f %d]", $n, $x, $y, $data;
	$n++;
}
print "\n";

print  "# generated with: $0 ",join(" ",@ARGV),"\n";
printf "# stats: nodes=%i terrain=%.1fm^2 node_sz=%.2fm^2\n", scalar @sensors, $tx*$tx, 0.1 * 0.1;
printf "# %s\n", '$Id: generate_grid.pl ??? jim $';
