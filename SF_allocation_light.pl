#!/usr/bin/perl -w

##################################################
# Light SF allocation algorithm                  #
# author: Dr. Dimitrios Zorbas                   #
# email: dimzorbas@ieee.org                      #
# distributed under GNUv2 General Public Licence #
##################################################

use strict;
use POSIX;
use List::Util qw[min max];
use Time::HiRes qw( time );
use Math::Random;
use GD::SVG;

die "usage: ./SF_allocation_single.pl terrain_file!\n" unless (@ARGV == 1);

my ($terrain, $norm_x, $norm_y) = (0, 0, 0);
my %ncoords = ();
my %consumption = ();
my %node_sf = ();
my %data = ();
my @sfs = ();
my @sflist = ([7,-124,-122,-116], [8,-127,-125,-119], [9,-130,-128,-122], [10,-133,-130,-125], [11,-135,-132,-128], [12,-137,-135,-129]);
my $bw = 500;
my @pl = (100, 100, 100, 100, 100, 100);
my $stored_data = 10000;
my %time_per_sf = ();
my %slots = ();
my $guard = 0.04;
my $Ptx_w = 25 * 3.5 / 1000; # 25mA, 3.5V
my $Prx_w = 22 * 3.5 / 1000; # 22mA
my $sync_time = 0.05; # 50ms
my $avg_sf = 0;
my $generate_figure = 0;
my $simulate_transmissions = 1;
my $pdr = 1.000;

read_data();

my $max_f = 0;
my $max_data = 0;
foreach my $n (keys %ncoords){
	my $d0 = distance3d(sqrt($terrain)/2, $ncoords{$n}[0], sqrt($terrain)/2, $ncoords{$n}[1], 0, 10);
	my $f = min_sf($n, $d0);
	push(@sfs, [$n, $f]);
	if ($f > $max_f){
		$max_f = $f;
	}
	if ($data{$n} > $max_data){
		$max_data = $data{$n};
	}
}
my $start = time;
@sfs = sf_sorted(\@sfs);

# compute schedule
my $sched = optimize_times(\@sfs);
my $finish = time;

my $avg_cons = 0;
foreach my $tup (@$sched){
	my ($n, $sf, $t) = @$tup;
	print "$n -> $sf -> $t\n";
	$avg_sf += $sf;
}
$avg_sf /= (scalar @$sched);

print "--------------------\n";

my %nodes_per_sf = ();
foreach my $n (keys %node_sf){
	$nodes_per_sf{$node_sf{$n}} += 1;
}

print "# Time, slots & nodes per SF\n";
for (my $F=7; $F<=12; $F+=1){
	my $at = airtime($F);
	# if the bar length doesn't exceed the length imposed by the duty cycle restriction, set it to maximum
	if (($time_per_sf{$F} < ($at*100)) && ($time_per_sf{$F} > 0)){
		$slots{$F} = ceil( $at*100 / ($at + 2*$guard) );
		$time_per_sf{$F} = $slots{$F} * ($at + 2*$guard);
	}else{ # if it exceeds or no time
		$time_per_sf{$F} = ($at + 2*$guard) * $slots{$F};
	}
	$nodes_per_sf{$F} = 0 if (!exists $nodes_per_sf{$F});
	printf "%d : %.2f secs, %d slots (%d nodes)\n", $F, $time_per_sf{$F}, $slots{$F}, $nodes_per_sf{$F};
}
print "--------------------\n";
print "# Downlink time per SF\n";
my $down_time = 0;
my $last_pkt = 0;
for (my $F=7; $F<=12; $F+=1){
	next if ($nodes_per_sf{$F} == 0);
	my $downlink = int($nodes_per_sf{$F} / 126) * airtime($max_f, 125, 254)*10;
	$last_pkt = ($nodes_per_sf{$F} % 126) * 2 + 2; # last packet in bytes
	$downlink += airtime($max_f, 125, $last_pkt)*10;

	print "$F $downlink\n";
	$down_time += $downlink;
}
$down_time -= airtime($max_f, 125, $last_pkt)*9;
print "--------------------\n";

# compute the total time for all the transmissions
my $max_time = 0;
my $max_bar = undef;
foreach my $tup (@$sched){
	my ($n, $sf, $t) = @$tup;
	my $time = $slots{$sf} * (int($data{$n}/$pl[$sf-7])-1) * (airtime($sf)+2*$guard);
	my $rem = $data{$n} % $pl[$sf-7];
	if ($rem > 0){
		$time += ($slots{$sf} * (airtime($sf)+2*$guard) + airtime($sf, undef, $rem) + $guard);
	}else{
		$time += $nodes_per_sf{$sf} * (airtime($sf)+2*$guard) - $guard;
	}
	if ($time > $max_time){
		$max_time = $time;
		$max_bar = $sf;
	}
	$consumption{$n} += $sync_time * $Prx_w / 5; # sync consumption every 5 frames
	$consumption{$n} *= int($data{$n}/$pl[$sf-7]);
	$consumption{$n} += airtime($sf, undef, $rem) * $Ptx_w if ($rem > 0);
	$avg_cons += $consumption{$n};
}

draw_schedule() if ($generate_figure == 1);

print "# Statistics \n";
print "Longest time to deliver data: $max_time secs\n";
print "Avg SF: $avg_sf\n";
print "Downlink time: $down_time secs\n";
printf "Avg node consumption: %.5f J\n", $avg_cons/(scalar keys %ncoords);
printf "Processing time: %.6f secs\n", $finish-$start;
$pdr = compute_pdr() if ($simulate_transmissions == 1);
printf "PDR: %.3f\n", $pdr;


sub min_sf{
	my ($n, $d0) = @_;
	my $var = 3.57;
	my $G = 0.5; # rand(1);
	my ($dref, $Ptx, $Lpld0, $Xs, $gamma) = (40, 7, 95, $var*$G, 2.08);
	my $sf = undef;
	my $bwi = bwconv($bw);
	for (my $f=7; $f<=12; $f+=1){
		my $S = $sflist[$f-7][$bwi];
# 		my $Prx = $Ptx - ($Lpld0 + 10*$gamma * log10($d0/$dref) + $Xs);
# 		exit if ($Prx < $S); # just a test
		my $d = $dref * 10**( ($Ptx - $S - $Lpld0 - $Xs)/(10*$gamma) );
		if ($d > $d0){
			$sf = $f;
			$f = 13;
		}
	}
	if (!defined $sf){
		print "node $n unreachable!\n";
		exit;
	}
	return $sf;
}

sub optimize_times{
	my $sfs = shift;
	my @schedule = ();
	for (my $i=7; $i<=12; $i+=1){ # initiate the buckets
		$slots{$i} = 0;
		$time_per_sf{$i} = 0;
	}
	while (scalar @$sfs > 0){
		my $tuple = shift (@$sfs);
		my ($n, $f) = @$tuple;
# 		print "# I picked up $n with sf $f\n";
		
		# find the minimum available SF
		my $min_F = undef;
		my $min_time = 99999999999999999;
		for (my $F=$f; $F<=12; $F+=1){
			my $time = 0;
			if ($time_per_sf{$F} <= 100*airtime($F)){
				$time = 100*airtime($F) + (airtime($F) + 2*$guard);
			}else{
				$time = $time_per_sf{$F} + (airtime($F) + 2*$guard);
			}
			if ($time < $min_time){
				$min_time = $time;
				$min_F = $F;
			}
		}
		
		# allocate SF and slot
		$time_per_sf{$min_F} += airtime($min_F) + 2*$guard;
		push (@schedule, [$n, $min_F, $slots{$min_F}]);
		$slots{$min_F} += 1;
		
		$node_sf{$n} = $min_F;
		$consumption{$n} = airtime($min_F) * $Ptx_w;
	}
	return \@schedule;
}

sub sf_sorted{
	my $tuples = shift;
	my @new_sfs = ();
	my %examined = ();
	while (scalar keys %examined < scalar @$tuples){
		my $max_sf = 0;
		my $sel = undef;
		foreach my $tup (@$tuples){
			my ($n, $f) = @$tup;
			next if (exists $examined{$n});
			if ($f > $max_sf){
				$max_sf = $f;
				$sel = $n;
			}
		}
		$examined{$sel} = 1;
		push (@new_sfs, [$sel, $max_sf]);
	}
	return @new_sfs;
}

sub airtime{
	my $sf = shift;
	my $bandwidth = shift;
	my $payload = shift;
	my $cr = 1;
	my $H = 0;       # implicit header disabled (H=0) or not (H=1)
	my $DE = 0;      # low data rate optimization enabled (=1) or not (=0)
	my $Npream = 8;  # number of preamble symbol (12.25  from Utz paper)
	$bandwidth = $bw if (!defined $bandwidth);
	$payload = $pl[$sf-7] if (!defined $payload);
	
	if (($bandwidth == 125) && (($sf == 11) || ($sf == 12))){
		# low data rate optimization mandated for BW125 with SF11 and SF12
		$DE = 1;
	}
	
	if ($sf == 6){
		# can only have implicit header with SF6
		$H = 1;
	}
	
	my $Tsym = (2**$sf)/$bandwidth;
	my $Tpream = ($Npream + 4.25)*$Tsym;
	my $payloadSymbNB = 8 + max( ceil((8.0*$payload-4.0*$sf+28+16-20*$H)/(4.0*($sf-2*$DE)))*($cr+4), 0 );
	my $Tpayload = $payloadSymbNB * $Tsym;
	return ($Tpream + $Tpayload)/1000;
}

sub bwconv{
	my $bwi = 0;
	if ($bw == 125){
		$bwi = 1;
	}elsif ($bw == 250){
		$bwi = 2;
	}elsif ($bw == 500){
		$bwi = 3;
	}
	return $bwi;
}

sub read_data{
	my $terrain_file = $ARGV[0];
	open(FH, "<$terrain_file") or die "Error: could not open terrain file $terrain_file\n";
	my @coords = ();
	while(<FH>){
		chomp;
		if (/^# stats: (.*)/){
			my $stats_line = $1;
			if ($stats_line =~ /terrain=([0-9]+\.[0-9]+)m\^2/){
				$terrain = $1;
			}
			$norm_x = sqrt($terrain);
			$norm_y = sqrt($terrain);
		} elsif (/^# node coords: (.*)/){
			my $point_coord = $1;
			@coords = split(/\] /, $point_coord);
		}
	}
	close(FH);
	
	foreach my $line (@coords){
		$line = substr ($line, 0, -1) if (substr ($line, -1) eq "]");
		my @el = split(/ \[/, $line);
		my $n = shift(@el);
		my @coord = split(/ /, $el[0]);
		$ncoords{$n} = [$coord[0], $coord[1]];
		if (scalar @coord == 3){
			$data{$n} = ceil($coord[2]/$pl[0])*$pl[0]; # let's send full packets
		}else{
			$data{$n} = $stored_data;
# 			$data{$n} = (8 + int(rand(6)))*100;
		}
	}
}

sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2))+(($y1-$y2)*($y1-$y2)) );
}

sub distance3d {
	my ($x1, $x2, $y1, $y2, $z1, $z2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2))+(($y1-$y2)*($y1-$y2))+(($z1-$z2)*($z1-$z2)) );
}

sub draw_schedule{
	my $width = ceil($max_time / (airtime($max_bar) + 2*$guard)) * (int(airtime($max_bar)*230)+18) + 150;
	
	my $img   = GD::SVG::Image->new($width,400);
	my $white = $img->colorAllocate(255,255,255);
	my $black = $img->colorAllocate(0,0,0);
	my $blue  = $img->colorAllocate(0,0,255);
	my $red   = $img->colorAllocate(255,0,0);
	
	$img->string(gdGiantFont,10,10,"Light Scheduling",$black);
	$img->string(gdGiantFont,10,30,$max_time,$black);
	
	my $start_x = 50;
	my $start_y = 100;
	my $offset = 18;
	for (my $F=7; $F<=12; $F+=1){
		next if ($nodes_per_sf{$F} == 0);
		my $block_width = int(airtime($F)*230);
		my $y = $start_y + ($F-7)*20;
		my $frames = ceil($max_data/$pl[$F-7]);
		for (my $i=0; $i<$slots{$F}*$frames; $i+=1){
			my $x = $start_x + $i * ($block_width + $offset);
			if ($i % $slots{$F} == 0){
				$img->setThickness(5);
				$img->line($x-$offset/2, $y-10, $x-$offset/2, $y+20, $black);
				$img->setThickness(1);
			}
			$img->rectangle($x, $y, $x+$block_width, $y+10, $black);
		}
	}
	
	foreach my $tup (@$sched){
		my ($n, $sf, $t) = @$tup;
		
		my $block_width = int(airtime($sf)*230);
		my $x = $start_x + $t * ($block_width + $offset);
		my $y = $start_y + ($sf-7)*20;
		while ($data{$n} > 0){
			$block_width = airtime($sf, undef, $data{$n})*230 if ($data{$n} < $pl[$sf-7]);
			$img->filledRectangle($x, $y, $x+$block_width, $y+10, $blue);
			$data{$n} -= $pl[$sf-7];
			$x += $slots{$sf}*($block_width + $offset);
		}
	}
	
	$img->string(gdGiantFont,10,280,"Nodes per SF frame:",$black);
	my $i = 0;
	foreach my $F (sort { $a <=> $b } keys %time_per_sf){
		next if ($nodes_per_sf{$F} == 0);
		$img->string(gdGiantFont,10,300+$i*20,$nodes_per_sf{$F},$black);
		$i += 1;
	}
	
	my $image_file = "schedule-light.svg";
	open(FILEOUT, ">$image_file") or die "could not open file $image_file for writing!";
	binmode FILEOUT;
	print FILEOUT $img->svg;
	close FILEOUT;
}

sub compute_pdr{ # only the 1st transmmission
	my @thresholds = ([6,-16,-18,-19,-19,-20], [-24,6,-20,-22,-22,-22], [-27,-27,6,-23,-25,-25], [-30,-30,-30,6,-26,-28], [-33,-33,-33,-33,6,-29], [-36,-36,-36,-36,-36,6]); # capture effect power thresholds per SF[SF] for non-orthogonal transmissions
	my $var = 3.57;
	my ($dref, $Ptx, $Lpld0, $gamma) = (40, 7, 95, 2.08);
	my $dropped = 0;
	foreach my $tup (@$sched){
		my ($n, $sf, $t) = @$tup;
		my $collided = 0;
		my $lost = 0;
		my $G = rand(1);
		my $Xs = $var*$G;
# 		my $ran = rand($guard);
		my $d = distance3d(sqrt($terrain)/2, $ncoords{$n}[0], sqrt($terrain)/2, $ncoords{$n}[1], 10, 0);
		my $prx = $Ptx - ($Lpld0 + 10*$gamma * log10($d/$dref) + $Xs);
		if ($prx < $sflist[$sf-7][bwconv($bw)]){
			$collided = 1;
		}
		$G = rand(1);
		$Xs = $var*$G;
		$prx = 14 - ($Lpld0 + 10*$gamma * log10($d/$dref) + $Xs);
		if ($prx < $sflist[$sf-7][bwconv($bw)]){
			$lost = 1;
		}
		if (($collided == 1) || ($lost == 1)){
			$dropped += 1;
		}
	}
	
	return (1 - $dropped/(scalar @$sched));
}
