#!/usr/bin/perl -w

##################################################
# Global SF allocation algorithm                 #
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

die "usage: ./SF_allocation_global.pl terrain_file!\n" unless (@ARGV == 1);

my ($terrain, $norm_x, $norm_y) = (0, 0, 0);
my %ncoords = ();
my %rem_data = ();
my %can_transmit = ();
my %consumption = ();
my %avg_sf = ();
my %ntrans = ();
my @sfs = ();
my @sflist = ([7,-124,-122,-116], [8,-127,-125,-119], [9,-130,-128,-122], [10,-133,-130,-125], [11,-135,-132,-128], [12,-137,-135,-129]);
my $bw = 500;
my @pl = (100, 100, 100, 100, 100, 100);
my %time_per_sf = ();
my %slots = ();
my %sf_occurances = ();
my $guard = 0.01; # this must be adjusted according to the frame size
my $sync_size = 4; # 4 bytes for synchronisation
my $Ptx_w = 75 * 3.3 / 1000; # 75mA, 3.3V
my $Prx_w = 45 * 3.3 / 1000; # 45mA
my $generate_figure = 0;
my $simulate_transmissions = 1; # perform a path-loss and collision test
my $stored_data = 1000;

# for statistics
my $max_time = 0;
my $max_bar = 0;
my $avg_cons = 0;
my $gavg_sf = 0;
my $pdr = 1.000;

read_data();

foreach my $n (keys %ncoords){
	my $d0 = distance3d(sqrt($terrain)/2, $ncoords{$n}[0], sqrt($terrain)/2, $ncoords{$n}[1], 10, 0);
	push(@sfs, [$n, min_sf($n, $d0)]);
	$can_transmit{$n} = 0;
}
my $start = time;
@sfs = sf_sorted(\@sfs);

# compute schedule
my $sched = optimize_times();
my $finish = time;
foreach my $tup (@$sched){
	my ($n, $sf, $t) = @$tup;
	print "$n -> $sf -> $t\n";
}

for (my $F=7; $F<=12; $F+=1){
	my $s = 0;
	if (scalar keys %{$slots{$F}} == 0){
		$sf_occurances{$F} = 0;
		$time_per_sf{$F} = 0;
		next;
	}
	$s = max(keys %{$slots{$F}});
	$time_per_sf{$F} = $s * (airtime($F) + 2*$guard) - $guard;
	if ($time_per_sf{$F} > $max_time){
		$max_time = $time_per_sf{$F};
		$max_bar = $F;
	}
	printf "%d : %.2f secs, %d slots, (%d transmissions)\n", $F, $time_per_sf{$F}, $s, $sf_occurances{$F};
}

foreach my $n (keys %ncoords){
	$avg_cons += $consumption{$n};
	$gavg_sf += $avg_sf{$n}/$ntrans{$n};
}

print "--------------------\n";
print "Longest time to deliver data: $max_time\n";
printf "Processing time: %.5f secs\n", $finish-$start;
printf "Avg node consumption: %.6f J\n", $avg_cons/(scalar keys %ncoords);
printf "Avg SF: %.3f\n", $gavg_sf/(scalar keys %ncoords);
$pdr = compute_pdr() if ($simulate_transmissions == 1);
printf "PDR: %.3f\n", $pdr;
print "--------------------\n";

draw_schedule() if ($generate_figure);

sub min_sf{
	my ($n, $d0) = @_;
	my $var = 3.57;
	my $G = 0.5; #rand(1);
	my ($dref, $Ptx, $Lpld0, $Xs, $gamma) = (40, 7, 95, $var*$G, 2.08);
	my $sf = undef;
	my $bwi = bwconv($bw);
	for (my $f=7; $f<=12; $f+=1){
		my $S = $sflist[$f-7][$bwi];
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
	my @schedule = ();

	my @examined = ();
	my @temp_sfs = @sfs;
	while (scalar @examined < scalar @sfs){
		if (scalar @temp_sfs == 0){
			@temp_sfs = @sfs;
# 			print "---\n";
		}
		my $tuple = shift(@temp_sfs);
		my ($n, $sf) = @$tuple;
		next if ($rem_data{$n} <= 0);
# 		print "# picked $n with SF $sf and rem. data $rem_data{$n}\n";
		
		# find the best SF pot
		my $min_F = undef;
		my $min_time = 99999999999999999;
		my $min_slot = undef;
		for (my $F=$sf; $F<=12; $F+=1){
			my $at = airtime($F);
			$can_transmit{$n} += $guard if ($can_transmit{$n} == 0);
			# convert allowance time to a slot number in pot
			my $node_slot = ceil( ($can_transmit{$n}-$guard) / ($at + 2*$guard) ) + 1;
			# search for the first available slot in pot
			while (exists $slots{$F}{$node_slot}){
				$node_slot += 1;
			}
			# convert this slot back to an actual time
			my $time = 0;
			if ($rem_data{$n} <= $pl[$F-7]){
				$time = $node_slot * ($at + 2*$guard) + ($at + 2*$guard);
			}else{
				$time = $node_slot * ($at + 2*$guard) + 100*$at; # mix together the current avail. slot and the next feasible one
			}
			if ($time < $min_time){
				$min_time = $time;
				$min_F = $F;
				$min_slot = $node_slot;
			}
		}
# 		print "\t selected SF $min_F for $n\n";
		$avg_sf{$n} += $min_F;
		$ntrans{$n} += 1;
		$sf_occurances{$min_F} += 1;
		
		# allocate SF and slot
		$slots{$min_F}{$min_slot} = 1;
		push (@schedule, [$n, $min_F, $min_slot]);
		my $payl = $pl[$min_F-7];
		$payl = $rem_data{$n} if ($rem_data{$n} < $pl[$min_F-7]);
		$consumption{$n} += airtime($min_F, $payl) * $Ptx_w + (airtime($min_F, $sync_size)+$guard) * $Prx_w; # not 100% precise
		$rem_data{$n} -= $payl;
		if ((($min_slot-1)*(airtime($min_F)+2*$guard)+$guard) < $can_transmit{$n}){ # duty cycle check
			exit;
		}
		if ($rem_data{$n} <= 0){
			push (@examined, $n);
		}else{
			$can_transmit{$n} = ($min_slot-1)*(airtime($min_F)+2*$guard) + 100*airtime($min_F);
# 			print "$n $can_transmit{$n}\n";
		}
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
	my $cr = 1;
	my $H = 0;       # implicit header disabled (H=0) or not (H=1)
	my $DE = 0;      # low data rate optimization enabled (=1) or not (=0)
	my $Npream = 8;  # number of preamble symbol (12.25  from Utz paper)
	my $payload = shift;
	$payload = $pl[$sf-7] if (!defined $payload);
	
	if (($bw == 125) && (($sf == 11) || ($sf == 12))){
		# low data rate optimization mandated for BW125 with SF11 and SF12
		$DE = 1;
	}
	
	if ($sf == 6){
		# can only have implicit header with SF6
		$H = 1;
	}
	
	my $Tsym = (2**$sf)/$bw;
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
			$rem_data{$n} = ceil($coord[2]/$pl[0])*$pl[0]; # let's send full packets
		}else{
			$rem_data{$n} = $stored_data;
# 			$rem_data{$n} = (8 + int(rand(6)))*100;
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

sub compute_pdr{
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
			$lost = 1;
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

sub draw_schedule{
	my $width = ceil($max_time / (airtime($max_bar) + 2*$guard)) * (int(airtime($max_bar)*230)+18) + 150;
	
	my $img   = GD::SVG::Image->new($width,400);
	my $white = $img->colorAllocate(255,255,255);
	my $black = $img->colorAllocate(0,0,0);
	my $blue  = $img->colorAllocate(0,0,255);
	my $red   = $img->colorAllocate(255,0,0);
	
	$img->string(gdGiantFont,10,10,"Global Scheduling",$black);
	$img->string(gdGiantFont,10,30,"Schedule length: $max_time secs",$black);
	$img->string(gdGiantFont,10,50,"BW$bw, min_guard=$guard secs, SF7-12",$black);
	
	my $start_x = 50;
	my $start_y = 100;
	my $offset = 18;
	for (my $F=7; $F<=12; $F+=1){
		next if ($sf_occurances{$F} == 0);
		$img->string(gdGiantFont,5,$start_y+($F-7)*20-3,"SF$F",$black);
		
		my $block_width = int(airtime($F)*230);
		my $y = $start_y + ($F-7)*20;
		
		my $s = max(keys %{$slots{$F}});
		my $x = $start_x;
		$img->setThickness(5);
		$img->line($x-$offset/2, $y-10, $x-$offset/2, $y+20, $black);
		$img->setThickness(1);
		for (my $i=0; $i<$s; $i+=1){
			$x = $start_x + $i * ($block_width + $offset);
			$img->rectangle($x, $y, $x+$block_width, $y+10, $black);
		}
		
		$img->setThickness(5);
		$img->line($x+$block_width+$offset/2, $y-10, $x+$block_width+$offset/2, $y+20, $black);
		$img->setThickness(1);
	}
	
	foreach my $tup (@$sched){
		my ($n, $sf, $t) = @$tup;
		my $block_width = int(airtime($sf)*230);
		my $x = $start_x + ($t-1) * ($block_width + $offset);
		my $y = $start_y + ($sf-7)*20;
		$img->filledRectangle($x, $y, $x+$block_width, $y+10, $blue);
	}
	
	$img->string(gdGiantFont,10,230,"Transmissions per SF frame:",$black);
	my $i = 0;
	for (my $F=7; $F<=12; $F+=1){
		next if ($sf_occurances{$F} == 0);
		$img->string(gdGiantFont,10,250+$i*20,"SF$F: $sf_occurances{$F}",$black);
		$i += 1;
	}
	
	my $image_file = "schedule-global.svg";
	open(FILEOUT, ">$image_file") or die "could not open file $image_file for writing!";
	binmode FILEOUT;
	print FILEOUT $img->svg;
	close FILEOUT;
}
