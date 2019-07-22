#!/usr/bin/perl -w

use strict;
use IO::Socket;
use Socket;
use POSIX;
use List::Util qw[min max];
use Time::HiRes qw( time );

my @pl = (100, 100, 100, 100, 100, 100);
my $guard = 40; # in ms
my $sync_rate = 5; # sync periodicity in frames
my $sync_method = 1; # 1=time-based, 2=sync
my $bw = 500;
my %slots = ();
my %time_per_sf = ();
my $DATA = 10000;
my $gws = 0;

# read node file
my $node_file = "nodes.txt"; # registered nodes
open(FH, "<$node_file") or die "Error: could not open terrain file $node_file\n";
my @sfs = ();
my @min_sfs = ();
while(<FH>){
	chomp;
	my ($node, $sf) = (/([0-9]+) ([0-9]+)/);
	push (@sfs, [$node, $sf]);
	if (!grep {$_ == $sf} @min_sfs){
		push (@min_sfs, $sf);
		$gws += 1;
	}
}
close(FH);
my $nodes = scalar @sfs;
print "Number of nodes: $nodes\n";
print "Number of gateways: $gws\n";

# compute schedule
my $schedule = "";
$guard /= 1000; # convert ms to s
my $sched = light(\@sfs);
my $exp_time = ceil(100*airtime(7)*$DATA/$pl[0]);
foreach my $tup (@$sched){
	my ($n, $sf, $t) = @$tup;
	print "$n -> $sf -> $t\n";
	$schedule .= "$n $sf $t,";
}
my $start_time = time;
my $datetime = localtime();
print "started at $datetime\n";
$schedule = substr($schedule, 0, -1);
$guard *= 1000; # convert back s to ms
my $msg = "$guard:" . "$sync_method:" . "$sync_rate:" . $schedule;
print "$msg\n";

# create socket and send request
for (my $i=1; $i<=$gws; $i+=1){ # we consider one antenna per SF
	$| = 1;
	my $socket = new IO::Socket::INET(
		PeerHost => "192.168.0.$i",
		PeerPort => 8000,
		Proto => "tcp",
	) or die ("$!\n");
	$socket->send($msg);
}
#shutdown($socket, 1);
#$socket->close();

# wait for data
my $socket = new IO::Socket::INET (
	LocalHost => '0.0.0.0',
	LocalPort => '8000',
	Proto => 'tcp',
	Listen => 5,
	ReuseAddr => 1,
);
die "cannot create socket $!\n" unless $socket;
print "server waiting for client connection\n";

my %occur = ();
my $total = 0;
my $g = 0;
while($g < $gws)
{
	# waiting for a new client connection
	my $client_socket = $socket->accept();
	
	# get information about a newly connected client
	my $client_address = $client_socket->peerhost();
	my $client_port = $client_socket->peerport();
	print "connection from $client_address:$client_port\n";
	
	# read up to 4096 characters from the connected client
	my $data = "";
	$client_socket->recv($data, 4096);
	$datetime = localtime();
	print "$datetime: received data: $data\n";
	$g += 1;
	
	$data = substr($data, 1);
	$data = substr($data, 0, -1);
	my @occur = split(/, /, $data);
	for (my $id=11; $id<($nodes+11); $id+=1){
		my $ocr = shift(@occur);
		print "Node $id: $ocr\n";
		$total += $ocr;
	}
}
my $finish_time = time;
print "Total packets received: $total\n";
printf "PDR = %.3f\n", $total/($nodes * $DATA/$pl[0]);
printf "Data collection time: %.3f secs\n", $finish_time-$start_time;

sub light{
	my $sfs = shift;
	my @schedule = ();
	for (my $i=7; $i<=12; $i+=1){
		$slots{$i} = 0;
		$time_per_sf{$i} = 0;
	}
	while (scalar @$sfs > 0){
		my $tuple = shift (@$sfs);
		my ($n, $f) = @$tuple;
		
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
		$time_per_sf{$min_F} += airtime($min_F) + 2*$guard;
		push (@schedule, [$n, $min_F, $slots{$min_F}]);
		$slots{$min_F} += 1;
	}
	return \@schedule;
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
