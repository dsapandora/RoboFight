
#using perl
use strict;
use warnings;

# quit unless we have the correct number of command-line args
my $num_args = $#ARGV + 1;
if ($num_args != 1) {
	print "\nNumber of arguments: $num_args\n";
	print "\nUsage: MotionParser.pl foo.mtn \n";
	exit;
}

# we got three command line args, so assume the second is the .mtn file
my $motion_file = $ARGV[0];
print "\nMotion Input File: $motion_file\n";
my $output_file = "motion.h";
print "Output File: $output_file\n";

# Attempt to open the input and output files
open(my $in, "<", $motion_file) or die "Can't open input motion file: $!";
open(my $out, ">", $output_file) or die "Can't open output header file: $!";

# Read the 3 or 4 header lines from the Motion File and perfrom basic checks
my $line = <$in>;
chomp($line);
# Check it's a motion file
if( $line !~ /motion/ ) {
	print STDERR "$motion_file - This is not a motion file!\n";
	close $in;
	exit;
}

# check the version number to decide 3 or 4 header lines
$line = <$in>;
chomp($line);
$line  =~ s/version=//; 
my $file_version = $line;
print "File Version = $file_version \n";

$line = <$in>;
chomp($line);
# Count the number of enabled Dynamixel servos and store the ID's
$line =~ s/enable=//; 
my @servos = split / /, $line;
my $i = 0;
my $active_servos = 0;
foreach (@servos) {
	$i += 1;
	if($_ == "1") { 
		$active_servos += 1;
	}
}
# number of packed servo values as well
my $packed_active_servos = 0;
# not an elegant way to do this, but it works :)
if($active_servos > 0) {
	use integer;
	$packed_active_servos = $active_servos / 3;
}

if( $file_version == 1.01 ) { 
	# version 1.01 files have a motor type header line which we ignore
	$line = <$in>;
}

# Start the output for the motion.h file with the top level comments
print $out "#ifndef __MOTION_H_\n";
print $out "#define __MOTION_H_\n";
print $out "/* ==========================================================================\n";
print $out " \n"; 
print $out "   COMPONENT:        Global motion variable definitions\n";
print $out "   AUTHOR:           Ariel Vernaza\n";
print $out "   DESCRIPTION:      This component defines the motion pages and related\n";
print $out "                     global variables. Auto-generated from the *.mtn file.\n";
print $out " \n"; 
print $out "========================================================================== */\n";
print $out " \n";
print $out "#include <stdint.h>\n";
print $out "#include <avr/pgmspace.h>\n";
print $out "#include \"global.h\"\n";
 
# Now output the servos as an array so we can test validity of hardware config
print $out " \n";
print $out "// Array showing which Dynamixel servos are enabled in motion file \n";
print $out "const uint8 AX12_ENABLED[MAX_AX12_SERVOS] = {";
$i = 0;
foreach (@servos) { 
	if($i != @servos-1) { 
		print $out "$_,"; 
	} else { 
		print $out "$_}; \n"; 
	}
	$i += 1;
}
print $out "\n";

# Next we need to define the struct for a motion page
my $total_steps = 0;
my $total_pages = 0;
my @motion_pages = ();
# we also want to keep track of the min and max value for each servo
my @servo_min_val = ();
for($i = 0; $i < $active_servos; $i++) { $servo_min_val[$i] = 512; }
my @servo_max_val = ();
for($i = 0; $i < $active_servos; $i++) { $servo_max_val[$i] = 512; }
# Start processing the 255 motion pages
for ($i = 1; $i <= 255; $i++) {
	# check that the first line is a page_begin line
	$line = <$in>;
	chomp($line);
	if( $line !~ /page_begin/ ) {
		print STDERR "Motion Page $i - Page Begin not found!\n";
		close $in;
		exit;
	}
	
	# Extract the page name (if it exists)
	$line = <$in>;
	chomp($line);
	$line =~ s/name=//; 
	my $page_name = $line;
	
	# Extract the joint flexibility values
	$line = <$in>;
	chomp($line);
	$line =~ s/compliance=//; 
	my @joints = split / /, $line;
	my @flex_values = ();
	my $j = 0;
	foreach (@joints) {
		# now we filter the values for the active servos only
		if( $j <=25 ) {
			if ($servos[$j] == "1" ) { 
				push( @flex_values, $joints[$j] );
			}
		}
		$j += 1;
	}

	
	# Extract the play parameters
	# Order: 1:Next 2:Exit 3:Repeat 4:Speed Rate 5:Inertial Force
	$line = <$in>;
	chomp($line);
	$line =~ s/play_param=//; 
	my @play_param = split / /, $line;
	
	# process each step in the page (up to 7 per page)
	$line = <$in>;
	chomp($line);
	my @servo_values = ();
	my @pause_times = ();
	my @play_times = ();
	my $steps = 0;
	
	while( $line =~ /step=/ ) { 
		# first remove the 'step=' preceding the servo values
		$line =~ s/step=//;
		$steps += 1;
		$total_steps += 1;
		
		# now we transfer the servo values and play times into an array
		my @servo_val = split / /, $line;
		my $j = 0;
		my $act_srvo = 0;
		my $count3 = 1;
		my $packed_servo_value = 0;
		# ready to loop 
		foreach (@servo_val) {
			# now we filter the values for the active servos only
			if( $j <=25 ) {
				use integer;
				if ($servos[$j] == "1" ) { 
					$act_srvo += 1;
					# check if we need to update min/max values
					if($servo_val[$j] > $servo_max_val[$act_srvo-1]) { $servo_max_val[$act_srvo-1] = $servo_val[$j]; }
					if($servo_val[$j] < $servo_min_val[$act_srvo-1]) { $servo_min_val[$act_srvo-1] = $servo_val[$j]; }
					# we now need to compress 3 values into 1 to save space
					if($count3 == 1) {
						# first value to add - need to shift 22 bits left
						$packed_servo_value += $servo_val[$j] * 2048 * 2048;
						$count3 += 1;
					} elsif ($count3 == 2) {
						# second value to add - need to shift 11 bits left
						$packed_servo_value += $servo_val[$j] * 2048;
						$count3 += 1;
					} else {
						# final value to add and then push into array
						$packed_servo_value += $servo_val[$j];
						push( @servo_values, $packed_servo_value );
						# reset temporary variables
						$count3 = 1;
						$packed_servo_value = 0;
					}
				}
			}
			
			# last we need to extract pause and play time for each step
			if ($j == 26) {
				push( @pause_times, $servo_val[$j]*1000 );
			} elsif ($j == 27) {
				push( @play_times, $servo_val[$j]*1000 );
			}
			
			# increment the counter
			$j += 1;
		}
		
		# read the next line
		$line = <$in>;
		chomp($line);
	}

	# check that the last line is a page_end line
	# we have already read in the next line in the above while loop
	if( $line !~ /page_end/ ) {
		print STDERR "Motion Page $i - Page End not found!\n";
		close $in;
		exit;
	}
	
	if( $steps >= 1) {
		# increase the number of active pages to calculate total
		$total_pages += 1;
		push( @motion_pages, $i );
		# Output the motion page to the header file
		print $out "struct // $page_name \n";
		print $out "{ \n";
		print $out "   const uint8 JointFlexibility[$active_servos]; \n";
		print $out "   const uint8 NextPage; \n"; 
		print $out "   const uint8 ExitPage; \n";
		print $out "   const uint8 RepeatTime; \n";
		print $out "   const uint8 SpeedRate10; \n";
		print $out "   const uint8 InertialForce; \n";
		print $out "   const uint8 Steps; \n";
		print $out "   const uint32 StepValues[$steps][$packed_active_servos]; \n";
		print $out "   const uint16 PauseTime[$steps]; \n";
		print $out "   const uint16 PlayTime[$steps]; \n";
		# now we can add the initialisers to the struct
		print $out "} MotionPage$i PROGMEM = { \n";
		# first the joint flexibility values
		my $j = 0;
		print $out "{";
		foreach (@flex_values) {
			if( $j != @flex_values-1 ) { print $out "$_,"; } else { print $out "$_}, \n"; }
			$j += 1;
		}
		# then the play parameters in order and the number of steps
		$play_param[3] = $play_param[3] * 10;
		print $out "$play_param[0], $play_param[1], $play_param[2], $play_param[3], $play_param[4], $steps, \n";
		# next come the servo values
		$j = 0;
		my $jj = 1;
		print $out "{{";
		foreach (@servo_values) {
			if( $jj == $packed_active_servos && $j != @servo_values-1 ) {
				print $out "$_},\n {";
				$jj = 0;
			} elsif( $j != @servo_values-1 ) { 
				print $out "$_,"; 
			} else { 
				print $out "$_}}, \n"; 
			}
			$j += 1;
			$jj += 1;
		}
		# and finally pause and play times
		$j = 0;
		print $out "{";
		foreach (@pause_times) {
			if( $j != @pause_times-1 ) { print $out "$_,"; } else { print $out "$_}, "; }
			$j += 1;
		}
		$j = 0;
		print $out "{";
		foreach (@play_times) {
			if( $j != @play_times-1 ) { print $out "$_,"; } else { print $out "$_} \n"; }
			$j += 1;
		}
		print $out "}; \n\n";
	}
}
# finally we know the number of active motion pages
print $out "// Number of active motion pages in this file \n";
print $out "const uint8 ACTIVE_MOTION_PAGES = $total_pages; \n\n";
print $out "// Min and max values for the servo values \n";
print $out "const uint16 SERVO_MAX_VALUES[$active_servos] = {";
my $j = 0;
foreach(@servo_max_val) {			
	if( $j != $active_servos-1 ) { print $out "$_,"; } else { print $out "$_}; \n"; }
	$j += 1;
}
print $out "const uint16 SERVO_MIN_VALUES[$active_servos] = {";
$j = 0;
foreach(@servo_min_val) {			
	if( $j != $active_servos-1 ) { print $out "$_,"; } else { print $out "$_}; \n"; }
	$j += 1;
}
print $out "\n";
print $out "#endif /* MOTION_H_ */";

# Total number of steps in the motion file
print "Complete - $total_pages pages and $total_steps motion steps processed.\n"; 
# Finally calculate total memory use to check we stay below 64KBytes
my $total_memory = $total_steps * ($packed_active_servos*4 + 2 + 2);
$total_memory += $total_pages*($active_servos + 6);
print "Total memory use is $total_memory bytes. Check value is below 64KB!\n";

# Close the input and output files
close $in;
close $out;

# we also need to auto-generate the code for motionPageInit to create the pointers
open($out, ">", "motionPageInit.c") or die "Can't open output header file: $!";

print $out "	// Motion Page pointer assignment to PROGMEM \n";
print $out "	motion_pointer[0] = NULL; \n";
foreach (@motion_pages) {
	print $out "	motion_pointer[$_] = (uint8*) &MotionPage$_; \n";
}
print $out "\n";

close $out;
