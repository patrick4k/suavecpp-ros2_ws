#!/usr/bin/perl
use strict;
use warnings;

if (defined($ENV{SUAVE_MAVLINK_SERIAL_ID}))
{
    die 'SUAVE_MAVLINK_SERIAL_ID is already set';
}

my @files;
{
    my $serial_path = '/dev/serial/by-id/';
    opendir my $dir, $serial_path or die "Cannot find '$serial_path'";
    @files = readdir $dir;
    closedir $dir;
}

for (@files)
{
    if (/(usb-FTDI_\w+_USB_UART_\w+-\w+-port0)/)
    {
        print "found '$1'! \n";
        `echo 'export SUAVE_MAVLINK_SERIAL_ID="$1"' >> ~/.bashrc`;
        die;
    }
}

print 'ERROR: Could not set SUAVE_MAVLINK_SERIAL_ID...';
