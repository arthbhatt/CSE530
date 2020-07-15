#!/bin/bash

# Author: Arth Bhatt (ASU ID: 1215361875)
# This file tests the sysfs created for hcsr device

DEV=HCSR_0
trigger_pin=0
echo_pin=1
m=5
delta=60

echo "Configuring Parameters"
echo $trigger_pin > "/sys/class/misc/$DEV/trigger"
echo $echo_pin > "/sys/class/misc/$DEV/echo"
echo $m > "/sys/class/misc/$DEV/number_samples"
echo $delta > "/sys/class/misc/$DEV/sampling_period"

#---------------------------------------------------

echo "Showing the configured parameters"

echo -n "Device: $DEV"

echo -n "Trigger Pin: "
cat "/sys/class/misc/$DEV/trigger"

echo -n "Echo Pin: "
cat "/sys/class/misc/$DEV/echo"

echo -n "Number of Samples: "
cat "/sys/class/misc/$DEV/number_samples"

echo -n "Sampling Period: "
cat "/sys/class/misc/$DEV/sampling_period"

#-----------------------------------------------------

echo "Setting enable to measure distance"
echo 1 > "/sys/class/misc/$DEV/enable"

echo "Waiting for measurement"
sleep 2
cat "/sys/class/misc/$DEV/distance"

#----------------------------------------------------

echo "Now we try to cancel the measurement"
echo 1 > "/sys/class/misc/$DEV/enable"
echo 0 > "/sys/class/misc/$DEV/enable"

echo "Waiting for measurement"
sleep 2
cat "/sys/class/misc/$DEV/distance"
