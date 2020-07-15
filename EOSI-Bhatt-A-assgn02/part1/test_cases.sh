#!/bin/bash

# Author: Arth Bhatt (ASU ID: 1215361875)
# This file tests the sysfs created for hcsr device

# Parameters for one device
DEV=HCSR_0
trigger_pin=0
echo_pin=1
m=5
delta=60

# Parameters for second device
DEV2=HCSR_1
trigger_pin2=2
echo_pin2=3
m2=5
delta2=60

config_pins=1
set_parameters=4

clear_buffer=1
maintain_buffer=0

insmod ./hcsr_drv.ko

echo " "
echo "#######################"
echo "Testcase 0 - Simple Run"
echo "#######################"
echo " "
./hcsr_tester "/dev/$DEV" "ioctl" $config_pins $trigger_pin $echo_pin
./hcsr_tester "/dev/$DEV" "ioctl" $set_parameters $m $delta
./hcsr_tester "/dev/$DEV" "write" $maintain_buffer
./hcsr_tester "/dev/$DEV" "read" 

sleep 3

echo " "
echo "#############################################################################################"
echo "Testcase 1 - Running two writes consecutively (Notice that only one write operation succeeds)"
echo "#############################################################################################"
echo " "
#./hcsr_tester "/dev/$DEV" "ioctl" $config_pins $trigger_pin $echo_pin
#./hcsr_tester "/dev/$DEV" "ioctl" $set_parameters $m $delta
./hcsr_tester "/dev/$DEV" "write" $maintain_buffer
./hcsr_tester "/dev/$DEV" "write" $maintain_buffer

sleep 3

echo " "
echo "#########################################################################################################"
echo "Testcase 2 - Running multiple read(s), which will empty the buffer and start triggering new measurements."
echo "#########################################################################################################"
echo " "
#./hcsr_tester "/dev/$DEV" "ioctl" $config_pins $trigger_pin $echo_pin
#./hcsr_tester "/dev/$DEV" "ioctl" $set_parameters $m $delta
./hcsr_tester "/dev/$DEV" "read" $maintain_buffer
./hcsr_tester "/dev/$DEV" "read" $maintain_buffer
./hcsr_tester "/dev/$DEV" "read" $maintain_buffer
./hcsr_tester "/dev/$DEV" "read" $maintain_buffer
./hcsr_tester "/dev/$DEV" "read" $maintain_buffer

sleep 3

echo " "
echo "#############################"
echo "Testcase 4 - Multiple Devices"
echo "#############################"
echo " "
./hcsr_tester "/dev/$DEV2" "ioctl" $config_pins $trigger_pin2 $echo_pin2
./hcsr_tester "/dev/$DEV2" "ioctl" $set_parameters $m2 $delta2
./hcsr_tester "/dev/$DEV" "write" $maintain_buffer
./hcsr_tester "/dev/$DEV2" "write" $maintain_buffer
./hcsr_tester "/dev/$DEV" "read"
./hcsr_tester "/dev/$DEV2" "read" 

rmmod ./hcsr_drv.ko