Author: Arth Bhatt (ASU ID: 1215361875)

Open a terminal and navigate to this (EOSI-Bhatt-A-assgn03) folder

How to make?
Run the Makefile command to generate the binaries for kernel module and the test program
To run the Makefile, type "make" in the terminal after navigating to this folder.

How to run?
1) Copy the binaries(namely kmodule.ko, led_matrix_device.ko, main to Intel Galileo Gen2 Board.
   To copy, you can use the scp command. For example,
	scp main kmodule.ko led_matrix_device.ko root@192.168.1.5:/home/root/

2) Insert the kernel module using insmod
	insmod kmodule.ko
	insmod led_matrix_device.ko

3) Run main to test the kernel module
	Check if it is executable or not. If not then run the following command:
		chmod +x main

Thank you 
