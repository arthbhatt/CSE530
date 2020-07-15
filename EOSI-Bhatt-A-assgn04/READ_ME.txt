Author: Arth Bhatt (ASU ID: 1215361875)

Open a terminal and navigate to this (EOSI-Bhatt-A-assgn04) folder

KERNEL COMPILATION:
Applying the patch:
As mentioned in the submission details, apply the patch in the following way:
patch -p1 < <path/to/patch>  // In my case, name of patch file is "Arth.patch"

Exporting environment variables:
export PATH=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux:$PATH

Run the makefile to compile the kernel:
ARCH=x86 LOCALVERSION= CROSS_COMPILE=i586-poky-linux- make -j4

Now compile the modules:
ARCH=x86 LOCALVERSION= INSTALL_MOD_PATH=../galileo-install CROSS_COMPILE=i586-poky-linux- make modules_install

Now copy arch/x86/boot/bzImage to /media/realroot on SD card (either via ethernet or physically to the sd card)

USERPROGRAM:
How to make?
Run the Makefile command to generate the binaries for kernel module and the test program
To run the Makefile, type "make" in the terminal after navigating to this folder.

How to run?
1) Copy the binaries(namely test_insdump to Intel Galileo Gen2 Board.
   To copy, you can use the scp command. For example,
	scp test_insdump root@192.168.1.5:/home/root/

3) Run test_ to test the syscalls
	Check if it is executable or not. If not then run the following command:
		chmod +x test_insdump
	./test_insdump <syscall_select> <syscall_parameter>
	syscall_select : 1=>insdump 2=>rmdump
	syscall_parameter : if syscall_select is 1, then this will be the dumpmode for syscall
			    if syscall_select is 2, then this will be the dumpid

TESTING STRATEGY:
Disclaimer: I was the one using the test application, so there are no error checks for incorrect input.
		There are also some warnings in userspace program, as I thought it was going to be discarded.

I have mainly tested the code by putting kprobes on do_fork. I created new threads using fork() and pthread_create.
I also tried to remove dump_stack operation by the owner and also by someone else.
Changed the dumpmode and tested the kprobe triggering. 
Ran multiple syscall users simultaneously, to test the atomicity at shared sections of the code.
Reran the code a multiple times, to check for deadlock conditions.

Thank you 
