KDIR:=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux/usr/src/kernel
CC = i586-poky-linux-gcc
CROSS_COMPILE = i586-poky-linux-

PWD:= $(shell pwd)

ARCH = x86
SROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux
EXTRA_CFLAGS += -Wall

export PATH:=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin:/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux:$(PATH)

LDLIBS = -L$(SROOT)/usr/lib


APP = test_insdump

.PHONY:all
all: main

main:
	$(CC) -Wall -o $(APP) test_insdump.c $(CCFLAGS) -lpthread --sysroot=$(SROOT)


.PHONY:clean
clean:
	rm -f *.o $(APP)

