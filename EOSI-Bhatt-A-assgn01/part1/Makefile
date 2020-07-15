# To run "make", you can either set up environment variables via
#		source /opt/iot-devkit/1.7.2/environment-setup-i586-poky-linux
# or set up the following make variables
#


CC = i586-poky-linux-gcc
ARCH = x86
CROSS_COMPILE = i586-poky-linux-
SDKTARGETSYSROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux
export PATH:=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin:/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux:$(PATH)

APP = rbt530_tester

obj-m:= rbt530_drv.o

D_CFLAGS+= -g -DDEBUG -Wall
ccflags-y+= ${D_CFLAGS}
CC+= ${D_CFLAGS}

FLAGS=-Wall

all:
	make ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(SDKTARGETSYSROOT)/usr/src/kernel M=$(shell pwd) modules EXTRA_CFLAGS="$(D_CFLAGS)" mregparm=3 $(FLAGS)
	$(CC) -o $(APP) rbt530_tester_bulk.c --sysroot=$(SDKTARGETSYSROOT) -lpthread $(FLAGS)

clean:
	rm -f *.ko
	rm -f *.o
	rm -f Module.symvers
	rm -f modules.order
	rm -f *.mod.c
	rm -rf .tmp_versions
	rm -f *.mod.c
	rm -f *.mod.o
	rm -f \.*.cmd
	rm -f Module.markers
	rm -f $(APP)
