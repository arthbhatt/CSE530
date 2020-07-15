# To run "make", you can either set up environment variables via
#		source /opt/iot-devkit/1.7.2/environment-setup-i586-poky-linux
# or set up the following make variables
#


CC = i586-poky-linux-gcc
ARCH = x86
CROSS_COMPILE = i586-poky-linux-
SDKTARGETSYSROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux
export PATH:=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin:/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux:$(PATH)

APP = hcsr_tester

obj-m:= hcsr_drv.o

D_CFLAGS+= -g -DDEBUG -Wall
ccflags-y+= ${D_CFLAGS}
CC+= ${D_CFLAGS}

FLAGS=-Wall

all:
	make ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(SDKTARGETSYSROOT)/usr/src/kernel M=$(shell pwd) modules EXTRA_CFLAGS="$(D_CFLAGS)" mregparm=3 $(FLAGS)
	$(CC) -o $(APP) hcsr_tester.c --sysroot=$(SDKTARGETSYSROOT) -lpthread $(FLAGS)

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
