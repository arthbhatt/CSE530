KDIR:=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux/usr/src/kernel
CC = i586-poky-linux-gcc
CROSS_COMPILE = i586-poky-linux-

PWD:= $(shell pwd)

ARCH = x86
SROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux
EXTRA_CFLAGS += -Wall

export PATH:=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin:/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux:$(PATH)

LDLIBS = -L$(SROOT)/usr/lib
CCFLAGS = -I$(SROOT)/usr/include/libnl3

APP = main

obj-m:= kmodule.o led_matrix_device.o

.PHONY:all
all: main kmodule.ko

kmodule.ko:
	make ARCH=x86 CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules

main:
	$(CC) -Wall -o $(APP) main.c $(CCFLAGS) -lnl-genl-3 -lnl-3 -lpthread --sysroot=$(SROOT)


.PHONY:clean
clean:
	make -C $(KDIR) M=$(PWD) clean
	rm -f *.o $(EXAMPLE) $(APP)

deploy:
	tar czf programs.tar.gz $(APP) $(EXAMPLE) $(SPI) genl_drv.ko
	scp programs.tar.gz root@10.0.1.100:/home/root
	ssh root@10.0.1.100 'tar xzf programs.tar.gz'
