# To run "make", you can either set up environment variables via
#		source /opt/iot-devkit/1.7.2/environment-setup-i586-poky-linux
# or set up the following make variables
#
## Uncomment for cross compilation
SROOT = i586-poky-linux-gcc
#ARCH = x86
#CROSS_COMPILE = i586-poky-linux-
SDKTARGETSYSROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux
export PATH:=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin:/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux:$(PATH)

APP = genl_ex

MODULE_NAME = genl_drv.ko
GALILEO_PATH = /home/root/
GALILEO_IP = 192.168.0.5

LDLIBS = -L$(SDKTARGETSYSROOT)/usr/lib
CCFLAGS = -I$(SDKTARGETSYSROOT)/usr/include/libnl3

obj-m:= genl_drv.o

## Uncomment for cross compilation
galileo:
	make ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(SDKTARGETSYSROOT)/usr/src/kernel M=$(PWD) modules
	$(SROOT) -Wall -o $(APP) genl_ex.c -lpthread --sysroot=$(SDKTARGETSYSROOT) $(CCFLAGS) $(LDLIBS) -lnl-genl-3 -lnl-3

host:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	gcc -Wall -o $(APP) genl_ex.c $(CCFLAGS) $(LDLIBS) -lnl-genl-3 -lnl-3
scp:
	scp $(MODULE_NAME) root@$(GALILEO_IP):$(GALILEO_PATH)
	scp $(APP) root@$(GALILEO_IP):$(GALILEO_PATH)
	ssh root@$(GALILEO_IP)

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
