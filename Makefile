MODULE_NAME := card-reader
#KDIR := /home/vitor/Embedded/buildroot-2020.02.8/output/build/linux-stable/
KDIR := /home/vitor/Embedded/buildroot-2020.02.8/output/build/linux-v5.0.2/
ARCH := arm
CROSS_COMPILE := arm-buildroot-linux-gnueabihf-
PWD := $(shell pwd)
MODULE_UPLOAD := $(MODULE_NAME).ko
USERNAME := root

# Put device driver sources here
$(MODULE_NAME)-objs += sources/card-reader.o
$(MODULE_NAME)-objs += sources/aba-sysfs.o
$(MODULE_NAME)-objs += sources/aba-reader.o
$(MODULE_NAME)-objs += sources/wiegand-sysfs.o
$(MODULE_NAME)-objs += sources/wiegand-reader.o
$(MODULE_NAME)-objs += sources/em125-sysfs.o
$(MODULE_NAME)-objs += sources/em125-reader.o

# Device driver name
obj-m := $(MODULE_NAME).o

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules
	
clean:
	rm -rf *.o *.ko .tmp_versions *.mod.c modules.order  Module.symvers sources/modules.order sources/*.mod.c sources/*.o sources/*.ko
