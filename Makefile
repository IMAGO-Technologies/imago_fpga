# Comment/uncomment the following line to disable/enable debugging
# or call make DEBUG=y default is DEBUG=n
#DEBUG = y

# Add your debugging flag (or not) to EXTRA_CLAGS
#Note:
# for kernel upto 2.6.23 uses
# 		CFLAGS
#
# later it used EXTRA_CLAGS instead of CFLAGS 
#		e.g: EXTRA_CLAGS += $(DEBFLAGS) 
#
# since ~2007/2009 it uses ccflags-y
#		e.g: ccflags-y += -v
#

ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g -DDEBUG # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2  
endif

ccflags-y := $(DEBFLAGS) -Werror -Wall -Wextra -Wno-unused-parameter -Wno-date-time 

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
# call from kernel build system

	agexpcidrv-objs := FileOps.o ISRTasklet.o LockedOps.o AGEXDrv.o PCI.o DMARead.o

	obj-m	:= agexpcidrv.o

# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD       := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

endif


deploy:
	make clean
	make
	strip --strip-debug agexpcidrv.ko
	mv agexpcidrv.ko agexpcidrv_$(shell uname -r)_$(shell uname -m).ko

devel:
	make 
	make install
	rmmod agexpcidrv
	modprobe agexpcidrv

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers

#default /lib/modules/$(KERNELRELEASE)/extra
install:
	make -C $(KERNELDIR) M=$(PWD) modules_install
	depmod -a 

