# Comment/uncomment the following line to disable/enable debugging
DEBUG = y

# Add your debugging flag (or not) to EXTRA_CLAGS
#Note:
#	for kernel upto 2.6.23
#       Makefile need to change for kernel 2.6.24 used EXTRA_CLAGS instead of CFLAGS 
ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

EXTRA_CLAGS += $(DEBFLAGS) -I$(LDDINCDIR)

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
# call from kernel build system

	AGEXDrvAMD64-objs := LockedOps.o AGEXDrv.o

	obj-m	:= AGEXDrvAMD64.o

# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD       := $(shell pwd)

default:
#für ein local include dir
#LDDINCDIR=$(PWD)/../include
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

endif



clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers

depend .depend dep:
	$(CC) $(EXTRA_CLAGS) -M *.c > .depend


ifeq (.depend,$(wildcard .depend))
include .depend
endif
