# Comment/uncomment the following line to disable/enable debugging
# or call make DEBUG=y default is DEBUG=n
# and use 'tail -f /var/log/kern.log' for debug massages
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

#DEBUG=y
ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g -DDEBUG -Wno-sign-compare  # "-O" is needed to expand inlines
  $(info we use debug flags [${DEBFLAGS}])
else
  DEBFLAGS = -O2 -Wno-sign-compare 
endif

ccflags-y := $(DEBFLAGS) -Werror -Wall -Wno-unused-parameter -Wno-date-time 
agexpcidrv-objs := FileOps.o ISRTasklet.o LockedOps.o AGEXDrv.o DMARead.o
ifeq ($(ARCH),arm64)
	agexpcidrv-objs += SPI.o
endif
ifeq ($(CONFIG_PCI),y)
	agexpcidrv-objs += PCI.o
endif
obj-m	:= agexpcidrv.o

# If KERNELDIR is defined, we've been invoked from the DKMS
# otherwise we were called directly from the command line
# '?=' has only an effect if the variable not defined
KERNELDIR ?= /lib/modules/$(shell uname -r)/build


default:
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) modules 

# create a file like 'agexpcidrv_4.9.0-6-amd64_x86_64.ko'
deploy:
	make clean
	make
	strip --strip-debug agexpcidrv.ko
	mv agexpcidrv.ko agexpcidrv_$(shell uname -r)_$(shell uname -m).ko

devel:
	clear
	make -j`nproc` DEBUG=y
	make install
	rmmod agexpcidrv
	modprobe agexpcidrv

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers

#default /lib/modules/$(KERNELRELEASE)/extra
install:
	make -C $(KERNELDIR) M=$(CURDIR) modules_install
	depmod -a 

