##================================================================
##     Davicom Semiconductor Inc.  	
##   --------------------------------------------------------
## Description:
##              Compile driver dm9051.c to dm9051.o and dm9051.ko
##
## Modification List:
##================================================================

#ifneq ($(KERNELRELEASE),)

#(call from kernel build system)
obj-m	:=	dm9051.o

#else
# MODULE_INSTALDIR ?= /lib/modules/$(shell uname -r)/kernel/drivers/net/usb
#endif

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD		:=	$(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)  

clean:
	rm -rf *.ko *.o *~ core .depend .*.cmd *.mod.c .tmp_versions .*.o.d *.symvers *.order

install:
	sudo insmod dm9051.ko

remove:
	sudo ifconfig eth1 down
	sudo rmmod dm9051

reinstall:
	#(modprobe -r dm9051)
	# install -c -m 0644 dm9051.ko $(MODULE_INSTALDIR)
	# depmod -a -e
	sudo ifconfig eth1 down
	sudo rmmod dm9051
	sudo insmod dm9051.ko
