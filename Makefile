LDLIBS=-lrt -lpthread
#CFLAGS=-g -Wall

ifneq ($(KERNELRELEASE),)
obj-m	:= tachol.o

else
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)

default:
	$(MAKE)	-C $(KDIR)	M=$(PWD) modules
endif

clean:
	rm -rf *.ko *.o *.mod.c 
	rm -rf modules.order Module.symvers
	rm -rf .tmp_versions .*.cmd *.dwo .*.dwo
	rm -f motor

