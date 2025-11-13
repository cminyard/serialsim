obj-m += serialsim.o

KERNEL ?= $(shell uname -r)

my_moddir = /lib/modules/$(KERNEL)

ccflags-y += -I$(src)/include

all:
	make -C $(my_moddir)/build M=$(PWD) modules

# Do our own clean, otherwise swig/serialsim.i gets removed
clean:
	rm -f  *.o*  *.ko*  *.mod*
	rm -f .*.o* .*.ko* .*.mod*
	rm -f *.symvers* .*.symvers*

private_key.der: openssl.conf
	openssl req -x509 -new -nodes -utf8 -sha256 -days 36500  \
		-batch -config openssl.conf -outform DER \
		-out public_key.der  -keyout private_key.der

KERNELBASE=/usr/src/linux-headers-$(KERNEL)

sign: all private_key.der
	$(KERNELBASE)/scripts/sign-file sha256 \
		private_key.der public_key.der serialsim.ko

prefix = /usr/local
includedir = $(prefix)/include

install:
	mkdir -p $(DESTDIR)/$(includedir)/linux
	cp include/linux/serialsim.h $(DESTDIR)/$(includedir)/linux
	mkdir -p $(my_moddir)/local
	cp serialsim.ko $(my_moddir)/local
	depmod -a $(KERNEL)
