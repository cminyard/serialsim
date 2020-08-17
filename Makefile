obj-m += serialsim.o

my_moddir = /lib/modules/$(shell uname -r)

ccflags-y += -I$(src)/include

all:
	make -C $(my_moddir)/build M=$(PWD) modules

# Do our own clean, otherwise swig/serialsim.i gets removed
clean:
	rm -f *.o* *.ko *.mod*

prefix = /usr/local
includedir = $(prefix)/include

install:
	mkdir -p $(DESTDIR)/$(includedir)/linux
	cp include/linux/serialsim.h $(DESTDIR)/$(includedir)/linux
	mkdir -p $(my_moddir)/local
	cp serialsim.ko $(my_moddir)/local
	depmod -a
