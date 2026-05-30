KVERSION ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)
LLVM ?= 1
ifeq ($(origin CC),default)
CC := clang
endif

obj-m := smitdtmb.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) LLVM=$(LLVM) CC=$(CC) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) LLVM=$(LLVM) CC=$(CC) clean

.PHONY: all clean
