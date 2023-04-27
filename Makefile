obj-m += grl_pwr_ctrl.o

KERNEL	  := /usr/src/linux-headers-4.9.140-tegra-ubuntu18.04_aarch64/kernel-4.9
PWD       := $(shell pwd)

all:
		$(MAKE) ARCH=arm64 -C $(KERNEL) M=$(PWD) modules

clean:
		make -C $(KERNEL) M=$(PWD) clean