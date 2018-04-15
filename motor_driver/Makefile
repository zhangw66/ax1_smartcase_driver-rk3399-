KRN_SRC_PWD=/home/zhangw/workdir/rockchip/rk3399/firefly/kernel

obj-m += neolix_timed_gpio.o

build:kernel_modules
kernel_modules:
	make -C $(KRN_SRC_PWD) M=$(CURDIR) modules
clean:
	make -C $(KRN_SRC_PWD) M=$(CURDIR) clean

