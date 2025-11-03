Q		:= @
DRIVER	:= f81601.ko

all:
	$(Q)cd driver && make

clean:
	$(Q)rm -f ./release/$(DRIVER)
	$(Q)cd driver && make clean
	rm -f /lib/modules/$(shell uname -r)/updates/f81601.ko
	depmod

install:
	$(Q)cd driver && make install
	$(Q)cp ./driver/$(DRIVER) ./release