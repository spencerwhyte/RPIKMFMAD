obj-m +=khaki.o

all:
	make -C /usr/src/linux M=$(CURDIR) modules
clean:
	make -C /usr/src/linux M=$(CURDIR) clean
