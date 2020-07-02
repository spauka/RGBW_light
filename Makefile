BINARY = main
OBJS += usb.o light.o colorsys.o

LDSCRIPT = stm32f103x4.ld

include Makefile.include
