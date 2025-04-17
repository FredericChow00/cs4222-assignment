CONTIKI_PROJECT = sender receiver
all: $(CONTIKI_PROJECT)

TARGET=cc26x0-cc13x0
BOARD=sensortag/cc2650
CFLAGS += -w

CONTIKI = ../..

MAKE_NET = MAKE_NET_NULLNET
include $(CONTIKI)/Makefile.include
