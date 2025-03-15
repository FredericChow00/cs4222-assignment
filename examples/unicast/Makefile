CONTIKI_PROJECT = receiver_unicast_communication transmitter_unicast_communication

TARGET=cc26x0-cc13x0
BOARD=sensortag/cc2650
CFLAGS += -w

all: $(CONTIKI_PROJECT)

CONTIKI = ../..


MAKE_NET = MAKE_NET_NULLNET
include $(CONTIKI)/Makefile.include
