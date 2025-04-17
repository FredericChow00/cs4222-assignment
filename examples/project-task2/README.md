## Instructions for executing your programs:
1. Create a new directory inside `contiki-ng/examples/` 
2. Copy `sender.c` and `receiver.c` into this directory. 
3. Create a Makefile with the following code:
```
CONTIKI_PROJECT = sender receiver
all: $(CONTIKI_PROJECT)

TARGET=cc26x0-cc13x0
BOARD=sensortag/cc2650
CFLAGS += -w

CONTIKI = ../..

MAKE_NET = MAKE_NET_NULLNET
include $(CONTIKI)/Makefile.include

```
4. Run `make` to compile both sender and receiver files 
5. Set up the CC2650F128 SensorTag using UniFlash and Realterm using the compiled executables of task2 and task3 to run the programs respectively

## Names & student IDs of group members: Dilys (A0238114M), Bryan (A0252218L), Ryan (A0255300W), Frederic (A0234411W)
