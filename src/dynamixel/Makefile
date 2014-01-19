include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_GLIB) $(CFLAGS_GTK) -msse2 -fPIC -O4
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_GLIB) $(LDFLAGS_GTK) -llcm

DYNAMIXEL_OBJS = serial.o ioutils.o dynamixel_bus.o dynamixel_serial_bus.o \
				 dynamixel_device.o axseries.o mxseries.o

LCMTYPES_C := $(shell cd ../lcmtypes && ls *.c)
LCMTYPES_C := $(addprefix ../lcmtypes/,$(LCMTYPES_C))

LIBDYNAMIXEL=$(LIB_PATH)/libdynamixel.a

all: $(LIBDYNAMIXEL) test driver


$(LIBDYNAMIXEL): $(DYNAMIXEL_OBJS)
	@echo "\t$@"
	@ar rc $@ $(DYNAMIXEL_OBJS)

test: $(LIBDYNAMIXEL) dynamixel_test.o
	@gcc -o dynamixel_test dynamixel_test.o $(DYNAMIXEL_OBJS) $(LDFLAGS) $(CFLAGS)

driver: $(LIBDYNAMIXEL) arm_driver.c
	@gcc -o arm_driver arm_driver.c $(LCMTYPES_C) $(DYNAMIXEL_OBJS) $(LDFLAGS) $(CFLAGS)
	@cp arm_driver ../../bin/

clean:
	@rm -f *.o *~ *.a
