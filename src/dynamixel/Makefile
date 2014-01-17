include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_GLIB) $(CFLAGS_GTK) -msse2 -fPIC -O4
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_GLIB) $(LDFLAGS_GTK)

DYNAMIXEL_OBJS = serial.o ioutils.o dynamixel_bus.o dynamixel_serial_bus.o \
				 dynamixel_device.o axseries.o mxseries.o


LIBDYNAMIXEL=$(LIB_PATH)/libdynamixel.a

all: $(LIBDYNAMIXEL)


$(LIBDYNAMIXEL): $(DYNAMIXEL_OBJS)
	@echo "\t$@"
	@ar rc $@ $(DYNAMIXEL_OBJS)

clean:
	@rm -f *.o *~ *.a
