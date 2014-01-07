include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_GLIB) $(CFLAGS_GTK) -msse2 -fPIC -O4
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_GLIB) $(LDFLAGS_GTK)

COMMON_OBJS = getopt.o gps_linearize.o ioutils.o math_util.o moxa.o serial_wrapper.o timespec.o timestamp.o serial.o ssocket.o string_util.o timesync.o \
c5.o matd.o zarray.o zhash.o task_thread.o image_u8.o image_u32.o image_util.o pg.o param_widget.o


LIBCOMMON=$(LIB_PATH)/libcommon.a

all: $(LIBCOMMON)


$(LIBCOMMON): $(COMMON_OBJS)
	@echo "\t$@"
	@ar rc $@ $(COMMON_OBJS)

pixels.o: pixels.c
	@echo "\t$@"
	@$(CC) $(CFLAGS) -msse3 -c $<

gridmap.o: gridmap.c
	@echo "\t$@";
	@$(CC) $(CFLAGS) -msse -c $<

clean:
	@rm -f *.o *~ *.a
	@rm -f gu_minheap_tester
