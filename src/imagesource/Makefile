include ../common.mk

CFLAGS = $(CFLAGS_COMMON) $(CFLAGS_STD) $(CFLAGS_GTK) $(CFLAGS_USB) $(CFLAGS_PNG) $(CFLAGS_DC1394)
LDFLAGS = $(LDFLAGS_COMMON) $(LDFLAGS_STD) $(LDFLAGS_GTK) $(LDFLAGS_USB) $(LDFLAGS_PNG) $(LDFLAGS_DC1394)

IMAGESOURCE_OBJS = image_source.o image_source_v4l2.o image_source_dc1394.o url_parser.o image_source_islog.o image_source_pgusb.o image_source_filedir.o image_source_tcp.o image_convert.o

LIBIMAGESOURCE = libimagesource.a
DESTLIB=../../lib/$(LIBIMAGESOURCE)

ALL = $(DESTLIB) istest tcpstream isview

#############################################################
# MacOS X
ifeq "$(shell uname)" "Darwin"

.phony:
	@echo "JCam is not supported on Darwin/MacOS."

ALL = .phony

endif

#############################################################


all: $(ALL)

isview: isview.o $(IMAGESOURCE_OBJS)
	@echo "\t$@"
	@$(LD) -o isview isview.o $(IMAGESOURCE_OBJS) $(LDFLAGS)

istest: istest.o $(IMAGESOURCE_OBJS)
	@echo "\t$@"
	@$(LD) -o istest istest.o $(IMAGESOURCE_OBJS) $(LDFLAGS)

tcpstream: tcp_image_streamer.c
	@gcc -o tcpstream -std=gnu99 tcp_image_streamer.c -lc

$(DESTLIB): $(LIBIMAGESOURCE)
	@cp $(LIBIMAGESOURCE) $(DESTLIB)

$(LIBIMAGESOURCE): $(IMAGESOURCE_OBJS)
	@ar rc $@ $(IMAGESOURCE_OBJS)

clean:
	@rm -f *~ *.o $(LIBIMAGESOURCE) $(DESTLIB) $(IMAGESOURCE_OBJS) istest tcpstream

