include ../common.mk

#CC = gcc -c
#CFLAGS = $(CFLAGS_COMMON) -Wall -g -std=gnu99 -fno-omit-frame-pointer -fno-stack-protector -D_REENTRANT -fPIC -shared -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -Wno-unused-parameter -Wno-unused-variable -Wno-format-zero-length -O0
CFLAGS = $(CFLAGS_COMMON) $(CFLAGS_STD)
LDFLAGS = $(LDFLAGS_COMMON) $(LDFLAGS_STD)

IMAGESOURCE_OBJS = image_source.o image_source_v4l2.o image_source_dc1394.o url_parser.o image_source_islog.o image_source_pgusb.o image_source_filedir.o image_convert.o

LIBIMAGESOURCE = libimagesource.a
DESTLIB=../../lib/$(LIBIMAGESOURCE)

ALL = $(DESTLIB) istest tcpstream

#############################################################
# MacOS X
ifeq "$(shell uname)" "Darwin"

.phony:
	@echo "JCam is not supported on Darwin/MacOS."

ALL = .phony

endif

#############################################################


all: $(ALL)

istest: main.o $(IMAGESOURCE_OBJS)
	@echo "\t$@"
	@gcc -L../../lib main.o $(IMAGESOURCE_OBJS) -o istest -ldc1394 -lc -lcommon -lpng -lpthread -lusb-1.0 -lm

tcpstream: tcp_image_streamer.c
	@gcc -o tcpstream -std=gnu99 tcp_image_streamer.c -lc

$(DESTLIB): $(LIBIMAGESOURCE)
	@cp $(LIBIMAGESOURCE) $(DESTLIB)

$(LIBIMAGESOURCE): $(IMAGESOURCE_OBJS)
	@ar rc $@ $(IMAGESOURCE_OBJS)

# XXX This will be a problem
#libimagesource.so: april_jcam_ImageSourceNative.h $(LIBIMAGESOURCE_OBJS)
#	$(LD) $(LDFLAGS) $(LIBIMAGESOURCE_OBJS) -o libimagesource.so -ldc1394 -lc -lcommon -lpng

#april_jcam_ImageSourceNative.h:
#	echo "Rebuilding JNI headers. Ensure java file has been recently built."
#	javah -classpath ../../april.jar  april.jcam.ImageSourceNative

clean:
	@rm -f *~ $(LIBIMAGESOURCE) $(DESTLIB) $(IMAGESOURCE_OBJS) istest main.o tcpstream tcp_image_streamer.o

# If you get an error message saying that you're missing jni.h or
# jni_md.h, locate those files and add the directories using the -I
# flag. See example below

DC1394_INCLUDES = -I/usr/local/include/

#%.o: %.c
#	$(CC) $(CFLAGS) $(LDFLAGS) $(DC1394_INCLUDES) $<
