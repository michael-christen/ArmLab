include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_GTK) -fPIC -O2
LDFLAGS =  $(LDFLAGS_VX_GTK) $(LDFLAGS_VX)  $(LDFLAGS_COMMON)  $(LDFLAGS_STD)  $(LDFLAGS_GTK) $(LDFLAGS_IMAGESOURCE)

VX_BASE = demo
LIB = ../../lib

APP_LIBS = $(LIB)/libvxgtk.a $(LIB)/libvxgl.a $(LIB)/libvx.a \
		   $(LIB)/libimagesource.a

all: $(VX_BASE)

$(VX_BASE) : vx_base.o
	@echo "    [$@]"
	gcc -o $@ $^ $(LDFLAGS)

clean:
	rm -rf  $(VX_BASE) *.o *~
