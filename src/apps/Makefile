include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_GTK) -fPIC -O2
LDFLAGS =  $(LDFLAGS_VX_GTK) $(LDFLAGS_VX) $(LDFLAGS_GTK) $(LDFLAGS_IMAGESOURCE) $(LDFLAGS_COMMON) $(LDFLAGS_STD)

VX_BASE = demo
LIB = ../../lib

all: $(VX_BASE)

$(VX_BASE) : eecs467_util.o eecs467_example.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf  $(VX_BASE) *.o *~
