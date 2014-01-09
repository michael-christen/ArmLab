############################################################
#
# This file should only contain CFLAGS_XXX and LDFLAGS_XXX directives.
# CFLAGS and LDFLAGS themselves should NOT be set: that is the job
# for the actual Makefiles (which will combine the flags given here)
#
# *** DO NOT SET CFLAGS or LDFLAGS in this file ***
#
# Our recommended flags for all projects. Note -pthread specifies reentrancy

# -Wno-format-zero-length: permit printf("");
# -Wno-unused-parameter: permit a function to ignore an argument
CFLAGS_STD   := -std=gnu99 -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_REENTRANT \
		-Wall -Wno-unused-parameter -Wno-format-zero-length -pthread
LDFLAGS_STD  := -lm

ROOT_PATH   := $(subst /src/common.mk,,$(realpath $(lastword $(MAKEFILE_LIST))))
SRC_PATH     := $(ROOT_PATH)/src
BIN_PATH     := $(ROOT_PATH)/bin
LIB_PATH     := $(ROOT_PATH)/lib
CONFIG_DIR   := $(shell pwd)/../../config

CC           := gcc
LD           := gcc

#.SILENT:

# dynamic libraries
ifeq "$(shell uname -s)" "Darwin"
	LDSH := -dynamic
	SHEXT := .dylib
	WHOLE_ARCHIVE_START := -all_load
else
	LD := gcc
	LDSH := -shared
	SHEXT := .so
	WHOLE_ARCHIVE_START := -Wl,-whole-archive
	WHOLE_ARCHIVE_STOP := -Wl,-no-whole-archive
endif

############################################################
#
# External libraries
#
# List these in roughly the order of dependency; those with fewest
# dependencies first. Within each LDFLAGS, list the dependencies in in
# decreasing order (e.g., end with LDFLAGS_GLIB)
#
############################################################

# common library
CFLAGS_COMMON  := -I$(SRC_PATH) -DCONFIG_DIR='"$(CONFIG_DIR)"'
LDFLAGS_COMMON := $(LIB_PATH)/libcommon.a

# glib
CFLAGS_GLIB  := `pkg-config --cflags glib-2.0 gmodule-2.0`
LDFLAGS_GLIB := `pkg-config --libs glib-2.0 gmodule-2.0 gthread-2.0 gobject-2.0`

# jpeg
ifeq "$(shell test -f /usr/lib/libjpeg-ipp.so -o -f /usr/lib64/libjpeg-ipp.so && echo ipp)" "ipp"
	LDFLAGS_JPEG := -ljpeg-ipp
else
	LDFLAGS_JPEG := -ljpeg
endif

# gtk
CFLAGS_GTK   :=`pkg-config --cflags gtk+-2.0`
LDFLAGS_GTK  :=`pkg-config --libs gtk+-2.0 gthread-2.0`

# lcm
CFLAGS_LCM  := `pkg-config --cflags lcm`
LDFLAGS_LCM := `pkg-config --libs lcm`

# lcmtypes
CFLAGS_LCMTYPES  := -I$(SRC_PATH)
LDFLAGS_LCMTYPES := $(LIB_PATH)/liblcmtypes.a

# vx (no GUI)
CFLAGS_VX  := -I$(SRC_PATH)
LDFLAGS_VX := $(LIB_PATH)/libvx.a

# vx gl
CFLAGS_VX_GL  := -I$(SRC_PATH)
LDFLAGS_VX_GL := $(LIB_PATH)/libvxgl.a  -lGL -lX11

# vx GUI only
CFLAGS_VX_GTK  := -I$(SRC_PATH)
LDFLAGS_VX_GTK := $(LIB_PATH)/libvxgtk.a $(LDFLAGS_VX_GL) $(LDFLAGS_VX) -lz

# Open GL
CFLAGS_GL    :=
LDFLAGS_GL   := -lGLU -lGLU -lglut

# our gl util library
LDFLAGS_GLUTIL := $(LIB_PATH)/libglutil.a

# libusb-1.0
CFLAGS_USB := `pkg-config --cflags libusb-1.0`
LDFLAGS_USB := `pkg-config --libs libusb-1.0`

# libpng
CFLAGS_PNG := `pkg-config --cflags libpng`
LDFLAGS_PNG := `pkg-config --libs libpng`

# dc1394
CFLAGS_DC1394 := `pkg-config --cflags libdc1394-2`
LDFLAGS_DC1394 := `pkg-config --libs libdc1394-2`

# Intel Integrated Performance Primitives
IPPA:=
IPP_LIBS:=-lguide -lippcore -lippi -lippcc -lippcv
ifeq "$(shell uname -s)" "Darwin"
    IPP_BASE:=/Library/Frameworks/Intel_IPP.framework
    CFLAGS_IPP:=-I$(IPP_BASE)/Headers
    LDFLAGS_IPP:=-L$(IPP_BASE)/Libraries $(IPP_LIBS)
else
    ifeq "$(shell uname -m)" "x86_64"
        IPP_SEARCH := /usr/local/intel/ipp/5.1/em64t \
                      /opt/intel/ipp/5.2/em64t       \
		      /opt/intel/ipp/5.3/em64t       \
		      /opt/intel/ipp/5.3.1.062/em64t \
	              /opt/intel/ipp/5.3.2.068/em64t
        test_dir = $(shell [ -e $(dir) ] && echo $(dir))
        IPP_SEARCH := $(foreach dir, $(IPP_SEARCH), $(test_dir))
        IPP_BASE := $(firstword $(IPP_SEARCH))
        IPP_LIBS:=-lguide -lippcoreem64t -lippiem64t -lippjem64t -lippccem64t -lippcvem64t -lpthread -lippsem64t
        IPPA:=em64t
    else
        IPP_BASE:=/usr/local/intel/ipp/5.1/ia32
    endif
    CFLAGS_IPP:=-I$(IPP_BASE)/include
    LDFLAGS_IPP:=-L$(IPP_BASE)/sharedlib -Wl,-R$(IPP_BASE)/sharedlib $(IPP_LIBS)
endif

# imagesource
CFLAGS_IMAGESOURCE = $(CFLAGS_COMMON) $(CFLAGS_STD)
LDFLAGS_IMAGESOURCE = $(LDFLAGS_COMMON) $(LDFLAGS_STD)

CFLAGS_IMAGESOURCE := $(CFLAGS_GTK) $(CFLAGS_USB) $(CFLAGS_PNG) $(CFLAGS_DC1394)
LDFLAGS_IMAGESOURCE := $(LIB_PATH)/libimagesource.a $(LDFLAGS_GTK) $(LDFLAGS_USB) $(LDFLAGS_PNG) $(LDFLAGS_DC1394)

%.o: %.c %.h
	@echo "\t$@"
	@$(CC) $(CFLAGS) -c $<

%.o: %.c
	@echo "\t$@"
	@$(CC) $(CFLAGS) -c $<

%.o: %.cpp
	@echo "\t$@"
	@g++ -c -o $@ $< $(CFLAGS_CXX)


MAKEFLAGS += --no-print-directory
