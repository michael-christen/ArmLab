include ../../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_GTK) -fPIC -O2
LDFLAGS =  $(LDFLAGS_VX_GTK) $(LDFLAGS_VX)  $(LDFLAGS_COMMON)  $(LDFLAGS_STD)  $(LDFLAGS_GTK)

LIB_GTK=libvxgtk.a
DESTLIB_GTK=../../../lib/$(LIB_GTK)

LIBVX_GTK_OBJS = vx_gtk_display_source.o vx_gtk_display.o gtkuimagepane.o vx_gtk_buffer_manager.o

VX_DEMO=../../../bin/vx_demo
VX_REMOTE_VIEWER=../../../bin/vx_remote_viewer
VX_SCENE_VIEWER=../../../bin/vx_scene_viewer

all: $(DESTLIB_GTK) $(VX_DEMO) $(VX_REMOTE_VIEWER) $(VX_SCENE_VIEWER)

$(DESTLIB_GTK) : $(LIB_GTK)
	cp $(LIB_GTK) $(DESTLIB_GTK)

$(LIB_GTK):  $(LIBVX_GTK_OBJS)
	@echo "    [$@]"
	ar rc $@ $^

$(VX_DEMO) : vx_demo.o $(LIB_GTK) ../../../lib/libvxgl.a ../../../lib/libvx.a
	@echo "    [$@]"
	gcc -o $@ $^ $(LDFLAGS) $(LDFLAGS_VX)

$(VX_SCENE_VIEWER) : vx_scene_viewer.o $(LIB_GTK) ../../../lib/libvxgl.a ../../../lib/libvx.a
	@echo "    [$@]"
	gcc -o $@ $^ $(LDFLAGS) $(LDFLAGS_VX)

$(VX_REMOTE_VIEWER) : vx_remote_viewer.o $(LIB_GTK)
	@echo "    [$@]"
	gcc -o $@ $^ $(LDFLAGS) $(LDFLAGS_VX)

clean:
	rm -rf  $(LIBVX_GTK_OBJS) $(VX_DEMO) $(VX_REMOTE_VIEWER) *.o $(DESTLIB_GTK) $(LIBVX_GTK)

test:
	@if [ -d test ]; then $(MAKE) -C test test ; else echo "  No tests"; fi

test-%:
	@if [ -d test ]; then $(MAKE) -C test $@ ; else echo "  No tests"; fi

.PHONY: test
