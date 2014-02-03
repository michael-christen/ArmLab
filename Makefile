include ../common.mk

CFLAGS = $(CFLAGS_COMMON) $(CFLAGS_GLIB) $(CFLAGS_VX) $(CFLAGS_GTK) $(CFLAGS_STD) -msse2 -fPIC -O4 -g
LDFLAGS = $(LDFLAGS_VX) $(LDFLAGS_VX_GTK) $(LDFLAGS_IMAGESOURCE) $(LDFLAGS_COMMON) $(LDFLAGS_GLIB) $(LDFLAGS_GTK) $(LDFLAGS_LCMTYPES) $(LDFLAGS_STD) 

DYNAMIXEL_TEST = ../../bin/dynamixel_test
REXARM_DRIVER = ../../bin/rexarm_driver
REXARM_EXAMPLE = ../../bin/rexarm_example
LIB = ../../lib

all: $(DYNAMIXEL_TEST) $(REXARM_DRIVER) $(REXARM_EXAMPLE)


$(DYNAMIXEL_TEST): dynamixel_test.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(REXARM_DRIVER): rexarm_driver.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(REXARM_EXAMPLE): rexarm_example.o gui.o eecs467_util.o blob_detection.o disjoint.o lcmtypes/ball_info_t.c lcmtypes/ball_list_t.c lcmtypes/arm_action_t.c
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

#blob_detection.o: blob_detection.c blob_detection.h disjoint.c disjoint.h
#	@$(CC) -c blob_detection.c disjoint.c

clean:
	@rm -f *.o *~ *.a
	@rm -f $(DYNAMIXEL_TEST) $(REXARM_DRIVER) $(REXARM_EXAMPLE)
