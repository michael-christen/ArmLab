include ../common.mk

CFLAGS = $(CFLAGS_COMMON) $(CFLAGS_GLIB) $(CFLAGS_GTK) $(CFLAGS_STD) -msse2 -fPIC -O4
LDFLAGS = $(LDFLAGS_COMMON) $(LDFLAGS_GLIB) $(LDFLAGS_GTK) $(LDFLAGS_LCMTYPES) $(LDFLAGS_STD)

DYNAMIXEL_TEST = ../../bin/dynamixel_test
REXARM_DRIVER = ../../bin/rexarm_driver
REXARM_EXAMPLE = ../../bin/rexarm_example

all: $(DYNAMIXEL_TEST) $(REXARM_DRIVER) $(REXARM_EXAMPLE)


$(DYNAMIXEL_TEST): dynamixel_test.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(REXARM_DRIVER): rexarm_driver.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(REXARM_EXAMPLE): rexarm_example.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(DYNAMIXEL_TEST) $(REXARM_DRIVER) $(REXARM_EXAMPLE)
