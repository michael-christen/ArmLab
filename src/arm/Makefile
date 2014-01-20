include ../common.mk

CFLAGS = $(CFLAGS_COMMON) $(CFLAGS_GLIB) $(CFLAGS_GTK) $(CFLAGS_STD) -msse2 -fPIC -O4
LDFLAGS = $(LDFLAGS_COMMON) $(LDFLAGS_GLIB) $(LDFLAGS_GTK) $(LDFLAGS_LCMTYPES) $(LDFLAGS_STD)

all: dynamixel_test arm_driver arm_example


dynamixel_test: dynamixel_test.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

arm_driver: arm_driver.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)
	@cp arm_driver $(BIN_PATH)

arm_example: arm_example.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)
	@cp arm_driver $(BIN_PATH)

clean:
	@rm -f *.o *~ *.a
	@rm -f dynamixel_test arm_driver arm_example
