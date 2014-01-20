include ../common.mk

# Construct the pathnames to objs
LCMTYPES_NAMES := $(shell cd ../../lcmtypes && ls *.lcm)
LCMTYPES_OBJS := $(LCMTYPES_NAMES:%.lcm=%.o)

LIBLCMTYPES=$(LIB_PATH)/liblcmtypes.a

all: $(LIBLCMTYPES)

$(LIBLCMTYPES): $(LCMTYPES_OBJS)
	@echo "\t$@"
	@ar rc $@ $(LCMTYPES_OBJS)

clean:
	@rm -f *~ *.o *.a *.h *.c
