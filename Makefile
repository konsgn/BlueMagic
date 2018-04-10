##########################################
# Makefile Adapted from Black Magic Probe
##########################################

BINARY = bluemagic

ENABLE_DEBUG ?=

ifneq ($(V), 1)
MAKEFLAGS += --no-print-dir
Q := @
endif



ENABLE_DEBUG ?=

ifneq ($(V), 1)
MAKEFLAGS += --no-print-dir
Q := @
endif

OPT_FLAGS ?= -O2

CFLAGS += -Wall -Wextra -Werror -Wno-char-subscripts\
	$(OPT_FLAGS) -std=gnu99 -g3 -MD \
	-I. -Istm32/include -mcpu=cortex-m3 -mthumb \
	-DSTM32F1 -Ilibopencm3/include

ifeq ($(ENABLE_DEBUG), 1)
CFLAGS += -DENABLE_DEBUG
endif

CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
	
LDFLAGS_BOOT := $(OPT_FLAGS) -lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,bluemagic.ld -nostartfiles -lc \
	-Wl,-Map=mapfile -mthumb -Istm32/include -mcpu=cortex-m3 -Wl,-gc-sections \
	-Llibopencm3/lib
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x08002000

LDFLAGS += -specs=nosys.specs

SRC =			\
	usbbulk.c	\
	csrspi.c	\
	main.c		

OBJ = $(SRC:.c=.o)

$(BINARY).elf: $(OBJ)
	@echo "  LD      $@"
	$(Q)$(CC) -o $@ $(OBJ) $(LDFLAGS)

%.o:	%.c
	@echo "  CC      $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

%.bin:	%.elf
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $^ $@

%.hex:	%.elf
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O ihex $^ $@

.PHONY:	clean all

clean:	
	$(Q)echo "  CLEAN"
	-$(Q)$(RM) -f *.o *.d *.hex *.elf $(BINARY) mapfile

all: $(BINARY).bin $(BINARY).hex

#   all_platforms:
#   	$(Q)set -e ;\
#   	mkdir -p artifacts/$(shell git describe --always) ;\
#   	echo "<html><body><ul>" > artifacts/index.html ;\
#   	for i in platforms/*/Makefile.inc ; do \
#   		export DIRNAME=`dirname $$i` ;\
#   		export PROBE_HOST=`basename $$DIRNAME` ;\
#   		export CFLAGS=-Werror ;\
#   		echo "Building for hardware platform: $$PROBE_HOST" ;\
#   		$(MAKE) $(MAKEFLAGS) clean ;\
#   		$(MAKE) $(MAKEFLAGS);\
#   		if [ -f blackmagic.bin ]; then \
#   			mv blackmagic.bin artifacts/blackmagic-$$PROBE_HOST.bin ;\
#   			echo "<li><a href='blackmagic-$$PROBE_HOST.bin'>$$PROBE_HOST</a></li>"\
#   				>> artifacts/index.html ;\
#   		fi ;\
#   	done ;\
#   	echo "</ul></body></html>" >> artifacts/index.html ;\
#   	cp artifacts/*.bin artifacts/$(shell git describe --always)

#command.c: version.h

#version.h: FORCE
#	$(Q)echo "#define FIRMWARE_VERSION 0.01" > $@

#-include *.d
