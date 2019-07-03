#
# ACRN Hypervisor test framework
#

T := $(CURDIR)

HV_OBJDIR ?= $(CURDIR)/build
HV_MODDIR ?= $(HV_OBJDIR)/modules
HV_FILE := acrn
BASEDIR := $(HV_ROOTDIR)

LIB_MOD = $(HV_MODDIR)/lib_mod.a
BOOT_MOD = $(HV_MODDIR)/boot_mod.a
HW_MOD = $(HV_MODDIR)/hw_mod.a
VP_BASE_MOD = $(HV_MODDIR)/vp_base_mod.a
VP_DM_MOD = $(HV_MODDIR)/vp_dm_mod.a
LIB_DEBUG = $(HV_MODDIR)/libdebug.a

include $(HV_ROOTDIR)/scripts/kconfig/kconfig.mk

ARFLAGS += crs

CFLAGS += -Wall -W
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fshort-wchar -ffreestanding
CFLAGS += -fsigned-char
CFLAGS += -m64 -mno-mmx -mno-sse -mno-sse2 -mno-80387 -mno-fp-ret-in-387
CFLAGS += -mno-red-zone
CFLAGS += -nostdinc -nostdlib -fno-common
CFLAGS += -Werror
CFLAGS += -fpie

ASFLAGS += -m64 -nostdinc -nostdlib

LDFLAGS += -Wl,--gc-sections -nostartfiles -nostdlib
LDFLAGS += -Wl,-n,-z,max-page-size=0x1000
LDFLAGS += -pie -z noreloc-overflow

LD_IN_TOOL = $(HV_ROOTDIR)/scripts/genld.sh
ARCH_LDSCRIPT = $(HV_ROOTDIR)/link_ram.ld
#ARCH_LDSCRIPT_IN = $(HV_ROOTDIR)/bsp/ld/link_ram.ld.in
BASH = $(shell which bash)

INCLUDE_PATH += $(HV_ROOTDIR)/include
INCLUDE_PATH += $(HV_ROOTDIR)/include/lib
INCLUDE_PATH += $(HV_ROOTDIR)/include/lib/crypto
INCLUDE_PATH += $(HV_ROOTDIR)/include/common
INCLUDE_PATH += $(HV_ROOTDIR)/include/arch/x86
INCLUDE_PATH += $(HV_ROOTDIR)/include/arch/x86/boot
INCLUDE_PATH += $(HV_ROOTDIR)/include/arch/x86/guest
INCLUDE_PATH += $(HV_ROOTDIR)/include/arch/x86/lib
INCLUDE_PATH += $(HV_ROOTDIR)/include/debug
INCLUDE_PATH += $(HV_ROOTDIR)/debug
INCLUDE_PATH += $(HV_ROOTDIR)/include/public
INCLUDE_PATH += $(HV_ROOTDIR)/include/dm
INCLUDE_PATH += $(HV_ROOTDIR)/include/hw
INCLUDE_PATH += $(HV_ROOTDIR)/boot/include
INCLUDE_PATH += $(HV_ROOTDIR)/boot/include/guest
INCLUDE_PATH += $(HV_OBJDIR)/include
INCLUDE_PATH += $(HV_ROOTDIR)/arch/x86/configs/$(CONFIG_BOARD)
INCLUDE_PATH += $(HV_ROOTDIR)/scenarios/$(SCENARIO_NAME)

CC ?= gcc
AS ?= as
AR ?= ar
LD ?= ld
OBJCOPY ?= objcopy

INCLUDE_PATH += include
STUB_C_SRCS += src/uart.c
#STUB_C_SRCS += src/sprintf.c
STUB_C_SRCS += src/test_printf.c
ifeq ("$(MAKECMDGOALS)", "boot-bin")
	STUB_C_SRCS += src/boot_stub.c
	MODULES += $(LIB_MOD)
	MODULES += $(BOOT_MOD)
endif
ifeq ("$(MAKECMDGOALS)", "hw-bin")
	STUB_C_SRCS += src/hw_stub.c
	MODULES += $(LIB_MOD)
	MODULES += $(BOOT_MOD)
	MODULES += $(HW_MOD)
endif

ifeq ("$(MAKECMDGOALS)", "vp-base-bin")
	STUB_C_SRCS += src/vp_base_stub.c
	MODULES += $(LIB_MOD)
	MODULES += $(BOOT_MOD)
	MODULES += $(HW_MOD)
	MODULES += $(VP_BASE_MOD)
endif

ifeq ("$(MAKECMDGOALS)", "vp-dm-bin")
	STUB_C_SRCS += src/vp_dm_stub.c
	MODULES += $(LIB_MOD)
	MODULES += $(BOOT_MOD)
	MODULES += $(HW_MOD)
	MODULES += $(VP_BASE_MOD)
	MODULES += $(VP_DM_MOD)
	MODULES += $(LIB_DEBUG)
endif


STUB_C_OBJS := $(patsubst %.c,$(HV_OBJDIR)/%.o,$(STUB_C_SRCS))

.PHONY: lib boot hw vp_base vp_dm
lib:
	$(MAKE) -C $(HV_ROOTDIR) HV_OBJDIR=$(HV_OBJDIR) BOARD=$(BOARD) \
		FIRMWARE=$(FIRMWARE) RELEASE=$(RELEASE) lib-mod
boot:
	$(MAKE) -C $(HV_ROOTDIR) HV_OBJDIR=$(HV_OBJDIR) BOARD=$(BOARD) \
		FIRMWARE=$(FIRMWARE) RELEASE=$(RELEASE) boot-mod
hw:
	$(MAKE) -C $(HV_ROOTDIR) HV_OBJDIR=$(HV_OBJDIR) BOARD=$(BOARD) \
		FIRMWARE=$(FIRMWARE) RELEASE=$(RELEASE) hw-mod
vp_base:
	$(MAKE) -C $(HV_ROOTDIR) HV_OBJDIR=$(HV_OBJDIR) BOARD=$(BOARD) \
		FIRMWARE=$(FIRMWARE) RELEASE=$(RELEASE) vp-base-mod
vp_dm:
	$(MAKE) -C $(HV_ROOTDIR) HV_OBJDIR=$(HV_OBJDIR) BOARD=$(BOARD) \
		FIRMWARE=$(FIRMWARE) RELEASE=$(RELEASE) lib
	$(MAKE) -C $(HV_ROOTDIR) HV_OBJDIR=$(HV_OBJDIR) BOARD=$(BOARD) \
		FIRMWARE=$(FIRMWARE) RELEASE=$(RELEASE) vp-dm-mod

$(LIB_MOD): 	lib
$(BOOT_MOD):	boot
$(HW_MOD):   	hw
$(VP_BASE_MOD): vp_base
$(VP_DM_MOD):   vp_dm

.PHONY: boot-bin hw-bin vp-base-bin vp-dm-bin
boot-bin hw-bin vp-base-bin vp-dm-bin: \
	$(HV_OBJDIR)/$(HV_FILE).32.out $(HV_OBJDIR)/$(HV_FILE).bin

$(HV_OBJDIR)/$(HV_FILE).32.out: $(HV_OBJDIR)/$(HV_FILE).out
	$(OBJCOPY) -S --section-alignment=0x1000 -O elf32-i386 $< $@

$(HV_OBJDIR)/$(HV_FILE).bin: $(HV_OBJDIR)/$(HV_FILE).out
	$(OBJCOPY) -O binary $< $(HV_OBJDIR)/$(HV_FILE).bin

	cp -f $(HV_OBJDIR)/$(HV_FILE).* $(HV_ROOTDIR)/../../out/hv/apl-mrb-sbl/hypervisor/
ifeq ($(BOARD),apl-mrb)
	#cp -f $(HV_OBJDIR)/$(HV_FILE).* $(HV_ROOTDIR)/../../out/hv/apl-mrb-sbl/hypervisor/
else
#	$(MAKE) -C $(BASEDIR)/../efi-stub HV_OBJDIR=$(HV_OBJDIR) EFI_OBJDIR=$(HV_OBJDIR)/efi-stub
#	cp -f $(HV_OBJDIR)/$(HV_FILE).* $(HV_ROOTDIR)/../../out/hv/apl-nuc-uefi/hypervisor/
endif
$(HV_OBJDIR)/$(HV_FILE).out: $(MODULES) $(STUB_C_OBJS)
#	${BASH} ${LD_IN_TOOL} $(ARCH_LDSCRIPT_IN) $(ARCH_LDSCRIPT) ${HV_OBJDIR}/.config
	$(CC) -Wl,-Map=$(HV_OBJDIR)/$(HV_FILE).map -o $@ $(LDFLAGS) $(ARCH_LDFLAGS) \
		-T$(ARCH_LDSCRIPT) -Wl,--start-group $^ -Wl,--end-group

.PHONY: clean
clean:
	rm -rf $(HV_OBJDIR)
$(HV_OBJDIR)/%.o: %.c $(HV_OBJDIR)/$(HV_CONFIG_H)
	[ ! -e $@ ] && mkdir -p $(dir $@); \
	$(CC) $(patsubst %, -I%, $(INCLUDE_PATH)) -c $(CFLAGS) $(ARCH_CFLAGS) $< -o $@ -MMD -MT $@
