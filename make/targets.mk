ALT_TARGETS       = $(sort $(filter-out target, $(basename $(notdir $(wildcard $(ROOT)/src/main/target/*/*.mk)))))
NOBUILD_TARGETS   = $(sort $(filter-out target, $(basename $(notdir $(wildcard $(ROOT)/src/main/target/*/*.nomk)))))
OPBL_TARGETS      = $(filter %_OPBL, $(ALT_TARGETS))

VALID_TARGETS   = $(dir $(wildcard $(ROOT)/src/main/target/*/target.h))
VALID_TARGETS  := $(subst /,, $(subst ./src/main/target/,, $(VALID_TARGETS)))
VALID_TARGETS  := $(VALID_TARGETS) $(ALT_TARGETS)
VALID_TARGETS  := $(sort $(VALID_TARGETS))
VALID_TARGETS  := $(filter-out $(NOBUILD_TARGETS), $(VALID_TARGETS))

ifeq ($(filter $(TARGET),$(NOBUILD_TARGETS)), $(TARGET))
ALTERNATES    := $(sort $(filter-out target, $(basename $(notdir $(wildcard $(ROOT)/src/main/target/$(TARGET)/*.mk)))))
$(error The target specified, $(TARGET), cannot be built. Use one of the ALT targets: $(ALTERNATES))
endif

UNSUPPORTED_TARGETS := \
    SITL \
    STM32F411DISCOVERY \
    STM32F4DISCOVERY \
    STM32F405 \
    STM32F411 \
    STM32F446 \
    STM32F745 \
    STM32F7X2 \
    STM32H743

SUPPORTED_TARGETS := $(filter-out $(UNSUPPORTED_TARGETS), $(VALID_TARGETS))

TARGETS_TOTAL := $(words $(SUPPORTED_TARGETS))
TARGET_GROUPS := 16

# Group K takes every TARGET_GROUPS-th target starting at K. Contiguous slices
# cluster same-MCU boards and unbalance the CI jobs.
group_slice = $(foreach i,$(shell seq $(1) $(TARGET_GROUPS) $(TARGETS_TOTAL)),$(word $(i),$(SUPPORTED_TARGETS)))

GROUP_1_TARGETS := $(call group_slice,1)
GROUP_2_TARGETS := $(call group_slice,2)
GROUP_3_TARGETS := $(call group_slice,3)
GROUP_4_TARGETS := $(call group_slice,4)
GROUP_5_TARGETS := $(call group_slice,5)
GROUP_6_TARGETS := $(call group_slice,6)
GROUP_7_TARGETS := $(call group_slice,7)
GROUP_8_TARGETS := $(call group_slice,8)
GROUP_9_TARGETS := $(call group_slice,9)
GROUP_10_TARGETS := $(call group_slice,10)
GROUP_11_TARGETS := $(call group_slice,11)
GROUP_12_TARGETS := $(call group_slice,12)
GROUP_13_TARGETS := $(call group_slice,13)
GROUP_14_TARGETS := $(call group_slice,14)
GROUP_15_TARGETS := $(call group_slice,15)

GROUP_16_TARGETS := $(call group_slice,16)

ifneq ($(words $(GROUP_1_TARGETS) $(GROUP_2_TARGETS) $(GROUP_3_TARGETS) $(GROUP_4_TARGETS) $(GROUP_5_TARGETS) $(GROUP_6_TARGETS) $(GROUP_7_TARGETS) $(GROUP_8_TARGETS) $(GROUP_9_TARGETS) $(GROUP_10_TARGETS) $(GROUP_11_TARGETS) $(GROUP_12_TARGETS) $(GROUP_13_TARGETS) $(GROUP_14_TARGETS) $(GROUP_15_TARGETS) $(GROUP_16_TARGETS)),$(TARGETS_TOTAL))
$(error Target groups do not cover all supported targets)
endif

ifeq ($(filter $(TARGET),$(ALT_TARGETS)), $(TARGET))
BASE_TARGET    := $(firstword $(subst /,, $(subst ./src/main/target/,, $(dir $(wildcard $(ROOT)/src/main/target/*/$(TARGET).mk)))))
include $(ROOT)/src/main/target/$(BASE_TARGET)/$(TARGET).mk
else
BASE_TARGET    := $(TARGET)
endif

ifeq ($(filter $(TARGET),$(OPBL_TARGETS)), $(TARGET))
OPBL            = yes
endif

# silently ignore if the file is not present. Allows for target specific.
-include $(ROOT)/src/main/target/$(BASE_TARGET)/target.mk

# Target header macros are read with the C preprocessor, as BF mk/config.mk does (pp_def_value).
# Host cc, not the ARM toolchain: it must work before the toolchain is installed (arm_sdk_install).
# MCU names are defined as themselves so a header's own empty "#define STM32F405" cannot erase FC_TARGET_MCU.
PP_CC          ?= cc
PP_MCU_NAMES   := STM32F405 STM32F411 STM32F446 STM32F7X2 STM32F745 STM32H743 STM32H723 STM32H725 STM32H730 STM32H750 SITL
PP_FLAGS       := $(foreach m,$(PP_MCU_NAMES),-D$(m)=$(m)) -I$(ROOT)/src/main -I$(ROOT)/src/main/target
TARGET_HEADER  := $(ROOT)/src/main/target/$(BASE_TARGET)/target.h

# $(call pp_def_value,NAME): expanded NAME from the target header; empty if undefined or no header.
pp_def_value = $(if $(wildcard $(TARGET_HEADER)),$(strip $(shell printf '\043if defined(%s)\n%s\n\043endif\n' $(1) $(1) | $(PP_CC) -E -P -xc $(PP_FLAGS) -imacros $(TARGET_HEADER) - 2>/dev/null | tr -d '\r\n')))

# The MCU group comes from FC_TARGET_MCU in the target header.
FC_TARGET_MCU := $(call pp_def_value,FC_TARGET_MCU)

ifneq ($(wildcard $(TARGET_HEADER)),)
ifeq ($(FC_TARGET_MCU),)
$(error Could not read FC_TARGET_MCU from $(TARGET_HEADER) with "$(PP_CC) -E". Is a C compiler installed?)
endif
endif

MCU_GROUP_STM32F405 := F405
MCU_GROUP_STM32F411 := F411
MCU_GROUP_STM32F446 := F446
MCU_GROUP_STM32F7X2 := F7X2RE
MCU_GROUP_STM32F745 := F7X5XG
MCU_GROUP_STM32H743 := H743
MCU_GROUP_STM32H723 := H723
MCU_GROUP_STM32H725 := H725
MCU_GROUP_STM32H730 := H730
MCU_GROUP_STM32H750 := H750
MCU_GROUP_SITL      := SITL

ifneq ($(MCU_GROUP_$(FC_TARGET_MCU)),)
$(eval $(MCU_GROUP_$(FC_TARGET_MCU))_TARGETS += $(TARGET))
endif

# SYSTEM_HSE_MHZ in the target header sets the crystal frequency; HSE_VALUE is that value in Hz.
SYSTEM_HSE_MHZ := $(call pp_def_value,SYSTEM_HSE_MHZ)

ifneq ($(SYSTEM_HSE_MHZ),)
ifneq ($(shell printf '%s\n' '$(SYSTEM_HSE_MHZ)' | grep -cE '^([2-9]|[1-5][0-9]|6[0-3])$$'),1)
$(error SYSTEM_HSE_MHZ "$(SYSTEM_HSE_MHZ)" in $(TARGET_HEADER) must be a whole number of MHz from 2 to 63 (PLL input divider range))
endif
HSE_VALUE := $(SYSTEM_HSE_MHZ)000000
endif

# FC_VMA_ADDRESS in the target header selects an EXST build and its load address.
FC_VMA_ADDRESS := $(call pp_def_value,FC_VMA_ADDRESS)

ifneq ($(FC_VMA_ADDRESS),)
EXST = yes
EXST_ADJUST_VMA := $(FC_VMA_ADDRESS)
endif

F4_TARGETS      := $(F405_TARGETS) $(F411_TARGETS) $(F446_TARGETS)
F7_TARGETS      := $(F7X2RE_TARGETS) $(F7X5XE_TARGETS) $(F7X5XG_TARGETS) $(F7X5XI_TARGETS) $(F7X6XG_TARGETS)
H7_TARGETS      := $(H743_TARGETS) $(H750_TARGETS) $(H723_TARGETS) $(H725_TARGETS) $(H730_TARGETS) $(H735_TARGETS) $(H7A3_TARGETS)

ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS). Have you prepared a valid target.h?)
endif

ifeq ($(filter $(TARGET),$(F4_TARGETS) $(F7_TARGETS) $(H7_TARGETS) $(SITL_TARGETS)),)
$(error Target '$(TARGET)' has no valid FC_TARGET_MCU in its target.h, must be one of STM32F405, STM32F411, STM32F446, STM32F7X2, STM32F745, STM32H743, STM32H723, STM32H725, STM32H730, STM32H750 or SITL)
endif

ifeq ($(TARGET),$(filter $(TARGET), $(F4_TARGETS)))
TARGET_MCU := STM32F4

else ifeq ($(TARGET),$(filter $(TARGET), $(F7_TARGETS)))
TARGET_MCU := STM32F7

else ifeq ($(TARGET),$(filter $(TARGET), $(SITL_TARGETS)))
TARGET_MCU := SITL
SIMULATOR_BUILD = yes

else ifeq ($(TARGET),$(filter $(TARGET), $(H7_TARGETS)))
TARGET_MCU := STM32H7
else
$(error Unknown target MCU specified.)
endif

TARGET_FLAGS  	:= $(TARGET_FLAGS) -D$(TARGET_MCU)
