OFFICIAL_TARGETS  = ALIENFLIGHTF3 ALIENFLIGHTF4 ANYFCF7 BETAFLIGHTF3 BLUEJAYF4 FURYF4 REVO SIRINFPV SPARKY SPRACINGF3 SPRACINGF3EVO SPRACINGF3NEO SPRACINGF4EVO SPRACINGF7DUAL STM32F3DISCOVERY
ALT_TARGETS       = $(sort $(filter-out target, $(basename $(notdir $(wildcard $(ROOT)/src/main/target/*/*.mk)))))
NOBUILD_TARGETS   = $(sort $(filter-out target, $(basename $(notdir $(wildcard $(ROOT)/src/main/target/*/*.nomk)))))
OPBL_TARGETS      = $(filter %_OPBL, $(ALT_TARGETS))

VALID_TARGETS   = $(dir $(wildcard $(ROOT)/src/main/target/*/target.mk))
VALID_TARGETS  := $(subst /,, $(subst ./src/main/target/,, $(VALID_TARGETS)))
VALID_TARGETS  := $(VALID_TARGETS) $(ALT_TARGETS)
VALID_TARGETS  := $(sort $(VALID_TARGETS))
VALID_TARGETS  := $(filter-out $(NOBUILD_TARGETS), $(VALID_TARGETS))

ifeq ($(filter $(TARGET),$(NOBUILD_TARGETS)), $(TARGET))
ALTERNATES    := $(sort $(filter-out target, $(basename $(notdir $(wildcard $(ROOT)/src/main/target/$(TARGET)/*.mk)))))
$(error The target specified, $(TARGET), cannot be built. Use one of the ALT targets: $(ALTERNATES))
endif

UNSUPPORTED_TARGETS := \
    AFROMINI \
    AIORACERF3 \
    AIR32 \
    AIRHEROF3 \
    ALIENFLIGHTF1 \
    ALIENFLIGHTF3 \
    BEEBRAIN \
    BEEBRAIN_V2D \
    BEEBRAIN_V2F \
    BEESTORM \
    BETAFLIGHTF3 \
    CC3D \
    CC3D_OPBL \
    CHEBUZZF3 \
    CJMCU \
    COLIBRI_RACE \
    CRAZYBEEF3DX \
    CRAZYBEEF3FR \
    CRAZYBEEF3FS \
    DOGE EACHIF3 \
    FF_ACROWHOOPSP \
    FF_KOMBINI \
    FF_PIKOBLX \
    FF_RADIANCE \
    FLIP32F3OSD \
    FRSKYF3 \
    FURYF3 \
    FURYF3OSD \
    FURYF4 \
    IMPULSERCF3 \
    IRCFUSIONF3 \
    IRCSYNERGYF3 \
    ISHAPEDF3 \
    KISSCC \
    KISSFC \
    LUMBAF3 \
    LUX_RACE \
    LUXV2_RACE \
    MICROSCISKY \
    MIDELICF3 \
    MOTOLAB \
    MULTIFLITEPICO \
    NAZE \
    OMNIBUS \
    RACEBASE \
    RCEXPLORERF3 \
    RG_SSD_F3 \
    RMDO \
    SINGULARITY \
    SIRINFPV \
    SITL \
    SPARKY \
    SPRACINGF3 \
    SPRACINGF3EVO \
    SPRACINGF3MINI \
    SPRACINGF3MQ \
    SPRACINGF3NEO \
    STM32F3DISCOVERY \
    STM32F411DISCOVERY \
    STM32F4DISCOVERY \
    STM32F7X2 \
    TINYBEEF3 \
    TINYFISH \
    X_RACERSPI \
    ZCOREF3 \
    CI_STM32F3 \
    CI_STM32F405 \
    CI_STM32F411 \
    CI_STM32F446 \
    CI_STM32F7X2 \
    CI_STM32F7X6

# Synthetic CI targets — compile-only, not flashable, excluded from release builds.
# CI_STM32F3 excluded from CI_TARGETS and UNSUPPORTED (no release build): all F3 targets exceed
# RAM budget in EF master (pre-existing). Target files kept for reference and manual builds.
# See: https://github.com/emuflight/EmuFlight/issues/1190 — tracking F3 RAM fix.
# HELIOSPRING and STRIXF10 are real product boards (not synthetic); they cover the
# IMUF9001 dual-MCU code path and are intentionally included in release builds.
CI_TARGETS := CI_STM32F405 CI_STM32F411 CI_STM32F446 CI_STM32F7X2 CI_STM32F7X6 HELIOSPRING STRIXF10

.PHONY: targets-ci-print targets-ci

targets-ci-print:
	@echo $(CI_TARGETS)

targets-ci: $(CI_TARGETS)

SUPPORTED_TARGETS := $(filter-out $(UNSUPPORTED_TARGETS), $(VALID_TARGETS))

TARGETS_TOTAL := $(words $(SUPPORTED_TARGETS))
TARGET_GROUPS := 8
TARGETS_PER_GROUP := $(shell expr $(TARGETS_TOTAL) / $(TARGET_GROUPS) )

ST := 1
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_1_TARGETS := $(wordlist $(ST), $(ET), $(SUPPORTED_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_2_TARGETS := $(wordlist $(ST), $(ET), $(SUPPORTED_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_3_TARGETS := $(wordlist $(ST), $(ET), $(SUPPORTED_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_4_TARGETS := $(wordlist $(ST), $(ET), $(SUPPORTED_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_5_TARGETS := $(wordlist $(ST), $(ET), $(SUPPORTED_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_6_TARGETS := $(wordlist $(ST), $(ET), $(SUPPORTED_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_7_TARGETS := $(wordlist $(ST), $(ET), $(SUPPORTED_TARGETS))

GROUP_OTHER_TARGETS := $(filter-out $(GROUP_1_TARGETS) $(GROUP_2_TARGETS) $(GROUP_3_TARGETS) $(GROUP_4_TARGETS) $(GROUP_5_TARGETS) $(GROUP_6_TARGETS) $(GROUP_7_TARGETS), $(SUPPORTED_TARGETS))

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

F4_TARGETS      := $(F405_TARGETS) $(F411_TARGETS) $(F446_TARGETS)
F7_TARGETS      := $(F7X2RE_TARGETS) $(F7X5XE_TARGETS) $(F7X5XG_TARGETS) $(F7X5XI_TARGETS) $(F7X6XG_TARGETS)

ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS). Have you prepared a valid target.mk?)
endif

ifeq ($(filter $(TARGET),$(F1_TARGETS) $(F3_TARGETS) $(F4_TARGETS) $(F7_TARGETS) $(SITL_TARGETS)),)
$(error Target '$(TARGET)' has not specified a valid STM group, must be one of F1, F3, F405, F411 or F7x5. Have you prepared a valid target.mk?)
endif

ifeq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
TARGET_MCU := STM32F3

else ifeq ($(TARGET),$(filter $(TARGET), $(F4_TARGETS)))
TARGET_MCU := STM32F4

else ifeq ($(TARGET),$(filter $(TARGET), $(F7_TARGETS)))
TARGET_MCU := STM32F7

else ifeq ($(TARGET),$(filter $(TARGET), $(SITL_TARGETS)))
TARGET_MCU := SITL
SIMULATOR_BUILD = yes

else ifeq ($(TARGET),$(filter $(TARGET), $(F1_TARGETS)))
TARGET_MCU := STM32F1
else
$(error Unknown target MCU specified.)
endif

ifneq ($(BASE_TARGET), $(TARGET))
TARGET_FLAGS  	:= $(TARGET_FLAGS) -D$(BASE_TARGET)
endif

TARGET_FLAGS  	:= $(TARGET_FLAGS) -D$(TARGET_MCU)
