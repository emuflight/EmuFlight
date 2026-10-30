F405_TARGETS   += $(TARGET)
FEATURES       +=  VCP ONBOARDFLASH
TARGET_SRC = \
							drivers/max7456.c

ifeq ($(TARGET), IFF4_TWIN_G_M)
TARGET_SRC +=
else
TARGET_SRC +=
endif
