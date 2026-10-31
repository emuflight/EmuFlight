F405_TARGETS  += $(TARGET)
FEATURES    = VCP SDCARD

TARGET_SRC = \
            drivers/max7456.c

ifeq ($(TARGET), SPRACINGF4EVODG)
TARGET_SRC +=
endif

