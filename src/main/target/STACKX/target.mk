F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/max7456.c \
            drivers/vtx_rtc6705_soft_spi.c
