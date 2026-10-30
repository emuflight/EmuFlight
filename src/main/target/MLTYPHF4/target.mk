F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

TARGET_SRC = \
            drivers/max7456.c \
            drivers/vtx_rtc6705_soft_spi.c
