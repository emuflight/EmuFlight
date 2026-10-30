F7X2RE_TARGETS += $(TARGET)
FEATURES    = VCP SDCARD

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705_soft_spi.c


