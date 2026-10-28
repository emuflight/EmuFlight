F405_TARGETS   += $(TARGET)
FEATURES       += VCP SDCARD
HSE_VALUE    = 16000000

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/max7456.c
