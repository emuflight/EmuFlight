F405_TARGETS    += $(TARGET) # STM32F405RGT

FEATURES    += ONBOARDFLASH VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu9250.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8963.c \
            drivers/flash_m25p16.c \
            drivers/max7456.c  \
            io/osd.c
