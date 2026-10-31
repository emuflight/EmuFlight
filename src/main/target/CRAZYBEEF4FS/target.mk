F411_TARGETS    += $(TARGET)

FEATURES        += VCP SDCARD

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/accgyro/accgyro_spi_icm426xx.c \
            drivers/accgyro/accgyro_spi_bmi270.c \
            drivers/max7456.c
