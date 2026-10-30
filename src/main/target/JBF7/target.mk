F7X2RE_TARGETS  += $(TARGET)
FEATURES        +=  VCP SDCARD ONBOARDFLASH
TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
						drivers/light_ws2811strip_hal.c
