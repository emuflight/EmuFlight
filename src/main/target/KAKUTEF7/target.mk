F7X5XG_TARGETS += $(TARGET)
ifeq ($(TARGET), KAKUTEF7MINIV1)
FEATURES       += VCP ONBOARDFLASH
else
FEATURES       += SDCARD VCP
endif

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c \
            drivers/max7456.c
