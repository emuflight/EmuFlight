F411_TARGETS    += $(TARGET)
FEATURES        += VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
						drivers/barometer/barometer_bmp085.c \
						drivers/barometer/barometer_bmp280.c \
						drivers/barometer/barometer_ms5611.c \
            drivers/max7456.c
