F7X5XG_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
						drivers/accgyro/accgyro_mpu.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
					  drivers/compass/compass_lis3mdl.c \
            drivers/light_ws2811strip.c \
            drivers/max7456.c
