//select stm processor
F405_TARGETS   += $(TARGET)
F411_TARGETS    += $(TARGET)
F7X2RE_TARGETS  += $(TARGET)
F7X5XG_TARGETS += $(TARGET) //f745

//sdcard or flash or none, but keep VCP
FEATURES       += VCP ONBOARDFLASH SDCARD

//default just delete gyro unused
TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \ //mpu6000
						drivers/accgyro/accgyro_spi_mpu6000.c \ //spi mpu6000
						drivers/accgyro/accgyro_mpu6500.c \    //icm20602
						drivers/accgyro/accgyro_spi_mpu6500.c \ // spi icm20602
            drivers/accgyro/accgyro_spi_icm20689.c \ //icm20689
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c \
            drivers/max7456.c
