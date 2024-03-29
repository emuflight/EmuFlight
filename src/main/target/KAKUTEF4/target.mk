F405_TARGETS    += $(TARGET)
FEATURES        += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/max7456.c

ifeq ($(TARGET), FLYWOOF405)
TARGET_SRC += drivers/accgyro/accgyro_spi_mpu6000.c
endif

ifeq ($(TARGET), KAKUTEF4V2)
TARGET_SRC += drivers/accgyro/accgyro_spi_mpu6000.c
endif
