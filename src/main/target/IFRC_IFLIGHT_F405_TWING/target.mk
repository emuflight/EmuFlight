F405_TARGETS   += $(TARGET)
FEATURES       +=  VCP ONBOARDFLASH
TARGET_SRC = \
							drivers/barometer/barometer_bmp280.c \
							drivers/barometer/barometer_bmp085.c\
							drivers/barometer/barometer_ms5611.c\
							drivers/compass/compass_hmc5883l.c \
							drivers/compass/compass_qmc5883l.c \
							drivers/max7456.c

ifeq ($(TARGET), IFF4_TWIN_G_M)
TARGET_SRC += \
							drivers/accgyro/accgyro_mpu.c \
							drivers/accgyro/accgyro_spi_mpu6000.c
else
TARGET_SRC += \
							drivers/accgyro/accgyro_spi_icm20689.c
endif
