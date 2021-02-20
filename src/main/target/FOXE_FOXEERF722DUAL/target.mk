F7X2RE_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/max7456.c

ifeq ($(TARGET), FOXEERF722V2)
TARGET_SRC += \
drivers/accgyro/accgyro_mpu.c \
drivers/accgyro/accgyro_spi_mpu6000.c \

else
TARGET_SRC += \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_mpu6500.c

endif
