F7X2RE_TARGETS += $(TARGET)
ifeq ($(TARGET), KAKUTEF7MINIV2)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/max7456.c \
            drivers/pinio.c
