F7X2RE_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/accgyro/accgyro_spi_icm426xx.c \
drivers/max7456.c \

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 889ce5d
