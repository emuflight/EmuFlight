F7X5XG_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_mpu.c \
drivers/accgyro/accgyro_spi_icm426xx.c \

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 40b5552
