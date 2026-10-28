F411_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/max7456.c \

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 522aa66
