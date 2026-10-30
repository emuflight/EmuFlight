F405_TARGETS   += $(TARGET)
FEATURES       += VCP SDCARD

TARGET_SRC = \
drivers/accgyro/accgyro_mpu.c \
drivers/accgyro/accgyro_spi_icm426xx.c \

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 34cc79d
