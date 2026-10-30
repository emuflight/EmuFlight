F405_TARGETS   += $(TARGET)
FEATURES       += VCP SDCARD

TARGET_SRC = \
drivers/accgyro/accgyro_mpu.c \
drivers/accgyro/accgyro_spi_bmi270.c \

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 1bce24c 
