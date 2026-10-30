F7X2RE_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_mpu.c \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/accgyro/accgyro_spi_icm426xx.c \

# notice - this file was programmatically generated and may be incomplete.
# drivers for ms5611 & compass manually added

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: c4eda1b
