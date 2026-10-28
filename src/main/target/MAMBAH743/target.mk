H743_TARGETS   += $(TARGET)
FEATURES       += VCP SDCARD ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/accgyro/accgyro_spi_icm426xx.c \
drivers/accgyro/accgyro_spi_bmi270.c \
drivers/max7456.c \

# notice - this file was programmatically generated and may be incomplete.

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 215ae87 + 1 file changed, 31 insertions(+), 8 deletions(-)
