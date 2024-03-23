F7X2RE_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_mpu.c \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/accgyro/accgyro_spi_icm426xx.c \
drivers/barometer/barometer_ms5611.c \
drivers/barometer/barometer_bmp280.c \
drivers/compass/compass_hmc5883l.c \
drivers/light_led.h \
drivers/light_ws2811strip.c \
drivers/max7456.c \

# notice - this file was programmatically generated and may be incomplete.
# drivers for ms5611 & compass manually added

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: c4eda1b
