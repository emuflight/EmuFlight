F7X2RE_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_mpu.c \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/accgyro/accgyro_spi_icm426xx.c \
drivers/light_led.h \
drivers/light_ws2811strip.c \
drivers/max7456.c \

# notice - this file was programmatically generated and may be incomplete.
# eg: flash, compass, barometer, vtx6705, ledstrip, pinio, etc.   especially mag/baro

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: bc7d9ef
