F7X2RE_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/barometer/barometer_bmp280.c \
$(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
drivers/light_led.h \
drivers/light_ws2811strip.c \
drivers/pinio.c \
drivers/max7456.c \

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 522aa66
