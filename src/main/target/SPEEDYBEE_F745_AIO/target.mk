F7X5XG_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_mpu.c \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/barometer/barometer_bmp280.c \
drivers/light_led.h \
drivers/light_ws2811strip.c \
drivers/pinio.c \
drivers/max7456.c \
drivers/barometer/barometer_bmp085.c \
drivers/barometer/barometer_bmp280.c \
drivers/barometer/barometer_fake.c \
drivers/barometer/barometer_lps.c \
drivers/barometer/barometer_ms5611.c \
drivers/barometer/barometer_qmp6988.c \

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 4cb7ad1 
