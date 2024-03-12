F7X2RE_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_mpu.c \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/accgyro/accgyro_mpu6500.c \
drivers/accgyro/accgyro_spi_mpu6500.c \
drivers/accgyro/accgyro_spi_icm426xx.c \
drivers/accgyro/accgyro_spi_bmi270.c \
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

# notice - this file was programmatically generated and may be incomplete.
# eg: flash, compass, barometer, vtx6705, ledstrip, pinio, etc.   especially mag/baro

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 4cb7ad1 

