F7X2RE_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_mpu.c \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/accgyro/accgyro_spi_icm20689.c \
drivers/accgyro/accgyro_spi_icm426xx.c \
drivers/barometer/barometer_bmp280.c \
drivers/light_led.h \
drivers/light_ws2811strip.c \
drivers/light_ws2811strip_hal.c \
drivers/pinio.c \
drivers/max7456.c \

# notice - this file was programmatically generated and may be incomplete.
#  eg: flash, compass, barometer, vtx6705, ledstrip, pinio, etc.
# existing src/main/target/JHEF7DUAL contains compass_fake, does not contain light_led, does not contain pinio