H750_TARGETS   += $(TARGET)
EXST            = yes
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_mpu6500.c \
drivers/accgyro/accgyro_spi_mpu6500.c \
$(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
drivers/light_led.c \
drivers/light_ws2811strip.c \
drivers/max7456.c \
