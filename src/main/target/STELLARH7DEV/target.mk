H743_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_spi_icm426xx.c \
$(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
drivers/light_ws2811strip.c \
drivers/pinio.c \
drivers/max7456.c
