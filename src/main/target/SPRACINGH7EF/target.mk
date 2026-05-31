H730_TARGETS   += $(TARGET)
EXST            = yes
FEATURES       += VCP

TARGET_SRC = \
drivers/accgyro/accgyro_spi_icm426xx.c \
$(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
drivers/light_led.c \
drivers/light_ws2811strip.c \
drivers/pinio.c \
drivers/max7456.c \

# notice - this file was programmatically generated and may be incomplete.

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: d7caf4a
