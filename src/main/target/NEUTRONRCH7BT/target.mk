H743_TARGETS   += $(TARGET)
FEATURES       += VCP SDCARD ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_spi_bmi270.c \
drivers/barometer/barometer_dps310.c \
$(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
drivers/light_led.h \
drivers/light_ws2811strip.c \
drivers/pinio.c \
drivers/max7456.c \

# notice - this file was programmatically generated and may be incomplete.

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 215ae87 + 1 file changed, 31 insertions(+), 8 deletions(-)
