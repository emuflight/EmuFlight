F411_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_spi_mpu6000.c \
drivers/accgyro/accgyro_spi_bmi270.c \
drivers/light_led.h \
drivers/light_ws2811strip.c \
drivers/max7456.c \
drivers/rx/rx_cc2500.c \
rx/cc2500_common.c \
rx/cc2500_frsky_shared.c \
rx/cc2500_frsky_d.c \
rx/cc2500_frsky_x.c \
rx/cc2500_redpine.c \
rx/cc2500_sfhss.c

# notice - this file was programmatically generated and may be incomplete.

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: 3f33ae6 + 1 file changed, 24 deletions(-)
