H730_TARGETS   += $(TARGET)
EXST            = yes
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/accgyro/accgyro_spi_icm426xx.c \
$(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
drivers/max7456.c \

# notice - this file was programmatically generated and may be incomplete.

# This resource file generated using https://github.com/nerdCopter/target-convert
# Commit: d7caf4a
