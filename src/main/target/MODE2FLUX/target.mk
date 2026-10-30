
F405_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = stm32f4xx_crc.c \
             drivers/max7456.c