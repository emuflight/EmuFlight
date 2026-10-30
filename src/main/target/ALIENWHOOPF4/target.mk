F405_TARGETS    += $(TARGET) # STM32F405RGT

FEATURES    += ONBOARDFLASH VCP

TARGET_SRC = \
            drivers/flash_m25p16.c \
            drivers/max7456.c  \
            io/osd.c
