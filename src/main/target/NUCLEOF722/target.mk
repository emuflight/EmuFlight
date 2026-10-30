F7X2RE_TARGETS += $(TARGET)
FEATURES       += VCP SDIO

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/light_ws2811strip_hal.c
