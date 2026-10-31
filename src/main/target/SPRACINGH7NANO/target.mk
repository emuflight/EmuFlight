H750_TARGETS   += $(TARGET)
EXST            = yes
EXST_ADJUST_VMA = 0x97CE0000
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
drivers/light_led.c \
drivers/light_ws2811strip.c \
