F446_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD CHIBIOS BRAINFPV SPECTROGRAPH

HSE_VALUE       = 16000000

TARGET_SRC =  \
              drivers/accgyro/accgyro_spi_bmi160.c \
              io/osd.c \
              drivers/light_ws2811strip.c \
              drivers/barometer/barometer_bmp280.c \
