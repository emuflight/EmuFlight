F7X2RE_TARGETS  += $(TARGET)
 FEATURES        += VCP ONBOARDFLASH

 TARGET_SRC = drivers/dma_spi_hal.c \
              drivers/accgyro/accgyro_imuf9001.c \
              drivers/max7456.c
