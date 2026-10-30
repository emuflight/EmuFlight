F405_TARGETS   += $(TARGET)
FEATURES       +=  VCP ONBOARDFLASH

ifeq ($(TARGET), IFF4_TWIN_G_M)
TARGET_SRC += \
							drivers/accgyro/accgyro_mpu.c \
							drivers/accgyro/accgyro_spi_mpu6000.c
else
TARGET_SRC += \
							drivers/accgyro/accgyro_spi_icm20689.c
endif
