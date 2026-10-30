F411_TARGETS += $(TARGET)
FEATURES     = VCP

TARGET_SRC   = \
            drivers/accgyro/accgyro_spi_bmi160.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/nbd7456.c \
            drivers/vtx_rtc6705_soft_spi.c
