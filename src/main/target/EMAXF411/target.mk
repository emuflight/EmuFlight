F411_TARGETS    += $(TARGET)

FEATURES        += VCP ONBOARDFLASH

TARGET_SRC = \
						drivers/accgyro/accgyro_mpu.c \
						drivers/accgyro/accgyro_spi_mpu6000.c \
						drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/max7456.c \
            drivers/rx/rx_cc2500.c \
						rx/cc2500_common.c \
            rx/cc2500_frsky_shared.c \
            rx/cc2500_frsky_d.c \
						rx/cc2500_frsky_x.c \
						rx/cc2500_redpine.c \
            rx/cc2500_sfhss.c\
            drivers/rx/rx_a7105.c \
						drivers/barometer/barometer_bmp280.c \
						drivers/barometer/barometer_bmp085.c \
						drivers/barometer/barometer_ms5611.c \
						drivers/compass/compass_hmc5883l.c \
						drivers/compass/compass_qmc5883l.c\
						drivers/light_ws2811strip.c
