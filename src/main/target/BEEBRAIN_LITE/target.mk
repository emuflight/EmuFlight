F411_TARGETS += $(TARGET)
FEATURES     = VCP ONBOARDFLASH

TARGET_SRC  =                                       \
             drivers/accgyro/accgyro_mpu.c          \
             drivers/accgyro/accgyro_mpu6500.c      \
             drivers/accgyro/accgyro_spi_mpu6500.c  \
             drivers/barometer/barometer_bmp280.c   \
             drivers/nbd7456.c                      \
             drivers/vtx_rtc6705_soft_spi.c         \
             drivers/flash_m25p16.c

ifneq ($(TARGET), BEEBRAIN_LITED)
TARGET_SRC += drivers/rx/rx_cc2500.c					 		  \
					  	rx/cc2500_common.c 										\
              rx/cc2500_frsky_shared.c 							\
              rx/cc2500_frsky_d.c 									\
					  	rx/cc2500_frsky_x.c										\
							rx/cc2500_redpine.c
endif
