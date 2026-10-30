F411_TARGETS += $(TARGET)
FEATURES     = VCP ONBOARDFLASH

TARGET_SRC  =                                       \
             drivers/nbd7456.c                      \
             drivers/vtx_rtc6705_soft_spi.c         \
			 		 	 drivers/flash_m25p16.c                 \
             drivers/rx/rx_cc2500.c                \
             rx/cc2500_frsky_shared.c              \
             rx/cc2500_common.c                    \
             rx/cc2500_frsky_d.c                   \
             rx/cc2500_frsky_x.c                   \
             rx/cc2500_redpine.c                   \
             rx/cc2500_sfhss.c
