F411_TARGETS += $(TARGET)
FEATURES     += VCP
TARGET_SRC  =                                       \
             drivers/rx/rx_cc2500.c                 \
             rx/cc2500_common.c                     \
             drivers/max7456.c                      \
             rx/cc2500_sfhss.c                      \
             rx/cc2500_frsky_shared.c               \
             rx/cc2500_frsky_d.c                    \
             rx/cc2500_frsky_x.c										\
						 rx/cc2500_redpine.c
