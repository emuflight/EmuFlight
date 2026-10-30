F411_TARGETS += $(TARGET)
FEATURES     = VCP ONBOARDFLASH

TARGET_SRC  =                                       \
             drivers/nbd7456.c                      \
             drivers/vtx_rtc6705_soft_spi.c         \
             drivers/flash_m25p16.c
