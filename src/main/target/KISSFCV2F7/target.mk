# Firmware starts at 0x08008000 after the KISS bootloader; vectors live in the firmware region.
LD_SCRIPT       = $(ROOT)/src/main/target/$(TARGET)/stm32_flash_f722_no_split.ld
# HSI as the PLL source (read by system_stm32f7xx.c).
CFLAGS +=        -DCLOCK_SOURCE_USE_HSI
