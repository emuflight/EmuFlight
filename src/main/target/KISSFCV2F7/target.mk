F7X2RE_TARGETS += $(TARGET)
LD_SCRIPT       = $(ROOT)/src/main/target/$(TARGET)/stm32_flash_f722_no_split.ld
CFLAGS +=        -DCLOCK_SOURCE_USE_HSI

