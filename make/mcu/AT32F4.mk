#
# AT32F4 Make file include
#

CMSIS_DIR       := $(ROOT)/lib/main/AT32F43x/cmsis
STDPERIPH_DIR   = $(ROOT)/lib/main/AT32F43x/drivers
MIDDLEWARES_DIR = $(ROOT)/lib/main/AT32F43x/middlewares

EXCLUDES        = at32f435_437_dvp.c \
                  at32f435_437_can.c \
                  at32f435_437_xmc.c \
                  at32f435_437_emac.c

USBDRV_DIR      = $(MIDDLEWARES_DIR)/usb_drivers
USBMSC_DIR      = $(MIDDLEWARES_DIR)/usbd_class/msc
USBCDC_DIR      = $(MIDDLEWARES_DIR)/usbd_class/cdc
I2CAPP_DIR      = $(MIDDLEWARES_DIR)/i2c_application_library

STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))
USBDRV_SRC      = $(notdir $(wildcard $(USBDRV_DIR)/src/*.c))
USBMSC_SRC      = $(notdir $(wildcard $(USBMSC_DIR)/*.c))

VPATH           := $(VPATH):$(CMSIS_DIR)/cm4/core_support:$(STDPERIPH_DIR)/src:$(STDPERIPH_DIR)/inc:$(SRC_DIR)/startup/at32:$(USBDRV_DIR)/src:$(USBMSC_DIR):$(USBCDC_DIR):$(I2CAPP_DIR)

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBDRV_SRC) \
                        $(USBMSC_SRC)

# The EF CMSIS-DSP arm_math.h must win over the copy in the vendor core_support directory.
INCLUDE_DIRS    := $(ROOT)/lib/main/CMSIS/DSP/Include \
                   $(INCLUDE_DIRS) \
                   $(SRC_DIR)/startup/at32 \
                   $(SRC_DIR)/drivers/at32 \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/cm4/core_support \
                   $(CMSIS_DIR)/cm4 \
                   $(I2CAPP_DIR) \
                   $(USBDRV_DIR)/inc \
                   $(USBMSC_DIR) \
                   $(USBCDC_DIR)

ifeq ($(TARGET),$(filter $(TARGET),$(AT32F435G_TARGETS)))
DEVICE_FLAGS    = -DAT32F435Rx -DAT32F435RGT7
LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xg.ld
TARGET_FLASH    := 1024
else ifeq ($(TARGET),$(filter $(TARGET),$(AT32F435M_TARGETS)))
DEVICE_FLAGS    = -DAT32F435Zx -DAT32F435ZMT7
LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xm.ld
TARGET_FLASH    := 4032
else
$(error Unknown MCU for AT32F4 target)
endif

STARTUP_SRC     = at32/startup_at32f435_437.s

ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS   += -DTARGET_FLASH_SIZE=$(TARGET_FLASH) -DAT32F435 -DUSE_ATBSP_DRIVER -DAT32F43x -DAT32 -DUSE_OTG_HOST_MODE

MCU_COMMON_SRC = \
            startup/at32/system_at32f435_437.c \
            startup/at32/at32f435_437_clock.c \
            $(addprefix drivers/at32/,$(notdir $(wildcard $(SRC_DIR)/drivers/at32/*.c))) \
            drivers/inverter.c \
            drivers/bus_i2c_timing.c \
            drivers/bus_i2c_utils.c \
            i2c_application.c

MSC_SRC = \
            msc/usbd_storage.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c

VCP_SRC = \
            $(notdir $(wildcard $(USBCDC_DIR)/*.c)) \
            drivers/usb_io.c

# STM32-only common units; drivers/at32 holds the AT32 counterparts.
MCU_EXCLUDES = \
            drivers/timer.c \
            drivers/serial_uart.c \
            drivers/exti.c \
            drivers/rcc.c

DSP_LIB := $(ROOT)/lib/main/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1U -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4

# The AT32 vendor library and system file have identical branches that GCC flags.
# Suppress -Wduplicated-branches only for those files; EF source keeps the check.
$(OBJECT_DIR)/$(TARGET)/at32f435_437_%.o: CFLAGS += -Wno-duplicated-branches
$(OBJECT_DIR)/$(TARGET)/startup/at32/system_at32f435_437.o: CFLAGS += -Wno-duplicated-branches
