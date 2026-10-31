FEATURES        += VCP SDCARD MSC

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c

TARGET_SRC += \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_msc_desc.c
