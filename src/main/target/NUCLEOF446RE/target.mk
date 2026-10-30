F446_TARGETS  += $(TARGET)
FEATURES    = VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/rx/rx_nrf24l01.c \
            rx/nrf24_cx10.c \
            rx/nrf24_inav.c \
            rx/nrf24_h8_3d.c \
            rx/nrf24_syma.c \
            rx/nrf24_v202.c
