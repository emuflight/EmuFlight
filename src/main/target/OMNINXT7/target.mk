F7X2RE_TARGETS  += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

# XXX Remove fake drivers for final production
TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c
