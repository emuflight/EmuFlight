F7X5XG_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_bmi270.c \
            drivers/accgyro/accgyro_spi_bmi270.c \
            drivers/light_ws2811strip.c \
            drivers/max7456.c \
            drivers/compass/compass_fake.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_lis3mdl.c \
            drivers/compass/compass_qmc5883l.c \
