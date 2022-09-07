F7X5XG_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH


#            $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
#            $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270.c \
#            drivers/accgyro/bmi270.c \
# BF using max_fifo ; accgyro_spi_bmi270.c specifies max-fifo


TARGET_SRC = \
            $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
            drivers/accgyro/accgyro_spi_bmi270.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_bmp280.c \
			drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c \
            drivers/max7456.c
