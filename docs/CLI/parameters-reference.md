# CLI Parameters Reference

> **Auto-generated** — do not edit manually.
> Source: `src/main/interface/settings.c` | Generated: 2026-06-13 | Commit: `47a13a7155` | Firmware: `0.4.3` | MSP: `0.1.54`

---

## Table of Contents

- [Gyro Config](#gyro-config)
- [Accelerometer Config](#accelerometer-config)
- [Compass Config](#compass-config)
- [Barometer Config](#barometer-config)
- [RX Config](#rx-config)
- [RX SPI Config](#rx-spi-config)
- [ADC Config](#adc-config)
- [PWM Config](#pwm-config)
- [Blackbox Config](#blackbox-config)
- [Motor Config](#motor-config)
- [Throttle Correction Config](#throttle-correction-config)
- [Failsafe Config](#failsafe-config)
- [Board Alignment](#board-alignment)
- [Gimbal Config](#gimbal-config)
- [Battery Config](#battery-config)
- [Voltage Sensor ADC Config](#voltage-sensor-adc-config)
- [Current Sensor ADC Config](#current-sensor-adc-config)
- [Current Sensor Virtual Config](#current-sensor-virtual-config)
- [Beeper Dev Config](#beeper-dev-config)
- [Beeper Config](#beeper-config)
- [Mixer Config](#mixer-config)
- [Motor 3D Config](#motor-3d-config)
- [Servo Config](#servo-config)
- [Control Rate Profiles](#control-rate-profiles)
- [Serial Config](#serial-config)
- [IMU Config](#imu-config)
- [Arming Config](#arming-config)
- [GPS Config](#gps-config)
- [GPS Rescue](#gps-rescue)
- [RC Controls Config](#rc-controls-config)
- [PID Config](#pid-config)
- [PID Profile](#pid-profile)
- [Telemetry Config](#telemetry-config)
- [LED Strip Config](#led-strip-config)
- [SDCARD Config](#sdcard-config)
- [SDIO Config](#sdio-config)
- [OSD Config](#osd-config)
- [System Config](#system-config)
- [VTX Settings Config](#vtx-settings-config)
- [VTX Config](#vtx-config)
- [VCD Config](#vcd-config)
- [MAX7456 Config](#max7456-config)
- [Display Port MSP Config](#display-port-msp-config)
- [Display Port MAX7456 Config](#display-port-max7456-config)
- [ESC Sensor Config](#esc-sensor-config)
- [RX Frsky SPI Config](#rx-frsky-spi-config)
- [Status LED Config](#status-led-config)
- [Dashboard Config](#dashboard-config)
- [Camera Control Config](#camera-control-config)
- [Rangefinder Config](#rangefinder-config)
- [Pinio Config](#pinio-config)
- [Piniobox Config](#piniobox-config)
- [USB Config](#usb-config)
- [Flash Config](#flash-config)
- [RCDEVICE Config](#rcdevice-config)

---

## Gyro Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `align_gyro` | uint8 | master | `DEFAULT`, `CW0`, `CW90`, `CW180`, `CW270`, `CW0FLIP`, `CW90FLIP`, `CW180FLIP`, `CW270FLIP`, `CW45`, `CW135`, `CW225`, `CW315`, `CW45FLIP`, `CW135FLIP`, `CW225FLIP`, `CW315FLIP` |
| `gyro_hardware_lpf` | uint8 | master | `NORMAL`, `OPTION1`, `OPTION2`, `EXPERIMENTAL`, `EXPERIMENTAL`, `1KHZ_SAMPLING` |
| `gyro_32khz_hardware_lpf` | uint8 | master | `NORMAL`, `EXPERIMENTAL` |
| `gyro_high_range` | uint8 | master | `OFF`, `ON` |
| `gyro_sync_denom` | uint8 | master | `1` – `32` |
| `gyro_lowpass_type` | uint8 | master | `PT1`, `SVF`, `PT2`, `PT3`, `PT4` |
| `gyro_lowpass_hz_roll` | uint16 | master | `0` – `16000` |
| `gyro_lowpass_hz_pitch` | uint16 | master | `0` – `16000` |
| `gyro_lowpass_hz_yaw` | uint16 | master | `0` – `16000` |
| `gyro_lowpass2_type` | uint8 | master | `PT1`, `SVF`, `PT2`, `PT3`, `PT4` |
| `gyro_lowpass2_hz_roll` | uint16 | master | `0` – `16000` |
| `gyro_lowpass2_hz_pitch` | uint16 | master | `0` – `16000` |
| `gyro_lowpass2_hz_yaw` | uint16 | master | `0` – `16000` |
| `gyro_abg_alpha` | uint16 | master | `0` – `1000` |
| `gyro_abg_boost` | uint16 | master | `0` – `2000` |
| `gyro_abg_half_life` | uint8 | master | `0` – `250` |
| `gyro_notch1_hz` | uint16 | master | `0` – `16000` |
| `gyro_notch1_cutoff` | uint16 | master | `0` – `16000` |
| `gyro_notch2_hz` | uint16 | master | `0` – `16000` |
| `gyro_notch2_cutoff` | uint16 | master | `0` – `16000` |
| `gyro_calib_duration` | uint16 | master | `50` – `3000` |
| `gyro_calib_noise_limit` | uint8 | master | `0` – `200` |
| `gyro_offset_yaw` | int16 | master | `-1000` – `1000` |
| `imuf_rate` | uint8 | master | `32K`, `16K`, `8K`, `4K`, `2K`, `1K` |
| `imuf_roll_q` | uint16 | master | `100` – `16000` |
| `imuf_pitch_q` | uint16 | master | `100` – `16000` |
| `imuf_yaw_q` | uint16 | master | `100` – `16000` |
| `imuf_w` | uint16 | master | `3` – `512` |
| `imuf_pitch_lpf_cutoff_hz` | uint16 | master | `0` – `450` |
| `imuf_roll_lpf_cutoff_hz` | uint16 | master | `0` – `450` |
| `imuf_yaw_lpf_cutoff_hz` | uint16 | master | `0` – `450` |
| `imuf_acc_lpf_cutoff_hz` | uint16 | master | `30` – `180` |
| `imuf_ptn_order` | uint8 | master | `1` – `4` |
| `imuf_roll_q` | uint16 | master | `0` – `16000` |
| `imuf_pitch_q` | uint16 | master | `0` – `16000` |
| `imuf_yaw_q` | uint16 | master | `0` – `16000` |
| `imuf_w` | uint16 | master | `0` – `512` |
| `gyro_overflow_detect` | uint8 | master | `OFF`, `YAW`, `ALL` |
| `yaw_spin_recovery` | uint8 | master | `OFF`, `ON`, `AUTO` |
| `yaw_spin_threshold` | uint16 | master | `YAW_SPIN_RECOVERY_THRESHOLD_MIN` – `YAW_SPIN_RECOVERY_THRESHOLD_MAX` |
| `gyro_use_32khz` | uint8 | master | `OFF`, `ON` |
| `gyro_to_use` | uint8 | master | `FIRST`, `SECOND`, `BOTH` |
| `dynamic_gyro_notch_axis` | uint8 | master | `RP`, `RPY` |
| `dynamic_gyro_notch_q` | uint16 | master | `1` – `1000` |
| `dynamic_gyro_notch_count` | uint8 | master | `1` – `5` |
| `dynamic_gyro_notch_min_hz` | uint16 | master | `30` – `1000` |
| `dynamic_gyro_notch_max_hz` | uint16 | master | `400` – `1000` |
| `smith_predict_enabled` | uint8 | master | `OFF`, `ON` |
| `smith_predict_str` | uint8 | master | `0` – `100` |
| `smith_predict_delay` | uint8 | master | `0` – `120` |
| `smith_predict_filt_hz` | uint8 | master | `1` – `250` |

## Accelerometer Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `align_acc` | uint8 | master | `DEFAULT`, `CW0`, `CW90`, `CW180`, `CW270`, `CW0FLIP`, `CW90FLIP`, `CW180FLIP`, `CW270FLIP`, `CW45`, `CW135`, `CW225`, `CW315`, `CW45FLIP`, `CW135FLIP`, `CW225FLIP`, `CW315FLIP` |
| `acc_hardware` | uint8 | master | `AUTO`, `NONE`, `ADXL345`, `MPU6050`, `MMA8452`, `BMA280`, `LSM303DLHC`, `MPU6000`, `MPU6500`, `MPU9250`, `ICM20601`, `ICM20602`, `ICM20608G`, `ICM20649`, `ICM20689`, `ICM42605`, `ICM42688P`, `BMI160`, `BMI270`, `ACC_IMUF9001`, `FAKE` |
| `acc_high_range` | uint8 | master | `OFF`, `ON` |
| `acc_lpf_hz` | uint16 | master | `0` – `400` |
| `acc_trim_pitch` | int16 | master | `-300` – `300` |
| `acc_trim_roll` | int16 | master | `-300` – `300` |
| `acc_calibration` | int16 | master | array\[XYZ_AXIS_COUNT\] |

## Compass Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `align_mag` | uint8 | master | `DEFAULT`, `CW0`, `CW90`, `CW180`, `CW270`, `CW0FLIP`, `CW90FLIP`, `CW180FLIP`, `CW270FLIP`, `CW45`, `CW135`, `CW225`, `CW315`, `CW45FLIP`, `CW135FLIP`, `CW225FLIP`, `CW315FLIP` |
| `mag_bustype` | uint8 | master | `NONE`, `I2C`, `SPI`, `SLAVE` |
| `mag_i2c_device` | uint8 | master | `0` – `I2CDEV_COUNT` |
| `mag_i2c_address` | uint8 | master | `0` – `I2C_ADDR7_MAX` |
| `mag_spi_device` | uint8 | master | `0` – `SPIDEV_COUNT` |
| `mag_hardware` | uint8 | master | `AUTO`, `NONE`, `HMC5883`, `AK8975`, `AK8963`, `QMC5883` |
| `mag_declination` | int16 | master | `-18000` – `18000` |
| `mag_calibration` | int16 | master | array\[XYZ_AXIS_COUNT\] |

## Barometer Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `baro_bustype` | uint8 | master | `NONE`, `I2C`, `SPI`, `SLAVE` |
| `baro_spi_device` | uint8 | master | `0` – `5` |
| `baro_i2c_device` | uint8 | master | `0` – `5` |
| `baro_i2c_address` | uint8 | master | `0` – `I2C_ADDR7_MAX` |
| `baro_hardware` | uint8 | master | `AUTO`, `NONE`, `BMP085`, `MS5611`, `BMP280`, `LPS`, `QMP6988` |
| `baro_tab_size` | uint8 | master | `1` – `BARO_SAMPLE_COUNT_MAX` |
| `baro_noise_lpf` | uint16 | master | `0` – `1000` |
| `baro_cf_vel` | uint16 | master | `0` – `1000` |
| `baro_cf_alt` | uint16 | master | `0` – `1000` |

## RX Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `mid_rc` | uint16 | master | `1200` – `1700` |
| `min_check` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `max_check` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `rssi_channel` | int8 | master | `0` – `MAX_SUPPORTED_RC_CHANNEL_COUNT` |
| `rssi_src_frame_errors` | int8 | master | `OFF`, `ON` |
| `rssi_scale` | uint8 | master | `RSSI_SCALE_MIN` – `RSSI_SCALE_MAX` |
| `rssi_offset` | int8 | master | `-100` – `100` |
| `rssi_invert` | int8 | master | `OFF`, `ON` |
| `rc_interp` | uint8 | master | `OFF`, `PRESET`, `AUTO`, `MANUAL` |
| `rc_interp_ch` | uint8 | master | `RP`, `RPY`, `RPYT`, `T`, `RPT` |
| `rc_interp_int` | uint8 | master | `1` – `50` |
| `rc_smoothing_type` | uint8 | master | `INTERPOLATION`, `FILTER` |
| `rc_smoothing_input_hz` | uint8 | master | `0` – `UINT8_MAX` |
| `rc_smoothing_debug_axis` | uint8 | master | `ROLL`, `PITCH`, `YAW`, `THROTTLE` |
| `rc_smoothing_input_type` | uint8 | master | `PT1`, `BIQUAD`, `PT2`, `PT3`, `PT4` |
| `fpv_mix_degrees` | uint8 | master | `0` – `90` |
| `cinematic_yaw` | uint8 | master | `OFF`, `ON` |
| `max_aux_channels` | uint8 | master | `0` – `MAX_AUX_CHANNEL_COUNT` |
| `serialrx_provider` | uint8 | master | `SPEK1024`, `SPEK2048`, `SBUS`, `SUMD`, `SUMH`, `XB-B`, `XB-B-RJ01`, `IBUS`, `JETIEXBUS`, `CRSF`, `SRXL`, `CUSTOM`, `FPORT`, `GHST` |
| `serialrx_inverted` | uint8 | master | `OFF`, `ON` |
| `spektrum_sat_bind` | uint8 | master | `SPEKTRUM_SAT_BIND_DISABLED` – `SPEKTRUM_SAT_BIND_MAX` |
| `spektrum_sat_bind_autoreset` | uint8 | master | `OFF`, `ON` |
| `sbus_baud_fast` | uint8 | master | `OFF`, `ON` |
| `airmode_start_throttle_percent` | uint8 | master | `0` – `100` |
| `rx_min_usec` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `rx_max_usec` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `serialrx_halfduplex` | uint8 | master | `OFF`, `ON` |
| `show_altered_rc` | uint8 | master | `OFF`, `ON` |

## RX SPI Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `rx_spi_protocol` | uint8 | master | `V202_250K`, `V202_1M`, `SYMA_X`, `SYMA_X5C`, `CX10`, `CX10A`, `H8_3D`, `INAV`, `FRSKY_D`, `FRSKY_X`, `FLYSKY`, `FLYSKY_2A`, `KN`, `SFHSS`, `REDPINE` |
| `rx_spi_bus` | uint8 | master | `0` – `SPIDEV_COUNT` |

## ADC Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `adc_device` | int8 | master | `0` – `ADCDEV_COUNT` |

## PWM Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `input_filtering_mode` | int8 | master | `OFF`, `ON` |

## Blackbox Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `blackbox_p_ratio` | uint16 | master | `0` – `INT16_MAX` |
| `blackbox_device` | uint8 | master | `NONE`, `SPIFLASH`, `SDCARD`, `SERIAL` |
| `blackbox_record_acc` | uint8 | master | `OFF`, `ON` |
| `blackbox_mode` | uint8 | master | `NORMAL`, `MOTOR_TEST`, `ALWAYS` |

## Motor Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `min_throttle` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `max_throttle` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `min_command` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `dshot_idle_value` | uint16 | master | `0` – `2000` |
| `dshot_burst` | uint8 | master | `OFF`, `ON` |
| `use_unsynced_pwm` | uint8 | master | `OFF`, `ON` |
| `motor_pwm_protocol` | uint8 | master | `OFF`, `ONESHOT125`, `ONESHOT42`, `MULTISHOT`, `BRUSHED`, `DSHOT150`, `DSHOT300`, `DSHOT600`, `DSHOT1200`, `DSHOT2400`, `DSHOT4800`, `PROSHOT1000` |
| `motor_pwm_rate` | uint16 | master | `200` – `32000` |
| `motor_pwm_inversion` | uint8 | master | `OFF`, `ON` |
| `motor_poles` | uint8 | master | `4` – `UINT8_MAX` |

## Throttle Correction Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `thr_corr_value` | uint8 | master | `0` – `150` |
| `thr_corr_angle` | uint16 | master | `1` – `900` |

## Failsafe Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `failsafe_delay` | uint8 | master | `0` – `200` |
| `failsafe_off_delay` | uint8 | master | `0` – `200` |
| `failsafe_throttle` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `failsafe_switch_mode` | uint8 | master | `STAGE1`, `KILL`, `STAGE2` |
| `failsafe_throttle_low_delay` | uint16 | master | `0` – `300` |
| `failsafe_procedure` | uint8 | master | `AUTO-LAND`, `DROP`, `GPS-RESCUE` |

## Board Alignment

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `align_board_roll` | int16 | master | `-180` – `360` |
| `align_board_pitch` | int16 | master | `-180` – `360` |
| `align_board_yaw` | int16 | master | `-180` – `360` |

## Gimbal Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `gimbal_mode` | uint8 | master | `NORMAL`, `MIXTILT` |

## Battery Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `bat_capacity` | uint16 | master | `0` – `20000` |
| `vbat_max_cell_voltage` | uint8 | master | `VBAT_CELL_VOTAGE_RANGE_MIN` – `VBAT_CELL_VOTAGE_RANGE_MAX` |
| `vbat_full_cell_voltage` | uint8 | master | `VBAT_CELL_VOTAGE_RANGE_MIN` – `VBAT_CELL_VOTAGE_RANGE_MAX` |
| `vbat_min_cell_voltage` | uint8 | master | `VBAT_CELL_VOTAGE_RANGE_MIN` – `VBAT_CELL_VOTAGE_RANGE_MAX` |
| `vbat_warning_cell_voltage` | uint8 | master | `VBAT_CELL_VOTAGE_RANGE_MIN` – `VBAT_CELL_VOTAGE_RANGE_MAX` |
| `vbat_hysteresis` | uint8 | master | `0` – `250` |
| `current_meter` | uint8 | master | `NONE`, `ADC`, `VIRTUAL`, `ESC`, `MSP` |
| `battery_meter` | uint8 | master | `NONE`, `ADC`, `ESC` |
| `vbat_detect_cell_voltage` | uint8 | master | `VBAT_CELL_VOTAGE_RANGE_MIN` – `VBAT_CELL_VOTAGE_RANGE_MAX` |
| `use_vbat_alerts` | uint8 | master | `OFF`, `ON` |
| `use_cbat_alerts` | uint8 | master | `OFF`, `ON` |
| `cbat_alert_percent` | uint8 | master | `0` – `100` |
| `vbat_cutoff_percent` | uint8 | master | `0` – `100` |
| `vbat_lpf_period` | uint8 | master | `0` – `UINT8_MAX` |
| `ibat_lpf_period` | uint8 | master | `0` – `UINT8_MAX` |
| `force_battery_cell_count` | uint8 | master | `0` – `24` |

## Voltage Sensor ADC Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `vbat_scale` | uint8 | master | `VBAT_SCALE_MIN` – `VBAT_SCALE_MAX` |
| `vbat_divider` | uint8 | master | `VBAT_DIVIDER_MIN` – `VBAT_DIVIDER_MAX` |
| `vbat_multiplier` | uint8 | master | `VBAT_MULTIPLIER_MIN` – `VBAT_MULTIPLIER_MAX` |

## Current Sensor ADC Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `ibata_scale` | int16 | master | `-16000` – `16000` |
| `ibata_offset` | int16 | master | `-32000` – `32000` |

## Current Sensor Virtual Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `ibatv_scale` | int16 | master | `-16000` – `16000` |
| `ibatv_offset` | uint16 | master | `0` – `16000` |

## Beeper Dev Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `beeper_inversion` | uint8 | master | `OFF`, `ON` |
| `beeper_od` | uint8 | master | `OFF`, `ON` |
| `beeper_frequency` | int16 | master | `0` – `16000` |

## Beeper Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `beeper_dshot_beacon_tone` | uint8 | master | `1` – `DSHOT_CMD_BEACON5` |

## Mixer Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `yaw_motors_reversed` | int8 | master | `OFF`, `ON` |
| `crashflip_motor_percent` | uint8 | master | `0` – `100` |
| `crashflip_power_percent` | uint8 | master | `25` – `100` |

## Motor 3D Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `3d_deadband_low` | uint16 | master | `PWM_PULSE_MIN` – `PWM_RANGE_MIDDLE` |
| `3d_deadband_high` | uint16 | master | `PWM_RANGE_MIDDLE` – `PWM_PULSE_MAX` |
| `3d_neutral` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `3d_deadband_throttle` | uint16 | master | `1` – `100` |
| `3d_limit_low` | uint16 | master | `PWM_PULSE_MIN` – `PWM_RANGE_MIDDLE` |
| `3d_limit_high` | uint16 | master | `PWM_RANGE_MIDDLE` – `PWM_PULSE_MAX` |
| `3d_switched_mode` | uint8 | master | `OFF`, `ON` |

## Servo Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `servo_center_pulse` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `servo_pwm_rate` | uint16 | master | `50` – `498` |
| `servo_lowpass_hz` | uint16 | master | `0` – `400` |
| `tri_unarmed_servo` | int8 | master | `OFF`, `ON` |
| `channel_forwarding_start` | uint8 | master | `AUX1` – `MAX_SUPPORTED_RC_CHANNEL_COUNT` |

## Control Rate Profiles

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `thr_mid` | uint8 | rate | `0` – `100` |
| `thr_expo` | uint8 | rate | `0` – `100` |
| `rates_type` | uint8 | rate | `BETAFLIGHT`, `RACEFLIGHT`, `KISS`, `ACTUAL` |
| `roll_rc_rate` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RC_RATES_MAX` |
| `pitch_rc_rate` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RC_RATES_MAX` |
| `yaw_rc_rate` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RC_RATES_MAX` |
| `roll_expo` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RC_EXPO_MAX` |
| `pitch_expo` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RC_EXPO_MAX` |
| `yaw_expo` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RC_EXPO_MAX` |
| `roll_srate` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RATE_MAX` |
| `pitch_srate` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RATE_MAX` |
| `yaw_srate` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RATE_MAX` |
| `das_yaw_with_roll_input` | uint8 | rate | `0` – `100` |
| `das_roll_with_yaw_input` | uint8 | rate | `0` – `100` |
| `roll_pitch_mag_expo` | uint8 | rate | `0` – `250` |
| `rate_center_sensitivity` | uint8 | rate | `25` – `175` |
| `rate_end_sensitivity` | uint8 | rate | `25` – `175` |
| `rate_center_correction` | uint8 | rate | `10` – `95` |
| `rate_end_correction` | uint8 | rate | `10` – `95` |
| `rate_center_weight` | uint8 | rate | `0` – `95` |
| `rate_end_weight` | uint8 | rate | `0` – `95` |
| `tpa_rate_p` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_TPA_MAX` |
| `tpa_rate_i` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_TPA_MAX` |
| `tpa_rate_d` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_TPA_MAX` |
| `tpa_breakpoint` | uint16 | rate | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `throttle_limit_type` | uint8 | rate | `OFF`, `SCALE`, `CLIP` |
| `throttle_limit_percent` | uint8 | rate | `25` – `100` |
| `vbat_comp_type` | uint8 | rate | `OFF`, `BOOST`, `LIMIT`, `BOTH` |
| `vbat_comp_ref` | uint8 | rate | `VBAT_CELL_VOTAGE_RANGE_MIN` – `VBAT_CELL_VOTAGE_RANGE_MAX` |

## Serial Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `reboot_character` | uint8 | master | `48` – `126` |
| `serial_update_rate_hz` | uint16 | master | `100` – `2000` |

## IMU Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `accxy_deadband` | uint8 | master | `0` – `100` |
| `accz_deadband` | uint8 | master | `0` – `100` |
| `acc_unarmedcal` | uint8 | master | `OFF`, `ON` |
| `imu_dcm_kp` | uint16 | master | `0` – `32000` |
| `imu_dcm_ki` | uint16 | master | `0` – `32000` |
| `small_angle` | uint8 | master | `0` – `180` |

## Arming Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `auto_disarm_delay` | uint8 | master | `0` – `60` |
| `gyro_cal_on_first_arm` | uint8 | master | `OFF`, `ON` |
| `prearm_allow_rearm` | uint8 | master | `OFF`, `ON` |
| `use_stick_arming` | int8 | master | `OFF`, `ON` |

## GPS Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `gps_provider` | uint8 | master | `NMEA`, `UBLOX` |
| `gps_sbas_mode` | uint8 | master | `AUTO`, `EGNOS`, `WAAS`, `MSAS`, `GAGAN` |
| `gps_auto_config` | uint8 | master | `OFF`, `ON` |
| `gps_auto_baud` | uint8 | master | `OFF`, `ON` |
| `gps_ublox_use_galileo` | uint8 | master | `OFF`, `ON` |

## GPS Rescue

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `gps_rescue_angle` | uint16 | master | `0` – `200` |
| `gps_rescue_initial_alt` | uint16 | master | `20` – `100` |
| `gps_rescue_descent_dist` | uint16 | master | `30` – `500` |
| `gps_rescue_ground_speed` | uint16 | master | `30` – `3000` |
| `gps_rescue_throttle_p` | uint16 | master | `0` – `500` |
| `gps_rescue_throttle_i` | uint16 | master | `0` – `500` |
| `gps_rescue_throttle_d` | uint16 | master | `0` – `500` |
| `gps_rescue_velocity_p` | uint16 | master | `0` – `500` |
| `gps_rescue_velocity_i` | uint16 | master | `0` – `500` |
| `gps_rescue_velocity_d` | uint16 | master | `0` – `500` |
| `gps_rescue_yaw_p` | uint16 | master | `0` – `500` |
| `gps_rescue_throttle_min` | uint16 | master | `1000` – `2000` |
| `gps_rescue_throttle_max` | uint16 | master | `1000` – `2000` |
| `gps_rescue_throttle_hover` | uint16 | master | `1000` – `2000` |
| `gps_rescue_sanity_checks` | uint8 | master | `RESCUE_SANITY_OFF`, `RESCUE_SANITY_ON`, `RESCUE_SANITY_FS_ONLY` |
| `gps_rescue_min_sats` | uint8 | master | `0` – `50` |

## RC Controls Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `deadband` | uint8 | master | `0` – `32` |
| `yaw_deadband` | uint8 | master | `0` – `100` |
| `yaw_control_reversed` | int8 | master | `OFF`, `ON` |

## PID Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `pid_process_denom` | uint8 | master | `1` – `MAX_PID_PROCESS_DENOM` |
| `runaway_takeoff_prevention` | uint8 | master | `OFF`, `ON` |

## PID Profile

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `dterm_dyn_notch_q` | uint16 | profile | `1` – `1000` |
| `dterm_abg_alpha` | uint16 | profile | `0` – `1000` |
| `dterm_abg_boost` | uint16 | profile | `0` – `2000` |
| `dterm_abg_half_life` | uint8 | profile | `0` – `250` |
| `dterm_lowpass_type` | uint8 | profile | `PT1`, `SVF`, `PT2`, `PT3`, `PT4` |
| `dterm_lowpass_hz_roll` | uint16 | profile | `0` – `16000` |
| `dterm_lowpass_hz_pitch` | uint16 | profile | `0` – `16000` |
| `dterm_lowpass_hz_yaw` | uint16 | profile | `0` – `16000` |
| `dterm_lowpass2_type` | uint8 | profile | `PT1`, `SVF`, `PT2`, `PT3`, `PT4` |
| `dterm_lowpass2_hz_roll` | uint16 | profile | `0` – `16000` |
| `dterm_lowpass2_hz_pitch` | uint16 | profile | `0` – `16000` |
| `dterm_lowpass2_hz_yaw` | uint16 | profile | `0` – `16000` |
| `pid_at_min_throttle` | uint8 | profile | `OFF`, `ON` |
| `spa_roll_p` | uint8 | profile | `0` – `250` |
| `spa_roll_i` | uint8 | profile | `0` – `250` |
| `spa_roll_d` | uint8 | profile | `0` – `250` |
| `spa_pitch_p` | uint8 | profile | `0` – `250` |
| `spa_pitch_i` | uint8 | profile | `0` – `250` |
| `spa_pitch_d` | uint8 | profile | `0` – `250` |
| `spa_yaw_p` | uint8 | profile | `0` – `250` |
| `spa_yaw_i` | uint8 | profile | `0` – `250` |
| `spa_yaw_d` | uint8 | profile | `0` – `250` |
| `acc_limit_yaw` | uint16 | profile | `0` – `500` |
| `acc_limit` | uint16 | profile | `0` – `500` |
| `crash_dthreshold` | uint16 | profile | `0` – `2000` |
| `crash_gthreshold` | uint16 | profile | `0` – `2000` |
| `crash_setpoint_threshold` | uint16 | profile | `0` – `2000` |
| `crash_time` | uint16 | profile | `0` – `5000` |
| `crash_delay` | uint16 | profile | `0` – `500` |
| `crash_recovery_angle` | uint8 | profile | `0` – `30` |
| `crash_recovery_rate` | uint8 | profile | `0` – `255` |
| `crash_limit_yaw` | uint16 | profile | `0` – `1000` |
| `crash_recovery` | uint8 | profile | `OFF`, `ON`, `BEEP` |
| `iterm_rotation` | uint8 | profile | `OFF`, `ON` |
| `iterm_relax_cutoff` | uint8 | profile | `0` – `100` |
| `iterm_relax_cutoff_yaw` | uint8 | profile | `0` – `100` |
| `iterm_relax_threshold` | uint8 | profile | `10` – `100` |
| `iterm_relax_threshold_yaw` | uint8 | profile | `10` – `100` |
| `iterm_windup` | uint8 | profile | `0` – `100` |
| `iterm_limit` | uint16 | profile | `0` – `500` |
| `pidsum_limit` | uint16 | profile | `PIDSUM_LIMIT_MIN` – `PIDSUM_LIMIT_MAX` |
| `pidsum_limit_yaw` | uint16 | profile | `PIDSUM_LIMIT_MIN` – `PIDSUM_LIMIT_MAX` |
| `throttle_boost` | uint8 | profile | `0` – `100` |
| `throttle_boost_cutoff` | uint8 | profile | `5` – `50` |
| `feathered_pids` | uint8 | profile | `0` – `100` |
| `i_decay` | uint8 | profile | `1` – `10` |
| `i_decay_cutoff` | uint8 | profile | `1` – `250` |
| `emu_boost` | uint16 | profile | `0` – `2000` |
| `emu_boost_yaw` | uint16 | profile | `0` – `2000` |
| `emu_boost_limit` | uint8 | profile | `0` – `250` |
| `emu_boost_limit_yaw` | uint8 | profile | `0` – `250` |
| `dterm_boost` | uint16 | profile | `0` – `2000` |
| `dterm_boost_limit` | uint8 | profile | `0` – `250` |
| `emu_gravity` | uint8 | profile | `0` – `255` |
| `axis_lock_multiplier` | uint8 | profile | `0` – `10` |
| `axis_lock_hz` | uint8 | profile | `1` – `50` |
| `p_pitch` | uint8 | profile | `0` – `200` |
| `i_pitch` | uint8 | profile | `0` – `200` |
| `d_pitch` | uint8 | profile | `0` – `200` |
| `p_roll` | uint8 | profile | `0` – `200` |
| `i_roll` | uint8 | profile | `0` – `200` |
| `d_roll` | uint8 | profile | `0` – `200` |
| `p_yaw` | uint8 | profile | `0` – `200` |
| `i_yaw` | uint8 | profile | `0` – `200` |
| `d_yaw` | uint8 | profile | `0` – `200` |
| `df_yaw` | uint8 | profile | `0` – `200` |
| `p_angle_low` | uint8 | profile | `0` – `200` |
| `d_angle_low` | uint8 | profile | `0` – `200` |
| `p_angle_high` | uint8 | profile | `0` – `200` |
| `d_angle_high` | uint8 | profile | `0` – `200` |
| `f_angle` | uint16 | profile | `0` – `2000` |
| `df_angle_low` | uint8 | profile | `0` – `200` |
| `df_angle_high` | uint8 | profile | `0` – `200` |
| `level_limit` | uint8 | profile | `10` – `90` |
| `angle_expo` | uint8 | profile | `0` – `100` |
| `angle_filter` | uint8 | profile | `0` – `255` |
| `horizon_transition` | uint8 | profile | `0` – `200` |
| `horizon_tilt_effect` | uint8 | profile | `0` – `180` |
| `horizon_strength` | uint8 | profile | `0` – `200` |
| `motor_output_limit` | uint8 | profile | `MOTOR_OUTPUT_LIMIT_PERCENT_MIN` – `MOTOR_OUTPUT_LIMIT_PERCENT_MAX` |
| `auto_profile_cell_count` | int8 | profile | `AUTO_PROFILE_CELL_COUNT_CHANGE` – `MAX_AUTO_DETECT_CELL_COUNT` |
| `linear_thrust_low_output` | uint8 | profile | `0` – `100` |
| `linear_thrust_high_output` | uint8 | profile | `0` – `100` |
| `linear_throttle` | uint8 | profile | `OFF`, `ON` |
| `mixer_impl` | uint8 | profile | `LEGACY`, `SMOOTH`, `2PASS` |
| `mixer_laziness` | uint8 | profile | `OFF`, `ON` |
| `mixer_yaw_throttle_comp` | uint8 | profile | `OFF`, `ON` |

## Telemetry Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `tlm_inverted` | uint8 | master | `OFF`, `ON` |
| `tlm_halfduplex` | uint8 | master | `OFF`, `ON` |
| `frsky_default_lat` | int16 | master | `-9000` – `9000` |
| `frsky_default_long` | int16 | master | `-18000` – `18000` |
| `frsky_gps_format` | uint8 | master | `0` – `FRSKY_FORMAT_NMEA` |
| `frsky_unit` | uint8 | master | `IMPERIAL`, `METRIC` |
| `frsky_vfas_precision` | uint8 | master | `FRSKY_VFAS_PRECISION_LOW` – `FRSKY_VFAS_PRECISION_HIGH` |
| `hott_alarm_int` | uint8 | master | `0` – `120` |
| `pid_in_tlm` | uint8 | master | `OFF`, `ON` |
| `report_cell_voltage` | uint8 | master | `OFF`, `ON` |
| `ibus_sensor` | uint8 | master | array\[IBUS_SENSOR_COUNT\] |
| `smartport_use_extra_sensors` | uint8 | master | `OFF`, `ON` |

## LED Strip Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `ledstrip_visual_beeper` | uint8 | master | `OFF`, `ON` |
| `ledstrip_grb_rgb` | uint8 | master | `GRB`, `RGB` |

## SDCARD Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `sdcard_dma` | uint8 | master | `OFF`, `ON` |

## SDIO Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `sdio_clk_bypass` | uint8 | master | `OFF`, `ON` |
| `sdio_use_cache` | uint8 | master | `OFF`, `ON` |

## OSD Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `osd_units` | uint8 | master | `IMPERIAL`, `METRIC` |
| `osd_warn_arming_disable` | uint16 | master | bitflag |
| `osd_warn_batt_not_full` | uint16 | master | bitflag |
| `osd_warn_batt_warning` | uint16 | master | bitflag |
| `osd_warn_batt_critical` | uint16 | master | bitflag |
| `osd_warn_visual_beeper` | uint16 | master | bitflag |
| `osd_warn_crash_flip` | uint16 | master | bitflag |
| `osd_warn_esc_fail` | uint16 | master | bitflag |
| `osd_task_frequency` | uint16 | master | `OSD_TASK_FREQUENCY_MIN` – `OSD_TASK_FREQUENCY_MAX` |
| `osd_warn_core_temp` | uint16 | master | bitflag |
| `osd_warn_rc_smoothing` | uint16 | master | bitflag |
| `osd_warn_dji` | uint16 | master | bitflag |
| `osd_lq_format` | uint8 | master | `SCALED`, `MODE`, `FREQ`, `SIMPLE`, `TBS` |
| `osd_lq_alarm` | uint16 | master | `0` – `300` |
| `osd_rssi_alarm` | uint8 | master | `0` – `100` |
| `osd_cap_alarm` | uint16 | master | `0` – `20000` |
| `osd_alt_alarm` | uint16 | master | `0` – `10000` |
| `osd_distance_alarm` | uint16 | master | `0` – `10000` |
| `osd_esc_temp_alarm` | int8 | master | `INT8_MIN` – `INT8_MAX` |
| `osd_esc_rpm_alarm` | int16 | master | `ESC_RPM_ALARM_OFF` – `INT16_MAX` |
| `osd_esc_current_alarm` | int16 | master | `ESC_CURRENT_ALARM_OFF` – `INT16_MAX` |
| `osd_core_temp_alarm` | uint8 | master | `0` – `UINT8_MAX` |
| `osd_stat_show_cell_value` | uint8 | master | `OFF`, `ON` |
| `osd_ah_max_pit` | uint8 | master | `0` – `90` |
| `osd_ah_max_rol` | uint8 | master | `0` – `90` |
| `osd_logo_on_arming` | uint8 | master | `OFF`, `ON`, `FIRST_ARMING` |
| `osd_logo_on_arming_duration` | uint8 | master | `5` – `50` |
| `osd_tim1` | uint16 | master | `0` – `INT16_MAX` |
| `osd_tim2` | uint16 | master | `0` – `INT16_MAX` |
| `osd_vbat_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_rssi_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_crsf_tx_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_crsf_snr_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_crsf_rssi_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_tim_1_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_tim_2_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_remaining_time_estimate_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_flymode_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_g_force_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_throttle_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_vtx_channel_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_crosshairs_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_ah_sbar_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_ah_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_current_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_mah_drawn_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_craft_name_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_gps_speed_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_gps_lon_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_gps_lat_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_gps_sats_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_plus_code_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `plus_code_digits` | uint8 | master | `10` – `13` |
| `plus_code_short` | uint8 | master | `OFF`, `ON` |
| `osd_home_dir_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_home_dist_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_compass_bar_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_altitude_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_pid_roll_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_pid_pitch_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_pid_yaw_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_debug_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_power_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_pidrate_profile_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_warnings_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_avg_cell_voltage_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_pit_ang_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_rol_ang_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_battery_usage_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_disarmed_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_nheading_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_nvario_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_esc_tmp_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_esc_rpm_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_rtc_date_time_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_adjustment_range_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_mah_percent_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_core_temp_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` |
| `osd_stat_rtc_date_time` | uint32 | master | bitflag |
| `osd_stat_tim_1` | uint32 | master | bitflag |
| `osd_stat_tim_2` | uint32 | master | bitflag |
| `osd_stat_max_spd` | uint32 | master | bitflag |
| `osd_stat_max_dist` | uint32 | master | bitflag |
| `osd_stat_min_batt` | uint32 | master | bitflag |
| `osd_stat_endbatt` | uint32 | master | bitflag |
| `osd_stat_battery` | uint32 | master | bitflag |
| `osd_stat_min_rssi` | uint32 | master | bitflag |
| `osd_stat_max_curr` | uint32 | master | bitflag |
| `osd_stat_used_mah` | uint32 | master | bitflag |
| `osd_stat_max_alt` | uint32 | master | bitflag |
| `osd_stat_bbox` | uint32 | master | bitflag |
| `osd_stat_bb_no` | uint32 | master | bitflag |

## System Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `task_statistics` | int8 | master | `OFF`, `ON` |
| `debug_mode` | uint8 | master | `NONE`, `CYCLETIME`, `BATTERY`, `GYRO_FILTERED`, `ACCELEROMETER`, `PIDLOOP`, `GYRO_SCALED`, `RC_INTERPOLATION`, `ANGLERATE`, `ESC_SENSOR`, `SCHEDULER`, `STACK`, `ESC_SENSOR_RPM`, `ESC_SENSOR_TMP`, `ALTITUDE`, `FFT`, `FFT_TIME`, `FFT_FREQ`, `RX_FRSKY_SPI`, `RX_SFHSS_SPI`, `GYRO_RAW`, `DUAL_GYRO`, `DUAL_GYRO_RAW`, `DUAL_GYRO_COMBINE`, `DUAL_GYRO_DIFF`, `MAX7456_SIGNAL`, `MAX7456_SPICLOCK`, `SBUS`, `FPORT`, `RANGEFINDER`, `RANGEFINDER_QUALITY`, `LIDAR_TF`, `CORE_TEMP`, `RUNAWAY_TAKEOFF`, `SDIO`, `CURRENT_SENSOR`, `USB`, `SMARTAUDIO`, `RTH`, `ITERM_RELAX`, `RC_SMOOTHING`, `RX_SIGNAL_LOSS`, `RC_SMOOTHING_RATE`, `IMU`, `KALMAN`, `ANGLE`, `HORIZON` |
| `rate_6pos_switch` | int8 | master | `OFF`, `ON` |
| `cpu_overclock` | uint8 | master | `OFF`, `192MHZ`, `216MHZ`, `240MHZ`, `108MHZ`, `120MHZ`, `240MHZ` |
| `pwr_on_arm_grace` | uint8 | master | `0` – `30` |

## VTX Settings Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `vtx_band` | uint8 | master | `VTX_SETTINGS_NO_BAND` – `VTX_SETTINGS_MAX_BAND` |
| `vtx_channel` | uint8 | master | `VTX_SETTINGS_MIN_CHANNEL` – `VTX_SETTINGS_MAX_CHANNEL` |
| `vtx_power` | uint8 | master | `VTX_SETTINGS_MIN_POWER` – `VTX_SETTINGS_POWER_COUNT - 1` |
| `vtx_low_power_disarm` | uint8 | master | `OFF`, `ON` |
| `vtx_freq` | uint16 | master | `0` – `VTX_SETTINGS_MAX_FREQUENCY_MHZ` |
| `vtx_pit_mode_freq` | uint16 | master | `0` – `VTX_SETTINGS_MAX_FREQUENCY_MHZ` |

## VTX Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `vtx_halfduplex` | uint8 | master | `OFF`, `ON` |

## VCD Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `vcd_video_system` | uint8 | master | `AUTO`, `PAL`, `NTSC`, `HD` |
| `vcd_h_offset` | int8 | master | `-32` – `31` |
| `vcd_v_offset` | int8 | master | `-15` – `16` |

## MAX7456 Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `max7456_clock` | uint8 | master | `HALF`, `DEFAULT`, `FULL` |
| `max7456_spi_bus` | uint8 | master | `0` – `SPIDEV_COUNT` |

## Display Port MSP Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `displayport_msp_col_adjust` | int8 | master | `-6` – `0` |
| `displayport_msp_row_adjust` | int8 | master | `-3` – `0` |

## Display Port MAX7456 Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `displayport_max7456_col_adjust` | int8 | master | `-6` – `0` |
| `displayport_max7456_row_adjust` | int8 | master | `-3` – `0` |
| `displayport_max7456_inv` | uint8 | master | `OFF`, `ON` |
| `displayport_max7456_blk` | uint8 | master | `0` – `3` |
| `displayport_max7456_wht` | uint8 | master | `0` – `3` |

## ESC Sensor Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `esc_sensor_halfduplex` | uint8 | master | `OFF`, `ON` |
| `esc_sensor_current_offset` | uint16 | master | `0` – `16000` |

## RX Frsky SPI Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `frsky_spi_autobind` | uint8 | master | `OFF`, `ON` |
| `frsky_spi_tx_id` | uint8 | master | array\[2\] |
| `frsky_spi_offset` | int8 | master | `-127` – `127` |
| `frsky_spi_bind_hop_data` | uint8 | master | array\[50\] |
| `frsky_x_rx_num` | uint8 | master | `0` – `255` |
| `frsky_spi_use_external_adc` | uint8 | master | `OFF`, `ON` |

## Status LED Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `led_inversion` | uint8 | master | `0` – `((1 << STATUS_LED_NUMBER) - 1)` |

## Dashboard Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `dashboard_i2c_bus` | uint8 | master | `0` – `I2CDEV_COUNT` |
| `dashboard_i2c_addr` | uint8 | master | `I2C_ADDR7_MIN` – `I2C_ADDR7_MAX` |

## Camera Control Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `camera_control_mode` | uint8 | master | `HARDWARE_PWM`, `SOFTWARE_PWM`, `DAC` |
| `camera_control_ref_voltage` | uint16 | master | `200` – `400` |
| `camera_control_key_delay` | uint16 | master | `100` – `500` |
| `camera_control_internal_resistance` | uint16 | master | `10` – `1000` |
| `camera_control_button_resistance` | uint16 | master | array\[CAMERA_CONTROL_KEYS_COUNT\] |
| `camera_control_inverted` | uint8 | master | `OFF`, `ON` |

## Rangefinder Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `rangefinder_hardware` | uint8 | master | `NONE`, `HCSR04`, `TFMINI`, `TF02` |

## Pinio Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `pinio_config` | uint8 | master | array\[PINIO_COUNT\] |

## Piniobox Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `pinio_box` | uint8 | master | array\[PINIO_COUNT\] |

## USB Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `usb_hid_cdc` | uint8 | master | `OFF`, `ON` |
| `usb_msc_pin_pullup` | uint8 | master | `OFF`, `ON` |

## Flash Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `flash_spi_bus` | uint8 | master | `0` – `SPIDEV_COUNT` |

## RCDEVICE Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `rcdevice_init_dev_attempts` | uint8 | master | `0` – `10` |
| `rcdevice_init_dev_attempt_interval` | uint32 | master | `500` – `5000` |

---
*Generated by `docs/gen_cli_docs.py` from `src/main/interface/settings.c`*
