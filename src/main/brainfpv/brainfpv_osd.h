
#ifndef BRAINFPV_OSD_H
#define BRAINFPV_OSD_H

#include <stdint.h>
#include "brainfpv/video.h"
#include "pg/pg.h"

typedef struct bfOsdConfig_s {
    uint8_t sync_threshold;
    uint8_t white_level;
    uint8_t black_level;
    int8_t x_offset;
    uint8_t x_scale;
    uint8_t sbs_3d_enabled;
    uint8_t sbs_3d_right_eye_offset;
    uint8_t font;
    uint8_t ir_system;
    uint16_t ir_trackmate_id;
    uint32_t ir_ilap_id;
    uint8_t ahi_steps;
    uint8_t bmi160foc;
    uint16_t bmi160foc_ret;
    uint8_t altitude_scale;
    uint8_t speed_scale;
    uint8_t map;
    uint16_t map_max_dist_m;
    uint8_t sticks_display;
    uint8_t show_logo_on_arm;
    uint8_t show_pilot_logo;
    uint8_t invert;
    uint8_t hd_frame;
    uint8_t hd_frame_width;
    uint8_t hd_frame_height;
    int8_t hd_frame_h_offset;
    int8_t hd_frame_v_offset;
    uint8_t crsf_link_stats;
    uint8_t crsf_link_stats_power;
    uint8_t crsf_link_stats_rssi;
    uint8_t crsf_link_stats_snr;
    int8_t crsf_link_stats_snr_threshold;
#if defined(USE_BRAINFPV_SPECTROGRAPH)
    uint8_t spec_enabled;
#endif
} bfOsdConfig_t;

typedef enum {
    CRSF_OFF = 0,
    CRSF_LQ_LOW = 1,
    CRSF_SNR_LOW = 2,
    CRSF_ON = 3,
} CrsfMode_t;

PG_DECLARE(bfOsdConfig_t, bfOsdConfig);

void brainFpvOsdInit(void);
void osdMain(void);
void resetBfOsdConfig(bfOsdConfig_t *bfOsdConfig);

void brainFpvOsdArtificialHorizon(void);
void brainFpvOsdCenterMark(void);
void brainFpvOsdUserLogo(uint16_t x, uint16_t y);
void brainFpvOsdMainLogo(uint16_t x, uint16_t y);

bool osdElementRssi_BrainFPV(uint16_t x_pos, uint16_t y_pos);

#endif /* BRAINFPV_OSD */
