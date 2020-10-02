#pragma once

#include "common/filter.h"

#include "common/axis.h"

//TYPES
//-----
typedef struct {
      pt1Filter_t pt1;          //PT1 filter
      biquadFilter_t biquad;    //Biquad filter

      float Fc;                 //Cutoff freq
      pt1Filter_t pt1Fc;        //PT1 on Fc

      bool Dyn_Fc;              //Dynamic E or Fixed E
      uint16_t Fmax;
      uint16_t Fmin_init;

      uint16_t throttleThreshold;
      uint16_t throttleGain;

      uint16_t dynGainOnError;

      uint32_t targetLooptime;

      uint8_t  dynlpf2_enable;
      uint8_t  dynlpf2_type;

      uint8_t  dynlpf2_debug;
      flight_dynamics_index_t gyroDebugAxis;
  } dynlpf2_t;

#define DEFAULT_DYNLPF2_ENABLE              1       //Enable DYN_LPF2 by default

#define DEFAULT_DYNLPF2_FMIN                50   //Fmin in Hz
#define DEFAULT_DYNLPF2_FMAX               600   //user Fmax in Hz
#define DEFAULT_DYNLPF2_GAIN                70      //Gain

#define DEFAULT_DYNLPF2_THROTTLE_THRESHOLD  35      //Throttle in %
#define DEFAULT_DYNLPF2_THROTTLE_GAIN       12      // 12Hz / % throrrle over 35%

#define DEFAULT_DYNLPF2_FC_FC              10     //Cut of freq on FC value


#define DEFAULT_DYNLPF2_TYPE                0 //Default


extern void init_dynLpf2(dynlpf2_t* dynLpf, uint32_t targetLooptime, uint16_t min, uint16_t max, uint16_t fc_fc,
  uint16_t threshold, uint16_t throttle_gain, uint16_t gain, uint8_t axis, uint8_t type, uint8_t enable, uint8_t debug);
extern float dynLpf2Apply(dynlpf2_t* filter, int axis, float input, float gyro);
