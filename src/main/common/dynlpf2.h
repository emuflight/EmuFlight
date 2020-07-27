#pragma once

typedef enum {
  PT1 = 0,
  BIQUAD = 1,
}DYN_LPF2_TYPE;



#define DEFAULT_DYNLPF2_ENABLE              1       //Enable DYN_LPF2 by default

#define DEFAULT_DYNLPF2_CENTER_THRESHOLD    10.0f   //Value in �/s

#define DEFAULT_DYNLPF2_FMIN                50.0f   //Fmin in Hz
#define DEFAULT_DYNLPF2_FMAX               600.0f   //user Fmax in Hz
#define DEFAULT_DYNLPF2_GAIN                70      //Gain

#define DEFAULT_DYNLPF2_THROTTLE_THRESHOLD  35      //Throttle in %
#define DEFAULT_DYNLPF2_THROTTLE_GAIN       12      // 12Hz / % throrrle over 35%

#define DEFAULT_DYNLPF2_FC_FC              10.0f     //Cut of freq on FC value


#define DEFAULT_DYNLPF2_TYPE                PT1 //Default


extern void init_dynLpf2(void);
extern float dynLpf2Apply(int axis, float input);
