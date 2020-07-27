#pragma GCC push_options
#pragma GCC optimize("O3")

#include <stdbool.h>

#include "platform.h"
#include "math.h"

#include "dynLpf2.h"

#include "fc/rc.h"

#include "build/debug.h"

#include "common/filter.h"

#include "sensors/gyro.h"

#include "fc/rc_controls.h"

//DEFINITIONS
//-----------
#define DYNLPF2_HYTEREIS  2.0f  //Value in �/s

//TYPES
//-----
  typedef struct {
      pt1Filter_t pt1;          //PT1 filter
      biquadFilter_t biquad;    //Biquad filter

      float Fc;                 //Cutoff freq
      pt1Filter_t pt1Fc;        //PT1 on Fc

      bool Dyn_Fc;              //Dynamic E or Fixed E
  } dynlpf2_t;


//VARIABLES
//---------
    static dynlpf2_t dynLpf[3];

    static float Fmin_init,Fmax;
    static float throttleThreshold;
    static float throttleGain;
    static float dynFcThreshold;
    static float dynGainOnError;
    static flight_dynamics_index_t gyroDebugAxis;

//////////////////////////////
//                          //
//       DYN PT1 INIT       //
//                          //
//////////////////////////////
void init_dynLpf2(void)
{
    const float gyroDt = gyro.targetLooptime * 1e-6f;

    //Init PT1
    float gain = pt1FilterGain(gyroConfig()->dynlpf2_fmin, gyroDt);
    pt1FilterInit(&dynLpf[0].pt1, gain);
    pt1FilterInit(&dynLpf[1].pt1, gain);
    pt1FilterInit(&dynLpf[2].pt1, gain);

    //Init Biquad
    biquadFilterInitLPF(&dynLpf[0].biquad, gyroConfig()->dynlpf2_fmin, gyro.targetLooptime);
    biquadFilterInitLPF(&dynLpf[1].biquad, gyroConfig()->dynlpf2_fmin, gyro.targetLooptime);
    biquadFilterInitLPF(&dynLpf[2].biquad, gyroConfig()->dynlpf2_fmin, gyro.targetLooptime);

    //Fc filter
    gain = pt1FilterGain(gyroConfig()->dynlpf2_fc_fc, gyroDt);
    pt1FilterInit(&dynLpf[0].pt1Fc, gain);
    pt1FilterInit(&dynLpf[1].pt1Fc, gain);
    pt1FilterInit(&dynLpf[2].pt1Fc, gain);

    dynLpf[0].Fc = gyroConfig()->dynlpf2_fmin;
    dynLpf[1].Fc = gyroConfig()->dynlpf2_fmin;
    dynLpf[2].Fc = gyroConfig()->dynlpf2_fmin;

    dynLpf[0].Dyn_Fc = false;
    dynLpf[1].Dyn_Fc = false;
    dynLpf[2].Dyn_Fc = false;

    Fmax                 = (float)gyroConfig()->dynlpf2_fmax;         //PT1 maxFc in Hz
    Fmin_init            = (float)gyroConfig()->dynlpf2_fmin;         //PT1 min Fc in Hz

    throttleThreshold    = (float)gyroConfig()->dynlpf2_throttle_threshold;
    throttleGain         = (float)gyroConfig()->dynlpf2_throttle_gain;

    dynFcThreshold       = (float)(gyroConfig()->dynlpf2_center_threshold);   //Min Setpoint & Gyro value to rise PT1 Fc
    dynGainOnError       = (float)(gyroConfig()->dynlpf2_gain);

    gyroDebugAxis        = gyroConfig()->gyro_filter_debug_axis;
}

//////////////////////////////
//                          //
//      DYN LPF PROCESS     //
//         on ratio         //
//                          //
//////////////////////////////

FAST_CODE float dynlpf2_process_type1(dynlpf2_t* filter, float input, float target) {

float newFc, Fmin;
float throttle;
float Average;

Fmin = Fmin_init;
throttle  = (rcCommand[THROTTLE] - 1000.0f) * 0.1f; //Throttle scaled to [0-100]
const float gyroDt = gyro.targetLooptime * 1e-6f;

    //Compute average between setpoint and Gyro
    //-----------------------------------------
        Average = (target + input) * 0.5f;

    //Check if setpoint or gyro are high enought to compute "e" ratio
    //---------------------------------------------------------------
        if (filter->Dyn_Fc) {
            if ((float)(fabs(Average)) < (dynFcThreshold - DYNLPF2_HYTEREIS)) {
                filter->Dyn_Fc = false;
            }
        } else {
            //Enable Dyn_Fc when stick or Quad move
            if ((float)(fabs(Average)) > (dynFcThreshold + DYNLPF2_HYTEREIS)) {
                filter->Dyn_Fc = true;
            }
        }

    //Rise Fmin according to Throttle;
    //--------------------------------
        if(throttle > throttleThreshold){
            Fmin += (throttle - throttleThreshold) * throttleGain;
        }


    //Compute e & Fc
    //--------------
        if (filter->Dyn_Fc) {

            //Compute e factor
                float Error, e;
                Error = (float)fabs(target - input);
                e = Error / Average;                           //Compute ratio between Error and average. e is image of noise in % of signal

            //New freq
                newFc = Fmin + dynGainOnError * 100.0f  * powf(e, 3.0f);  //"e" power 3 and multiply by a gain

        } else {
                newFc  = Fmin;
        }

    //Limit & Filter newFc
    //---------------------

        newFc = constrainf(newFc, Fmin, Fmax);
        //Filter the cut-off freq ;)
        newFc = pt1FilterApply(&filter->pt1Fc, newFc);

    //Update PT1 filter
    //-----------------
    if (gyroConfig()->dynlpf2_type == PT1) {
        pt1FilterUpdateCutoff(&filter->pt1, pt1FilterGain(newFc, gyroDt));
    } else {
        biquadFilterUpdateLPF(&filter->biquad, newFc, gyro.targetLooptime);
    }
        pt1FilterUpdateCutoff(&filter->pt1, pt1FilterGain(newFc, gyroDt));
        filter->Fc = newFc;

    //Apply filter
    //------------
        float output;

        if (gyroConfig()->dynlpf2_type == PT1) {
            output = pt1FilterApply(&filter->pt1, input);
        } else {
            output = biquadFilterApplyDF1(&filter->biquad, input);
        }
 return output;
}


//////////////////////////////
//                          //
//      DYN LPF2 APPLY      //
//                          //
//////////////////////////////

FAST_CODE float dynLpf2Apply(int axis, float input) {

float output;
float target = getSetpointRate(axis);


  //Apply filter if filter is enable.
    if (gyroConfig()->dynlpf2_enable != 0) {
        output = dynlpf2_process_type1(&dynLpf[axis], input, target);
    } else {
      output = input;
      dynLpf[axis].Fc = 0;  //To show filter is disable in blackbox.
    }


  //Blackbox
    if (axis == gyroDebugAxis) {
        DEBUG_SET(DEBUG_DYN_LPF2, 0, (int16_t)(lrintf(input)));
        DEBUG_SET(DEBUG_DYN_LPF2, 1, (int16_t)(lrintf(output)));
        DEBUG_SET(DEBUG_DYN_LPF2, 2, (int16_t)(lrintf(dynLpf[axis].Fc)));
        DEBUG_SET(DEBUG_DYN_LPF2, 3, (int16_t)(lrintf(target)));
    }

return output;
}

#pragma GCC pop_options
