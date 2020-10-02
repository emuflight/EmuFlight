#include <stdbool.h>

#include "platform.h"
#include "math.h"

#include "dynlpf2.h"

#include "fc/rc.h"

#include "build/debug.h"

#include "common/filter.h"

#include "sensors/gyro.h"

#include "fc/rc_controls.h"

//DEFINITIONS
//-----------
#define DYNLPF2_HYTEREIS  2.0f  //Value in �/s

//////////////////////////////
//                          //
//       DYN PT1 INIT       //
//                          //
//////////////////////////////

void init_dynLpf2(dynlpf2_t* dynLpf, uint32_t targetLooptime, uint16_t min, uint16_t max, uint16_t fc_fc,
  uint16_t threshold, uint16_t throttle_gain, uint16_t gain, uint8_t axis, uint8_t type, uint8_t enable, uint8_t debug)
{
    const float gyroDt = targetLooptime * 1e-6f;

    //Init PT1
    float pt1gain = pt1FilterGain(min, gyroDt);
    pt1FilterInit(&dynLpf->pt1, pt1gain);

    //Init Biquad
    biquadFilterInitLPF(&dynLpf->biquad, min, targetLooptime);

    //Fc filter
    pt1gain = pt1FilterGain(fc_fc, gyroDt);
    pt1FilterInit(&dynLpf->pt1Fc, pt1gain);

    dynLpf->Fc                   = min;

    dynLpf->Fmax                 = max;         //PT1 maxFc in Hz
    dynLpf->Fmin_init            = min;         //PT1 min Fc in Hz

    dynLpf->throttleThreshold    = threshold;
    dynLpf->throttleGain         = throttle_gain;

    dynLpf->dynGainOnError       = gain;

    dynLpf->gyroDebugAxis        = axis;
    dynLpf->dynlpf2_debug        = debug;
    dynLpf->dynlpf2_type         = type;
    dynLpf->dynlpf2_enable       = enable;
    dynLpf->targetLooptime       = targetLooptime;
}

//////////////////////////////
//                          //
//      DYN LPF PROCESS     //
//         on ratio         //
//                          //
//////////////////////////////

FAST_CODE float dynlpf2_process_type1(dynlpf2_t* filter, float input, float target, float gyro)
{
float newFc, Fmin;
float throttle;
float Average;

Fmin = filter->Fmin_init;
throttle  = (rcCommand[THROTTLE] - 1000.0f) * 0.1f; //Throttle scaled to [0-100]
const float gyroDt = filter->targetLooptime * 1e-6f;

    //Compute average between setpoint and Gyro
    //-----------------------------------------
    //take the absolute value of target and gyro
    //to guard against situations where the sign of target and gyro are not the same
        Average = (fabsf(target) + fabsf(gyro)) * 0.5f;
        Average = MAX(Average, 15.0f);
    //Rise Fmin according to Throttle;
    //--------------------------------
        if(throttle > filter->throttleThreshold) {
            Fmin += (throttle - filter->throttleThreshold) * filter->throttleGain;
        }


    //Compute e & Fc
    //--------------

    //Compute e factor
        float Error, e;
        Error = fabsf(target - gyro);
        e = Error / Average;                           //Compute ratio between Error and average. e is image of noise in % of signal

        //New freq
        newFc = Fmin + filter->dynGainOnError * 100.0f  * powf(e, 3.0f);  //"e" power 3 and multiply by a gain
    //Limit & Filter newFc
    //---------------------

        newFc = constrainf(newFc, Fmin, filter->Fmax);
        //Filter the cut-off freq ;)
        newFc = pt1FilterApply(&filter->pt1Fc, newFc);

    //Update PT1 filter
    //-----------------
    if (filter->dynlpf2_type == 0) {
        pt1FilterUpdateCutoff(&filter->pt1, pt1FilterGain(newFc, gyroDt));
    } else {
        biquadFilterUpdateLPF(&filter->biquad, newFc, filter->targetLooptime);
    }
        filter->Fc = newFc;

    //Apply filter
    //------------
        float output;

        if (filter->dynlpf2_type == 0) {
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

FAST_CODE float dynLpf2Apply(dynlpf2_t* filter, int axis, float input, float gyro) {

float output;
float target = getSetpointRate(axis);


  //Apply filter if filter is enable.
    if (filter->dynlpf2_enable != 0) {
        output = dynlpf2_process_type1(filter, input, target, gyro);
    } else {
      output = input;
      filter->Fc = 0;  //To show filter is disable in blackbox.
    }


  //Blackbox
    if (axis == filter->gyroDebugAxis && filter->dynlpf2_debug) {
        DEBUG_SET(DEBUG_DYN_LPF2, 0, (int16_t)(lrintf(input)));
        DEBUG_SET(DEBUG_DYN_LPF2, 1, (int16_t)(lrintf(output)));
        DEBUG_SET(DEBUG_DYN_LPF2, 2, (int16_t)(lrintf(filter->Fc)));
        DEBUG_SET(DEBUG_DYN_LPF2, 3, (int16_t)(lrintf(target)));
    }

return output;
}
