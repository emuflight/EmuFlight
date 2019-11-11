#include <string.h>
#include "arm_math.h"

#include "kalman.h"
#include "fc/fc_rc.h"
#include "build/debug.h"

#ifndef USE_GYRO_IMUF9001
#define MAX_KALMAN_WINDOW_SIZE 512

typedef struct variance
{
    float xVar;
    float yVar;
    float zVar;
    float xyCoVar;
    float xzCoVar;
    float yzCoVar;

    uint32_t windex;
    float xWindow[MAX_KALMAN_WINDOW_SIZE];
    float yWindow[MAX_KALMAN_WINDOW_SIZE];
    float zWindow[MAX_KALMAN_WINDOW_SIZE];

    float xSumMean;
    float ySumMean;
    float zSumMean;

    float xMean;
    float yMean;
    float zMean;

    float xSumVar;
    float ySumVar;
    float zSumVar;
    float xySumCoVar;
    float xzSumCoVar;
    float yzSumCoVar;

    float inverseN;
    uint32_t w;
} variance_t;


typedef struct kalman
{
    float q;     //process noise covariance
    float r;     //measurement noise covariance
    float p;     //estimation error covariance matrix
    float k;     //kalman gain
    float x;     //state
    float lastX; //previous state
    float e;
} kalman_t;


kalman_t    kalmanFilterStateRate[XYZ_AXIS_COUNT];
variance_t  varStruct;
float       setPoint[XYZ_AXIS_COUNT];



void init_kalman(kalman_t *filter, float q)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.000001f;   //add multiplier to make tuning easier
    filter->r = 88.0f;           //seeding R at 88.0f
    filter->p = 30.0f;           //seeding P at 30.0f
    filter->e = 1.0f;
}


void kalman_init(void)
{
    isSetpointNew = 0;

    setPoint[X]= 0.0f;
    setPoint[Y] = 0.0f;
    setPoint[Z] = 0.0f;

    memset(&varStruct, 0, sizeof(varStruct));

    init_kalman(&kalmanFilterStateRate[X],  gyroConfig()->imuf_roll_q);
    init_kalman(&kalmanFilterStateRate[Y],  gyroConfig()->imuf_pitch_q);
    init_kalman(&kalmanFilterStateRate[Z],  gyroConfig()->imuf_yaw_q);

    varStruct.w = gyroConfig()->imuf_w;
    varStruct.inverseN = 1.0f/(float)(varStruct.w);
}


#pragma GCC push_options
#pragma GCC optimize("O3")
void update_kalman_covariance(float *gyroRateData)
{
     varStruct.xWindow[ varStruct.windex] = gyroRateData[X];
     varStruct.yWindow[ varStruct.windex] = gyroRateData[Y];
     varStruct.zWindow[ varStruct.windex] = gyroRateData[Z];

     varStruct.xSumMean +=  varStruct.xWindow[ varStruct.windex];
     varStruct.ySumMean +=  varStruct.yWindow[ varStruct.windex];
     varStruct.zSumMean +=  varStruct.zWindow[ varStruct.windex];
     varStruct.xSumVar =  varStruct.xSumVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.xWindow[ varStruct.windex]);
     varStruct.ySumVar =  varStruct.ySumVar + ( varStruct.yWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.zSumVar =  varStruct.zSumVar + ( varStruct.zWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.xySumCoVar =  varStruct.xySumCoVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.xzSumCoVar =  varStruct.xzSumCoVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.yzSumCoVar =  varStruct.yzSumCoVar + ( varStruct.yWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.windex++;
    if ( varStruct.windex >= varStruct.w)
    {
         varStruct.windex = 0;
    }
     varStruct.xSumMean -=  varStruct.xWindow[ varStruct.windex];
     varStruct.ySumMean -=  varStruct.yWindow[ varStruct.windex];
     varStruct.zSumMean -=  varStruct.zWindow[ varStruct.windex];
     varStruct.xSumVar =  varStruct.xSumVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.xWindow[ varStruct.windex]);
     varStruct.ySumVar =  varStruct.ySumVar - ( varStruct.yWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.zSumVar =  varStruct.zSumVar - ( varStruct.zWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.xySumCoVar =  varStruct.xySumCoVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.xzSumCoVar =  varStruct.xzSumCoVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.yzSumCoVar =  varStruct.yzSumCoVar - ( varStruct.yWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);

     varStruct.xMean =  varStruct.xSumMean *  varStruct.inverseN;
     varStruct.yMean =  varStruct.ySumMean *  varStruct.inverseN;
     varStruct.zMean =  varStruct.zSumMean *  varStruct.inverseN;

     varStruct.xVar =  ABS(varStruct.xSumVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.xMean));
     varStruct.yVar =  ABS(varStruct.ySumVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.yMean));
     varStruct.zVar =  ABS(varStruct.zSumVar *  varStruct.inverseN - ( varStruct.zMean *  varStruct.zMean));
     varStruct.xyCoVar =  ABS(varStruct.xySumCoVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.yMean));
     varStruct.xzCoVar =  ABS(varStruct.xzSumCoVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.zMean));
     varStruct.yzCoVar =  ABS(varStruct.yzSumCoVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.zMean));

    float squirt;
    arm_sqrt_f32(varStruct.xVar +  varStruct.xyCoVar +  varStruct.xzCoVar, &squirt);
    kalmanFilterStateRate[X].r = squirt;

    arm_sqrt_f32(varStruct.yVar +  varStruct.xyCoVar +  varStruct.yzCoVar, &squirt);
    kalmanFilterStateRate[Y].r = squirt;

    arm_sqrt_f32(varStruct.zVar +  varStruct.yzCoVar +  varStruct.xzCoVar, &squirt);
    kalmanFilterStateRate[Z].r = squirt;
}

inline float kalman_process(kalman_t* kalmanState, float input, float target)
{
	//project the state ahead using acceleration
    kalmanState->x += (kalmanState->x - kalmanState->lastX);

    //figure out how much to boost or reduce our error in the estimate based on setpoint target.
    //this should be close to 0 as we approach the setpoint and really high the further away we are from the setpoint.
    //update last state
    kalmanState->lastX = kalmanState->x;

    /*if (target != 0.0f && input  != 0.0f)
    {
        kalmanState->e = ABS(1.0f - target/input);
    }
    else
    {
    //    UNUSED(target);
        kalmanState->e = 1.0f;
    }*/

    kalmanState->e = (ABS((target - input) * 2) + ABS(input / 5));


    //prediction update
    kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);

    //measurement update
    kalmanState->k = kalmanState->p / (kalmanState->p + kalmanState->r);
    kalmanState->x += kalmanState->k * (input - kalmanState->x);
    kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;

    return kalmanState->x;
}


void kalman_update(float* input, float* output)
{
    if(isSetpointNew) {
        setPoint[X] = getSetpointRate(X);
        setPoint[Y] = getSetpointRate(Y);
        setPoint[Z] = getSetpointRate(Z);

        isSetpointNew = 0;
    }

    update_kalman_covariance(input);

    output[X] = kalman_process(&kalmanFilterStateRate[X], input[X], setPoint[X] );
    output[Y] = kalman_process(&kalmanFilterStateRate[Y], input[Y], setPoint[Y] );
    output[Z] = kalman_process(&kalmanFilterStateRate[Z], input[Z], setPoint[Z] );

    DEBUG_SET(DEBUG_KALMAN, 1, input[X]);                               //Gyro input

    int16_t Kgain = (kalmanFilterStateRate[X].k * 1000.0f);
    DEBUG_SET(DEBUG_KALMAN, 2, Kgain);                                  //Kalman gain
    DEBUG_SET(DEBUG_KALMAN, 3, output[X]);                              //Kalman output
}

#pragma GCC pop_options
#endif
