#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/axis.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/imu_silver.h"
#include "sensors/acceleration.h"

// TODO: remove unused includes

float GEstG[3] = {0.0, 0.0, 1.0f};
float heading = 0;

#define ACC_MIN 0.9f
#define ACC_MAX 1.1f
#define FASTFILTER 0.05
#define PREFILTER 0.5
#define FILTERTIME 4

static void lpf( float *out, float in , float coeff)
{
	*out = ( *out )* coeff + in * ( 1-coeff);
}

static float calcmagnitude(float vector[3])
{
	float accmag = 0;
	for (uint8_t axis = 0; axis < 3; axis++)
    accmag += vector[axis] * vector[axis];
	accmag = sqrtf(accmag);
	return accmag;
}

static float lpfcalc_hz(float sampleperiod, float filterhz) {
  float ga = 1.0f - sampleperiod * filterhz;
    if (ga > 1.0f)
      ga = 1.0f;
    if (ga < 0.0f)
      ga = 0.0f;
  return ga;
}

void imuSilverCalc(float dt, float gx, float gy, float gz, float ax, float ay, float az)
{
  float deltaGyroAngle[3];
  float accel[3];
  float gyro[3];

  accel[0] = ax / acc.dev.acc_1G;
  accel[1] = ay / acc.dev.acc_1G;
  accel[2] = az / acc.dev.acc_1G;

  gyro[0] = -gy;
  gyro[1] = gx;
  gyro[2] = gz;

  deltaGyroAngle[0] = gyro[0] * dt;
  deltaGyroAngle[1] = gyro[1] * dt;
  deltaGyroAngle[2] = gyro[2] * dt;

  GEstG[2] = GEstG[2] - (deltaGyroAngle[0]) * GEstG[0];
  GEstG[0] = (deltaGyroAngle[0]) * GEstG[2] +  GEstG[0];
  GEstG[1] =  GEstG[1] + (deltaGyroAngle[1]) * GEstG[2];
  GEstG[2] = -(deltaGyroAngle[1]) * GEstG[1] +  GEstG[2];
  GEstG[0] = GEstG[0] - (deltaGyroAngle[2]) * GEstG[1];
  GEstG[1] = (deltaGyroAngle[2]) * GEstG[0] +  GEstG[1];

  if (!ARMING_FLAG(ARMED)) { // ON GROUND, disarmed
    float accmag = calcmagnitude(accel);
    if ((accmag > ACC_MIN) && (accmag < ACC_MAX)) {
      // normalize acc
      for (int axis = 0; axis < 3; axis++) {
        accel[axis] = accel[axis] / accmag;
      }
      float filtcoeff = lpfcalc_hz(dt, 1.0f / (float)FASTFILTER);
      for (int x = 0; x < 3; x++) {
        lpf(&GEstG[x], accel[x], filtcoeff);
      }
    }
  } else {  // ON AIR, armed

     float filtcoeff = lpfcalc_hz(dt, 1.0f / (float)PREFILTER);
     for (int x = 0; x < 3; x++) {
       lpf(&accel[x], accel[x], filtcoeff);
     }

     float accmag = calcmagnitude(accel);

     if ((accmag > ACC_MIN) && (accmag < ACC_MAX)) {//The bartender makes the fusion if.....
       // normalize acc
       for (int axis = 0; axis < 3; axis++) {
           accel[axis] = accel[axis] / accmag;
       }
       // filter accel on to GEstG
       float filtcoeff = lpfcalc_hz(dt, 1.0f / (float)FILTERTIME);
       for (int x = 0; x < 3; x++) {
         lpf(&GEstG[x], accel[x], filtcoeff);
       }
       //heal the gravity vector after nudging it with accel (this is the fix for the yaw slow down bug some FC experienced)
       float GEstGmag = calcmagnitude(&GEstG[0]);
       for (int axis = 0; axis < 3; axis++) {
         GEstG[axis] = GEstG[axis] / GEstGmag;
       }
     }
  }

  heading += deltaGyroAngle[2];

  attitude.values.roll = lrintf( atan2_approx(GEstG[Y],GEstG[Z]) * 1800.f / M_PIf );
  attitude.values.pitch = lrintf( -atan2_approx( GEstG[X], GEstG[Z]) * 1800.f / M_PIf);
  attitude.values.yaw = lrintf( -heading * 1800.f / M_PIf);

  if (attitude.values.yaw < 0)
    attitude.values.yaw += 3600;
}
