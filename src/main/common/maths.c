/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>

#include "axis.h"
#include "maths.h"

#ifdef USE_ARM_MATH
#include "arm_math.h"
#endif 

#if defined(FAST_MATH)

// Backport of betaflight/betaflight#14790 (ledvinap): O(1) roundf-based range reduction, no while loop.
static inline float sin_poly5_r(float r)
{
    const float c0 =  0x1.921f1cp0f;  // 1.5707871913909912109375
    const float c1 = -0x1.4a974p-1f;  // -0.6456851959228515625
    const float c2 =  0x1.3db294p-4f; // 7.756288349628448486328125e-2
    float s = r * r;
    return r * ((c2 * s + c1) * s + c0);
}

static inline float cos_poly6_r(float r)
{
    const float d1 = -0x1.3bd39cp0f;  // -1.2336976528167724609375
    const float d2 =  0x1.03bp-2f;    // 0.25360107421875
    const float d3 = -0x1.4e5eecp-6f; // -2.04083733260631561279296875e-2
    float s = r * r;
    return ((d3 * s + d2) * s + d1) * s + 1.0f;
}

// r in [-0.5, 0.5], q is quadrant index (..., -1, 0, 1, 2, 3, 4, ...).
static inline float sinf_quadrant_r(float r, int q)
{
    q &= 3;
    if (q & 1) {
        float v = cos_poly6_r(r);
        return (q & 2) ? -v : v;
    } else {
        float v = sin_poly5_r(r);
        return (q & 2) ? -v : v;
    }
}

static inline float cosf_quadrant_r(float r, int q)
{
    q &= 3;
    if (q & 1) {
        float v = -sin_poly5_r(r);
        return (q & 2) ? -v : v;
    } else {
        float v = cos_poly6_r(r);
        return (q & 2) ? -v : v;
    }
}

static inline void sincosf_quadrant_r(float r, int q, float *out_s, float *out_c)
{
    q &= 3;
    float sb = sin_poly5_r(r);
    float cb = cos_poly6_r(r);

    float s = (q & 1) ? cb  : sb;
    float c = (q & 1) ? -sb : cb;

    if (q & 2) { s = -s; c = -c; }

    *out_s = s;
    *out_c = c;
}

float sin_approx(float x)
{
    float t = x * INV_PIO2;
    float qf = roundf(t);
    int   q  = (int)qf;
    float r  = t - qf;
    return sinf_quadrant_r(r, q);
}

float cos_approx(float x)
{
    float t = x * INV_PIO2;
    float qf = roundf(t);
    int   q  = (int)qf;
    float r  = t - qf;
    return cosf_quadrant_r(r, q);
}

void sincosf_approx(float x, float *out_s, float *out_c)
{
    float t = x * INV_PIO2;
    float qf = roundf(t);
    int   q  = (int)qf;
    float r  = t - qf;
    sincosf_quadrant_r(r, q, out_s, out_c);
}

// Initial implementation by Crashpilot1000 (https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292)
// Polynomial coefficients by Andor (http://www.dsprelated.com/showthread/comp.dsp/21872-1.php) optimized by Ledvinap to save one multiplication
// Max absolute error 0,000027 degree
// atan2_approx maximum absolute error = 7.152557e-07 rads (4.098114e-05 degree)
float atan2_approx(float y, float x) {
#define atanPolyCoef1  3.14551665884836e-07f
#define atanPolyCoef2  0.99997356613987f
#define atanPolyCoef3  0.14744007058297684f
#define atanPolyCoef4  0.3099814292351353f
#define atanPolyCoef5  0.05030176425872175f
#define atanPolyCoef6  0.1471039133652469f
#define atanPolyCoef7  0.6444640676891548f
    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
}

// http://http.developer.nvidia.com/Cg/acos.html
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.
// acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03 degree)
float acos_approx(float x) {
    float xa = fabsf(x);
    float result = fast_fsqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return M_PIf - result;
    else
        return result;
}
#endif

float fast_fsqrtf(const double value) {
#ifdef USE_ARM_MATH
    float squirt;
    arm_sqrt_f32(value, &squirt);
    return squirt;
#else 
    return sqrtf(value);
#endif
}

int gcd(int num, int denom) {
    if (denom == 0) {
        return num;
    }
    return gcd(denom, num % denom);
}

int32_t applyDeadband(const int32_t value, const int32_t deadband) {
    if (ABS(value) < deadband) {
        return 0;
    }
    return value >= 0 ? value - deadband : value + deadband;
}

float fapplyDeadband(const float value, const float deadband) {
    if (fabsf(value) < deadband) {
        return 0;
    }
    return value >= 0 ? value - deadband : value + deadband;
}

void devClear(stdev_t *dev) {
    dev->m_n = 0;
}

void devPush(stdev_t *dev, float x) {
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

float devVariance(stdev_t *dev) {
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev) {
    return fast_fsqrtf(devVariance(dev));
}

float degreesToRadians(int16_t degrees) {
    return degrees * RAD;
}

int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo) {
    long int a = ((long int) destTo - (long int) destFrom) * ((long int) x - (long int) srcFrom);
    long int b = (long int) srcTo - (long int) srcFrom;
    return (a / b) + destFrom;
}

float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo) {
    float a = (destTo - destFrom) * (x - srcFrom);
    float b = srcTo - srcFrom;
    return (a / b) + destFrom;
}

void buildRotationMatrix(fp_angles_t *delta, float matrix[3][3]) {
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;
    sincosf_approx(delta->angles.roll, &sinx, &cosx);
    sincosf_approx(delta->angles.pitch, &siny, &cosy);
    sincosf_approx(delta->angles.yaw, &sinz, &cosz);
    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;
    matrix[0][X] = cosz * cosy;
    matrix[0][Y] = -cosy * sinz;
    matrix[0][Z] = siny;
    matrix[1][X] = sinzcosx + (coszsinx * siny);
    matrix[1][Y] = coszcosx - (sinzsinx * siny);
    matrix[1][Z] = -sinx * cosy;
    matrix[2][X] = (sinzsinx) - (coszcosx * siny);
    matrix[2][Y] = (coszsinx) + (sinzcosx * siny);
    matrix[2][Z] = cosy * cosx;
}

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }
#define QMF_SORTF(a,b) { if ((a)>(b)) QMF_SWAPF((a),(b)); }
#define QMF_SWAPF(a,b) { float temp=(a);(a)=(b);(b)=temp; }

int32_t quickMedianFilter3(int32_t * v) {
    int32_t p[3];
    QMF_COPY(p, v, 3);
    QMF_SORT(p[0], p[1]);
    QMF_SORT(p[1], p[2]);
    QMF_SORT(p[0], p[1]) ;
    return p[1];
}

int32_t quickMedianFilter5(int32_t * v) {
    int32_t p[5];
    QMF_COPY(p, v, 5);
    QMF_SORT(p[0], p[1]);
    QMF_SORT(p[3], p[4]);
    QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[4]);
    QMF_SORT(p[1], p[2]);
    QMF_SORT(p[2], p[3]);
    QMF_SORT(p[1], p[2]);
    return p[2];
}

int32_t quickMedianFilter7(int32_t * v) {
    int32_t p[7];
    QMF_COPY(p, v, 7);
    QMF_SORT(p[0], p[5]);
    QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[6]);
    QMF_SORT(p[2], p[4]);
    QMF_SORT(p[0], p[1]);
    QMF_SORT(p[3], p[5]);
    QMF_SORT(p[2], p[6]);
    QMF_SORT(p[2], p[3]);
    QMF_SORT(p[3], p[6]);
    QMF_SORT(p[4], p[5]);
    QMF_SORT(p[1], p[4]);
    QMF_SORT(p[1], p[3]);
    QMF_SORT(p[3], p[4]);
    return p[3];
}

int32_t quickMedianFilter9(int32_t * v) {
    int32_t p[9];
    QMF_COPY(p, v, 9);
    QMF_SORT(p[1], p[2]);
    QMF_SORT(p[4], p[5]);
    QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[1]);
    QMF_SORT(p[3], p[4]);
    QMF_SORT(p[6], p[7]);
    QMF_SORT(p[1], p[2]);
    QMF_SORT(p[4], p[5]);
    QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[3]);
    QMF_SORT(p[5], p[8]);
    QMF_SORT(p[4], p[7]);
    QMF_SORT(p[3], p[6]);
    QMF_SORT(p[1], p[4]);
    QMF_SORT(p[2], p[5]);
    QMF_SORT(p[4], p[7]);
    QMF_SORT(p[4], p[2]);
    QMF_SORT(p[6], p[4]);
    QMF_SORT(p[4], p[2]);
    return p[4];
}

float quickMedianFilter3f(float * v) {
    float p[3];
    QMF_COPY(p, v, 3);
    QMF_SORTF(p[0], p[1]);
    QMF_SORTF(p[1], p[2]);
    QMF_SORTF(p[0], p[1]) ;
    return p[1];
}

float quickMedianFilter5f(float * v) {
    float p[5];
    QMF_COPY(p, v, 5);
    QMF_SORTF(p[0], p[1]);
    QMF_SORTF(p[3], p[4]);
    QMF_SORTF(p[0], p[3]);
    QMF_SORTF(p[1], p[4]);
    QMF_SORTF(p[1], p[2]);
    QMF_SORTF(p[2], p[3]);
    QMF_SORTF(p[1], p[2]);
    return p[2];
}

float quickMedianFilter7f(float * v) {
    float p[7];
    QMF_COPY(p, v, 7);
    QMF_SORTF(p[0], p[5]);
    QMF_SORTF(p[0], p[3]);
    QMF_SORTF(p[1], p[6]);
    QMF_SORTF(p[2], p[4]);
    QMF_SORTF(p[0], p[1]);
    QMF_SORTF(p[3], p[5]);
    QMF_SORTF(p[2], p[6]);
    QMF_SORTF(p[2], p[3]);
    QMF_SORTF(p[3], p[6]);
    QMF_SORTF(p[4], p[5]);
    QMF_SORTF(p[1], p[4]);
    QMF_SORTF(p[1], p[3]);
    QMF_SORTF(p[3], p[4]);
    return p[3];
}

float quickMedianFilter9f(float * v) {
    float p[9];
    QMF_COPY(p, v, 9);
    QMF_SORTF(p[1], p[2]);
    QMF_SORTF(p[4], p[5]);
    QMF_SORTF(p[7], p[8]);
    QMF_SORTF(p[0], p[1]);
    QMF_SORTF(p[3], p[4]);
    QMF_SORTF(p[6], p[7]);
    QMF_SORTF(p[1], p[2]);
    QMF_SORTF(p[4], p[5]);
    QMF_SORTF(p[7], p[8]);
    QMF_SORTF(p[0], p[3]);
    QMF_SORTF(p[5], p[8]);
    QMF_SORTF(p[4], p[7]);
    QMF_SORTF(p[3], p[6]);
    QMF_SORTF(p[1], p[4]);
    QMF_SORTF(p[2], p[5]);
    QMF_SORTF(p[4], p[7]);
    QMF_SORTF(p[4], p[2]);
    QMF_SORTF(p[6], p[4]);
    QMF_SORTF(p[4], p[2]);
    return p[4];
}

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count) {
    for (int i = 0; i < count; i++) {
        dest[i] = array1[i] - array2[i];
    }
}

int16_t qPercent(fix12_t q) {
    return (100 * q) >> 12;
}

int16_t qMultiply(fix12_t q, int16_t input) {
    return (input *  q) >> 12;
}

fix12_t  qConstruct(int16_t num, int16_t den) {
    return (num << 12) / den;
}

// quaternions
void quaternionTransformVectorBodyToEarth(quaternion *qVector, quaternion *qReference) {
    quaternion qVectorBuffer, qReferenceConjugate;
    qVector->w = 0;
    quaternionCopy(qVector, &qVectorBuffer);
    quaternionConjugate(qReference, &qReferenceConjugate);
    quaternionMultiply(qReference, &qVectorBuffer, &qVectorBuffer);
    quaternionMultiply(&qVectorBuffer, &qReferenceConjugate, qVector);
}

void quaternionTransformVectorEarthToBody(quaternion *qVector, quaternion *qReference) {
    quaternion qVectorBuffer, qReferenceConjugate;
    qVector->w = 0;
    quaternionCopy(qVector, &qVectorBuffer);
    quaternionConjugate(qReference, &qReferenceConjugate);
    quaternionMultiply(&qReferenceConjugate, &qVectorBuffer, &qVectorBuffer);
    quaternionMultiply(&qVectorBuffer, qReference, qVector);
}

void quaternionComputeProducts(quaternion *qIn, quaternionProducts *qPout) {
    qPout->ww = qIn->w * qIn->w;
    qPout->wx = qIn->w * qIn->x;
    qPout->wy = qIn->w * qIn->y;
    qPout->wz = qIn->w * qIn->z;
    qPout->xx = qIn->x * qIn->x;
    qPout->xy = qIn->x * qIn->y;
    qPout->xz = qIn->x * qIn->z;
    qPout->yy = qIn->y * qIn->y;
    qPout->yz = qIn->y * qIn->z;
    qPout->zz = qIn->z * qIn->z;
}

void quaternionMultiply(quaternion *l, quaternion *r, quaternion *o) {
    const float w = l->w * r->w - l->x * r->x - l->y * r->y - l->z * r->z;
    const float x = l->w * r->x + l->x * r->w + l->y * r->z - l->z * r->y;
    const float y = l->w * r->y - l->x * r->z + l->y * r->w + l->z * r->x;
    const float z = l->w * r->z + l->x * r->y - l->y * r->x + l->z * r->w;
    o->w = w;
    o->x = x;
    o->y = y;
    o->z = z;
}

void quaternionNormalize(quaternion *q) {
    float modulus = quaternionModulus(q);
    if (modulus == 0) {
        // normalization not possible
    } else {
        q->w /= modulus;
        q->x /= modulus;
        q->y /= modulus;
        q->z /= modulus;
    }
}

void quaternionAdd(quaternion *l, quaternion *r, quaternion *o) {
    o->w = l->w + r->w;
    o->x = l->x + r->x;
    o->y = l->y + r->y;
    o->z = l->z + r->z;
}

void quaternionCopy(quaternion *s, quaternion *d) {
    d->w = s->w;
    d->x = s->x;
    d->y = s->y;
    d->z = s->z;
}

void quaternionConjugate(quaternion *i, quaternion *o) {
    o->w = + i->w;
    o->x = - i->x;
    o->y = - i->y;
    o->z = - i->z;
}

float quaternionDotProduct(quaternion *l, quaternion *r) {
    return (l->w * r->w + l->x * r->x + l->y * r->y + l->z * r->z);
}

float quaternionNorm(quaternion *q) {
    return (q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

float quaternionModulus(quaternion *q) {
    return (fast_fsqrtf(quaternionNorm(q)));
}

void quaternionInitQuaternion(quaternion *i) {
    i->w = 1;
    i->x = 0;
    i->y = 0;
    i->z = 0;
}

void quaternionInitVector(quaternion *i) {
    i->w = 0;
    i->x = 0;
    i->y = 0;
    i->z = 0;
}
