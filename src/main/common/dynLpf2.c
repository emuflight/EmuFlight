
#include <stdbool.h>

#include "platform.h"
#include "math.h"

#include "dynLpf2.h"

#include "build/debug.h"

#include "common/filter.h"

#include "sensors/gyro.h"

#ifdef	USE_DYN_LPF2
//DEFINITIONS
//-----------
#define Sqr(x) ((x)*(x))

//TYPES
//-----
typedef struct ABGF_s {
    float a, b, g;
    float ak_1, vk_1, xk_1;
   // real32 dT, dT2;
} filterStructABG;

typedef enum {
    CRITICAL_DAMPED = 0,
    UNDER_DAMPED,
} eABGF;


//VARIABLES
//---------
static filterStructABG agbFilter[3];
static float gyroDt;
//________________________________________________________________________________________

// Robert Bouwens

// www.megamanual.com/alphabeta.htm
// Alpha, Beta and Gamma can be considered to proportional, first and second derivative terms respectively
// Alpha 0.5..1.5 (0.9) Beta < 1.0 (0.8)  Gamma < 0.5 (0.1)


void initABGLPF(filterStructABG *F, float alpha, eABGF ftype) {

	F->xk_1 = 0.0f;
	F->vk_1 = 0.0f;
	F->ak_1 = 0.0f;

	if (ftype == CRITICAL_DAMPED) {
		// near critically damped F
		const float beta = 0.8f
				* (2.0f - Sqr(alpha) - 2.0f * sqrtf(1.0f - Sqr(alpha)))
				/ (Sqr(alpha));
		F->a = alpha;
		F->b = beta;
		F->g = Sqr(beta) / (alpha * 2.0f);
	} else {
		const float beta = Sqr(alpha) / (2.0f - alpha); //  standard, under damped beta value
		F->a = alpha;
		F->b = beta;
		F->g = Sqr(beta) / (alpha * 2.0f);
	}

} // ABGInit

float ABGLPF(filterStructABG *F, float input, float dT) {
	float x0 = F->xk_1;
	const float dT2 = Sqr(dT);

	// update our (estimated) state 'x' from the system (ie pos = pos + vel (last).dT)
	F->xk_1 += dT * F->vk_1 + (0.5f * dT2 * F->ak_1);
	// update (estimated) velocity
	F->vk_1 += dT * F->ak_1;
	// what is our residual error (measured - estimated)
	const float rk = input - F->xk_1;
	// update our estimates given the residual error.
	F->xk_1 += F->a * rk;                 // prediction
	F->xk_1 += F->a * (x0 - F->xk_1);     // correction

	float v0 = F->vk_1;
	F->vk_1 += (F->b / dT) * rk;        // prediction
	F->vk_1 += F->b * (v0 - F->vk_1);   // correction

	if (F->g != 0.0f) {
		F->ak_1 += (F->g / (2.0f * dT2)) * rk;   // prediction
		F->ak_1 += (F->g / dT) * (v0 - F->vk_1); // correction
	}

	return F->xk_1;

} // ABG



//////////////////////////////
//                          //
//       DYN PT1 INIT       //
//                          //
//////////////////////////////
void init_dynLpf2(void)
{
   	const float		ABGalpha = 0.5f;
   	const uint8_t	ABGType = UNDER_DAMPED;

    gyroDt = gyro.targetLooptime * 1e-6f;
    initABGLPF(&agbFilter[0], ABGalpha, ABGType);
    initABGLPF(&agbFilter[1], ABGalpha, ABGType);
    initABGLPF(&agbFilter[2], ABGalpha, ABGType);
}


//////////////////////////////
//                          //
//      DYN LPF2 APPLY      //
//                          //
//////////////////////////////

FAST_CODE float dynLpf2Apply(int axis, float input)
{
	float output;

	// Apply filter if filter is enable.
    if (gyroConfigMutable()->dynlpf2_enable != 0)
    {
        output = ABGLPF(&agbFilter[axis], input, gyroDt);
    }
    else
    {
        output = input;
    }

    return output;
}


#endif
