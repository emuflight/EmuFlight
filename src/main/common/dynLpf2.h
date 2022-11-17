#pragma once

#define DEFAULT_DYNLPF2_ENABLE          1     //Enable DYN_LPF2 by default

extern void init_dynLpf2(void);
extern float dynLpf2Apply(int axis, float input);
