#pragma once



//Max N = 25
typedef struct {
    int windowSize;
    int windowBufIndex;
    int N;
    float buf[51];
    float minBufA[51];
    float luluInterimA[51];
    float luluInterimB[51];
} luluFilter_t;

typedef struct {
	luluFilter_t A;
	luluFilter_t B;
} luluFilter2_t;

void luluFilterInit(luluFilter_t *filter, int N);
float luluFilterApply(luluFilter2_t *filter, float input);
