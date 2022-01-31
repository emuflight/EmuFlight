#include "lulu.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "fc/fc_rc.h"


void luluFilterInit(luluFilter_t *filter, int N)
{
    filter->N = N;
    filter->windowSize = N*2 + 1;
    filter->windowBufIndex = 0;
    memset(filter->buf, 0, sizeof(float) * (2 * N + 1));
    memset(filter->minBufA, 0, sizeof(float) * (2 * N + 1));
    memset(filter->luluInterimA, 0, sizeof(float) * (2 * N + 1));
    memset(filter->luluInterimB, 0, sizeof(float) * (2 * N + 1));
}



FAST_CODE float luluFilterPartialApply(luluFilter_t *filter, float input)
{
	int windowIndex = filter->windowBufIndex;
	//Update the current filter window
	filter->buf[windowIndex] = input;
	filter->luluInterimA[windowIndex] = input;

	int filterN = filter->N;
	int filterCount = filterN + 1;
	int filterWindow = filter->windowSize;

	const int indexer = (windowIndex - filterN + filterWindow) % filterWindow;

	float latestMinimum = filter->buf[indexer];

	for(int i = indexer + 1; i < filterCount + indexer; i++)
	{
		if(filter->buf[i % filterWindow] < latestMinimum)
		{
			latestMinimum = filter->buf[i % filterWindow];
		}
	}

	//Calculate the previous sequence minimum and store in "bufMin"
	filter->minBufA[windowIndex] = latestMinimum;
	//calculatePrevSequenceMaximum(filter->minBufA, windowIndex, filterCount, filterWindow);
	for(int i = 1; i < filterCount; i++)
	{
		int curIndex = (windowIndex + filterWindow - i) % filterWindow;
		if(filter->minBufA[curIndex] > latestMinimum)
		{
			latestMinimum = filter->minBufA[curIndex];
		}
	}

	filter->luluInterimA[indexer] = latestMinimum;

	filter->luluInterimB[0] = latestMinimum;
	for(int i = 1; i <= filterN; i++)
	{
		int curIndex = (indexer + filterWindow - i) % filterWindow;
		if(latestMinimum < filter->luluInterimA[curIndex])
		{
			latestMinimum = filter->luluInterimA[curIndex];
		}
		filter->luluInterimB[i] = latestMinimum;
	}
	//float luluMinN = calculatePrevSequenceMaximum(filter->luluInterimA, filter->windowBufIndex, filter->N + 1, filter->windowSize);

	//luluMinN = luluMinN < filter->luluInterimB[filter->N] ? luluMinN : filter->luluInterimB[filter->N];

	float bufMaxInterim = filter->luluInterimB[0];

	for(int i = filterN; i >= 0; i--)
	{
		float prevMaxVals = filter->luluInterimB[i];
		float curEvalFloat = filter->luluInterimA[indexer + (filterN - i) % filterWindow];
		bufMaxInterim = bufMaxInterim < curEvalFloat ? curEvalFloat : bufMaxInterim;
		float bufMaxN = bufMaxInterim;
		//Calculate the maximum of the L filtered values and store in maxBuf, remember to update the values that might have changed
//		bufMaxN = calculatePrevSequenceMaximum(filter->luluInterimA, index, filterCount - i, filterWindow);
		if(prevMaxVals > bufMaxN)
		{
			bufMaxN = prevMaxVals;
		}
		if(latestMinimum > bufMaxN)
		{
			latestMinimum = bufMaxN;
		}
	}

	if(windowIndex++ >= filterWindow) {
		filter->windowBufIndex = 0;
	} else {
		filter->windowBufIndex = windowIndex;
	}

	return latestMinimum;
}

FAST_CODE float luluFilterApply(luluFilter2_t *filter, float input)
{
	float resultA = luluFilterPartialApply(&filter->A, input);
	float resultB = luluFilterPartialApply(&filter->B, -input);
	return (resultA - resultB) / 2;
}