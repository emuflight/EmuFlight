#include "build/atomic.h"

struct barrierTrace {
    int enter, leave;
};

// Cleanup function for atomic barrier testing
static void __atomic_barrier_cleanup(struct barrierTrace **barrier_ptr) {
    if (barrier_ptr && *barrier_ptr) {
        (*barrier_ptr)->leave++;
    }
}

// Macro helper to create unique identifier for each use
#define __ATOMIC_CONCAT(a, b) a ## b
#define __ATOMIC_MAKE_UNIQUE(x, line) __ATOMIC_CONCAT(x, line)
#define __UNIQUE __ATOMIC_MAKE_UNIQUE(__barrier_, __LINE__)

int testAtomicBarrier_C(struct barrierTrace *b0, struct barrierTrace *b1, struct barrierTrace sample[][2]) {
    int sIdx = 0;
// replace barrier macros to track barrier invocation
// pass known struct as barrier variable, keep track inside it
#undef ATOMIC_BARRIER_ENTER
#undef ATOMIC_BARRIER_LEAVE
#undef ATOMIC_BARRIER
#define ATOMIC_BARRIER_ENTER(ptr, refStr) do {(ptr)->enter++; } while(0)
#define ATOMIC_BARRIER_LEAVE(ptr, refStr) do {(ptr)->leave++; } while(0)
// For C code, use __cleanup__ to track when we leave scope
#define ATOMIC_BARRIER(data) \
    struct barrierTrace *__attribute__((cleanup(__atomic_barrier_cleanup))) __UNIQUE = &(data); \
    ATOMIC_BARRIER_ENTER(__UNIQUE, #data); \
    do {} while(0)
    b0->enter = 0;
    b0->leave = 0;
    b1->enter = 0;
    b1->leave = 0;
    sample[sIdx][0] = *b0;
    sample[sIdx][1] = *b1;
    sIdx++;
    do {
        ATOMIC_BARRIER(*b0);
        ATOMIC_BARRIER(*b1);
        sample[sIdx][0] = *b0;
        sample[sIdx][1] = *b1;
        sIdx++;
        do {
            ATOMIC_BARRIER(*b0);
            sample[sIdx][0] = *b0;
            sample[sIdx][1] = *b1;
            sIdx++;
        } while(0);
        sample[sIdx][0] = *b0;
        sample[sIdx][1] = *b1;
        sIdx++;
    } while(0);
    sample[sIdx][0] = *b0;
    sample[sIdx][1] = *b1;
    sIdx++;
    return sIdx;
// ATOMIC_BARRIER is broken in rest of this file
#undef ATOMIC_BARRIER_ENTER
#undef ATOMIC_BARRIER_LEAVE
}
