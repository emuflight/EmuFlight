#include "build/atomic.h"

struct barrierTrace {
    int enter, leave;
};

// Cleanup function for atomic barrier testing
static void atomic_test_barrier_cleanup(struct barrierTrace **barrier_ptr) {
    if (barrier_ptr && *barrier_ptr) {
        (*barrier_ptr)->leave++;
    }
}

// Macro helper to create unique identifier for each use
#define atomic_test_CONCAT(a, b) a ## b
#define atomic_test_MAKE_UNIQUE(x, line) atomic_test_CONCAT(x, line)
#define ATOMIC_TEST_UNIQUE atomic_test_MAKE_UNIQUE(atomic_test_barrier_, __LINE__)

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
#if defined(__clang__) || (defined(__has_attribute) && __has_attribute(cleanup))
#define ATOMIC_BARRIER(data) \
    struct barrierTrace *__attribute__((cleanup(atomic_test_barrier_cleanup))) ATOMIC_TEST_UNIQUE = &(data); \
    ATOMIC_BARRIER_ENTER(ATOMIC_TEST_UNIQUE, #data)
#else
// Fallback for compilers lacking cleanup support
#define ATOMIC_BARRIER(data) \
    ATOMIC_BARRIER_ENTER(&(data), #data)
#endif
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
#undef ATOMIC_BARRIER
}
