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
#include <stdbool.h>
#include <string.h>

#include "bitarray.h"

#define BITARRAY_BIT_OP(array, bit, op) ((array)[(bit) / (sizeof((array)[0]) * 8)] op (1 << ((bit) % (sizeof((array)[0]) * 8))))

bool bitArrayGet(const void *array, unsigned bit) {
    return BITARRAY_BIT_OP((uint32_t*)array, bit, &);
}

void bitArraySet(void *array, unsigned bit) {
    BITARRAY_BIT_OP((uint32_t*)array, bit, |= );
}

void bitArrayClr(void *array, unsigned bit) {
    BITARRAY_BIT_OP((uint32_t*)array, bit, &= ~);
}

void bitArrayXor(void *dest, size_t size, void *op1, void *op2) {
    for (size_t i = 0; i < size; i++) {
        ((uint8_t*)dest)[i] = ((uint8_t*)op1)[i] ^ ((uint8_t*)op2)[i];
    }
}

void bitArrayCopy(void *array, unsigned from, unsigned to) {
    if (bitArrayGet(array, from)) {
        bitArraySet(array, to);
    } else {
        bitArrayClr(array, to);
    }
}

void bitArrayClrAll(bitarrayElement_t *array, size_t size)
{
    memset(array, 0, size);
}

__attribute__((always_inline)) static inline uint8_t __CTZ(uint32_t val)
{
    // __builtin_ctz is not defined for zero, since it's arch
    // dependant. However, in ARM it gets translated to a
    // rbit and then a clz, making it return 32 for zero on ARM.
    // For other architectures, explicitely implement the same
    // semantics.
#ifdef __arm__
    uint8_t zc;
    __asm__ volatile ("rbit %1, %1\n\t"
                      "clz %0, %1"
                    : "=r" (zc)
                    : "r" (val) );
    return zc;
#else
    // __builtin_clz is not defined for zero, since it's arch
    // dependant. Make it return 32 like ARM's CLZ.
    return val ? __builtin_ctz(val) : 32;
#endif
}

int bitArrayFindFirstSet(const bitarrayElement_t *array, unsigned start, size_t size)
{
    const uint32_t *ptr = (uint32_t*)array;
    const uint32_t *end = ptr + (size / 4);
    const uint32_t *p = ptr + start / (8 * 4);

    if (p < end) {
        int ret;
        // First iteration might need to mask some bits
        uint32_t mask = 0xFFFFFFFF << (start % (8 * 4));
        if ((ret = __CTZ(*p & mask)) != 32) {
            return (((char *)p) - ((char *)ptr)) * 8 + ret;
        }
        p++;
        while (p < end) {
            if ((ret = __CTZ(*p)) != 32) {
                return (((char *)p) - ((char *)ptr)) * 8 + ret;
            }
            p++;
        }
    }
    return -1;
}