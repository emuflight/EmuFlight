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

#pragma once

typedef uint32_t bitarrayElement_t;

bool bitArrayGet(const void *array, unsigned bit);
void bitArraySet(void *array, unsigned bit);
void bitArrayClr(void *array, unsigned bit);
void bitArrayXor(void *dest, size_t size, void *op1, void *op2);
void bitArrayCopy(void *array, unsigned from, unsigned to);
void bitArrayClrAll(bitarrayElement_t *array, size_t size);
// Returns the first set bit with pos >= start_bit, or -1 if all bits
// are zero. Note that size must indicate the size of array in bytes.
// In most cases, you should use the BITARRAY_FIND_FIRST_SET() macro
// to call this function.
int bitArrayFindFirstSet(const bitarrayElement_t *array, unsigned start_bit, size_t size);

#define BITARRAY_DECLARE(name, bits) bitarrayElement_t name[(bits + 31) / 32]
#define BITARRAY_SET_ALL(array) bitArraySetAll(array, sizeof(array))
#define BITARRAY_CLR_ALL(array) bitArrayClrAll(array, sizeof(array))
#define BITARRAY_FIND_FIRST_SET(array, start_bit) bitArrayFindFirstSet(array, start_bit, sizeof(array))