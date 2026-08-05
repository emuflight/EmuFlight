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

extern "C" {

#include "platform.h"
#include "drivers/dma.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Mirrors the same-owner reclaim guard duplicated in serial_uart_stm32f4xx.c / serial_uart_stm32f7xx.c.
// uartOpen() re-invokes serialUART() on every reopen, so a stream already held by this same
// owner+resourceIndex must be re-claimable without going through dmaAllocate()'s OWNER_FREE check.
namespace {

resourceOwner_e fakeOwner = OWNER_FREE;
uint8_t fakeResourceIndex = 0;

void resetFakeDmaState() {
    fakeOwner = OWNER_FREE;
    fakeResourceIndex = 0;
}

resourceOwner_e fakeDmaGetOwner(dmaIdentifier_e) {
    return fakeOwner;
}

uint8_t fakeDmaGetResourceIndex(dmaIdentifier_e) {
    return fakeResourceIndex;
}

bool fakeDmaAllocate(dmaIdentifier_e, resourceOwner_e owner, uint8_t resourceIndex) {
    if (fakeOwner != OWNER_FREE) {
        return false;
    }
    fakeOwner = owner;
    fakeResourceIndex = resourceIndex;
    return true;
}

bool uartDmaClaim(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex) {
    if (fakeDmaGetOwner(identifier) == owner && fakeDmaGetResourceIndex(identifier) == resourceIndex) {
        return true;
    }
    return fakeDmaAllocate(identifier, owner, resourceIndex);
}

const dmaIdentifier_e kStream = DMA1_ST3_HANDLER;

}  // namespace

TEST(SerialUartDmaClaimUnittest, FirstClaimOnFreeStreamSucceeds) {
    resetFakeDmaState();
    EXPECT_TRUE(uartDmaClaim(kStream, OWNER_SERIAL_TX, RESOURCE_INDEX(2)));
    EXPECT_EQ(fakeOwner, OWNER_SERIAL_TX);
}

TEST(SerialUartDmaClaimUnittest, ReopenBySameOwnerAndIndexSucceeds) {
    resetFakeDmaState();
    fakeOwner = OWNER_SERIAL_RX;
    fakeResourceIndex = RESOURCE_INDEX(4);
    EXPECT_TRUE(uartDmaClaim(kStream, OWNER_SERIAL_RX, RESOURCE_INDEX(4)));
}

TEST(SerialUartDmaClaimUnittest, ConflictWithForeignOwnerFails) {
    resetFakeDmaState();
    fakeOwner = OWNER_SPI_SDI;
    fakeResourceIndex = RESOURCE_INDEX(1);
    EXPECT_FALSE(uartDmaClaim(kStream, OWNER_SERIAL_RX, RESOURCE_INDEX(1)));
    // failed claim must not disturb the existing owner's bookkeeping.
    EXPECT_EQ(fakeOwner, OWNER_SPI_SDI);
}

TEST(SerialUartDmaClaimUnittest, SameOwnerEnumDifferentResourceIndexFails) {
    resetFakeDmaState();
    fakeOwner = OWNER_SERIAL_RX;
    fakeResourceIndex = RESOURCE_INDEX(1);
    // same owner value but a different UART device instance: must not be treated as a reopen.
    EXPECT_FALSE(uartDmaClaim(kStream, OWNER_SERIAL_RX, RESOURCE_INDEX(2)));
}
