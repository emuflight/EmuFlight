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

// Mirrors sdioDmaClaim() (drivers/sdio_f4xx.c, drivers/sdio_f7xx.c -- byte-identical body,
// confirmed by direct read) rather than compiling those files: they need 118 direct
// SDIO->/RCC-> field accesses, real GPIO AF config, and NVIC -- the same disproportionate-
// mocking case bus_spi_ll.c/bus_spi_stdperiph.c already were in feat/dma-ll-unittest-infra
// (20+ mocks needed for functions that weren't even the test target). Follows the same
// mirror-with-fake-DMA-state convention already established for uartDmaClaim() in
// serial_uart_dma_claim_unittest.cc.
//
// SD_Initialize_LL() (SDIO mode) can be called from both sdcard_sdio_baremetal.c (boot) and
// usbd_storage_sdio.c (USB MSC passthrough). The second call is a legitimate reopen of the
// same OWNER_SDCARD claim, not a conflict, and must not be misidentified as one now that the
// caller-side return check exists.
//
// sdcard.c's SPI-mode driver has no DMA claim of its own -- DMA ownership for that path is
// resolved once at the bus level by spiInitBusDMA(), covered by bus_spi_unittest.cc.
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

bool sdioDmaClaim(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex) {
    if (fakeDmaGetOwner(identifier) == owner && fakeDmaGetResourceIndex(identifier) == resourceIndex) {
        return true;
    }
    return fakeDmaAllocate(identifier, owner, resourceIndex);
}

const dmaIdentifier_e kStream = DMA1_ST3_HANDLER;

}  // namespace

// sdioDmaClaim() -- SDIO-mode (drivers/sdio_f4xx.c, drivers/sdio_f7xx.c), same real callers
// always pass OWNER_SDCARD / index 0.

TEST(SdioDmaClaimUnittest, FirstClaimOnFreeStreamSucceeds) {
    resetFakeDmaState();
    EXPECT_TRUE(sdioDmaClaim(kStream, OWNER_SDCARD, 0));
    EXPECT_EQ(fakeOwner, OWNER_SDCARD);
}

TEST(SdioDmaClaimUnittest, UsbMscReopenBySameOwnerAndIndexSucceeds) {
    resetFakeDmaState();
    fakeOwner = OWNER_SDCARD;
    fakeResourceIndex = 0;
    // SD_Initialize_LL() called again from usbd_storage_sdio.c after sdcard_sdio_baremetal.c
    // already claimed it during boot.
    EXPECT_TRUE(sdioDmaClaim(kStream, OWNER_SDCARD, 0));
}

TEST(SdioDmaClaimUnittest, ConflictWithForeignOwnerFails) {
    resetFakeDmaState();
    fakeOwner = OWNER_SPI_SDI;
    fakeResourceIndex = 0;
    EXPECT_FALSE(sdioDmaClaim(kStream, OWNER_SDCARD, 0));
    EXPECT_EQ(fakeOwner, OWNER_SPI_SDI);
}

TEST(SdioDmaClaimUnittest, SameOwnerDifferentResourceIndexFails) {
    resetFakeDmaState();
    fakeOwner = OWNER_SDCARD;
    fakeResourceIndex = 1;
    EXPECT_FALSE(sdioDmaClaim(kStream, OWNER_SDCARD, 0));
}
