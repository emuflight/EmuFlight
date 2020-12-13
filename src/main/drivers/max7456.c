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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MAX7456

#include "build/debug.h"

#include "pg/max7456.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/vcd.h"

#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/max7456.h"
#include "drivers/max7456_symbols.h"
#include "drivers/nvic.h"
#include "drivers/time.h"


// DEBUG_MAX7456_SIGNAL
#define DEBUG_MAX7456_SIGNAL_MODEREG       0
#define DEBUG_MAX7456_SIGNAL_SENSE         1
#define DEBUG_MAX7456_SIGNAL_REINIT        2
#define DEBUG_MAX7456_SIGNAL_ROWS          3

// DEBUG_MAX7456_SPICLOCK
#define DEBUG_MAX7456_SPICLOCK_OVERCLOCK   0
#define DEBUG_MAX7456_SPICLOCK_DEVTYPE     1
#define DEBUG_MAX7456_SPICLOCK_DIVISOR     2

// VM0 bits
#define VIDEO_BUFFER_DISABLE        0x01
#define MAX7456_RESET               0x02
#define VERTICAL_SYNC_NEXT_VSYNC    0x04
#define OSD_ENABLE                  0x08

#define SYNC_MODE_AUTO              0x00
#define SYNC_MODE_INTERNAL          0x30
#define SYNC_MODE_EXTERNAL          0x20

#define VIDEO_MODE_PAL              0x40
#define VIDEO_MODE_NTSC             0x00
#define VIDEO_MODE_MASK             0x40
#define VIDEO_MODE_IS_PAL(val)      (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_PAL)
#define VIDEO_MODE_IS_NTSC(val)     (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_NTSC)

#define VIDEO_SIGNAL_DEBOUNCE_MS    100 // Time to wait for input to stabilize

// VM1 bits

// duty cycle is on_off
#define BLINK_DUTY_CYCLE_50_50 0x00
#define BLINK_DUTY_CYCLE_33_66 0x01
#define BLINK_DUTY_CYCLE_25_75 0x02
#define BLINK_DUTY_CYCLE_75_25 0x03

// blinking time
#define BLINK_TIME_0 0x00
#define BLINK_TIME_1 0x04
#define BLINK_TIME_2 0x08
#define BLINK_TIME_3 0x0C

// background mode brightness (percent)
#define BACKGROUND_BRIGHTNESS_0 0x00
#define BACKGROUND_BRIGHTNESS_7 0x01
#define BACKGROUND_BRIGHTNESS_14 0x02
#define BACKGROUND_BRIGHTNESS_21 0x03
#define BACKGROUND_BRIGHTNESS_28 0x04
#define BACKGROUND_BRIGHTNESS_35 0x05
#define BACKGROUND_BRIGHTNESS_42 0x06
#define BACKGROUND_BRIGHTNESS_49 0x07

#define BACKGROUND_MODE_GRAY 0x40

// STAT register bits

#define STAT_PAL      0x01
#define STAT_NTSC     0x02
#define STAT_LOS      0x04
#define STAT_NVR_BUSY 0x20

#define STAT_IS_PAL(val)  ((val) & STAT_PAL)
#define STAT_IS_NTSC(val) ((val) & STAT_NTSC)
#define STAT_IS_LOS(val)  ((val) & STAT_LOS)

#define VIN_IS_PAL(val)  (!STAT_IS_LOS(val) && STAT_IS_PAL(val))
#define VIN_IS_NTSC(val)  (!STAT_IS_LOS(val) && STAT_IS_NTSC(val))

// Kluege warning!
// There are occasions that NTSC is not detected even with !LOS (AB7456 specific?)
// When this happens, lower 3 bits of STAT register is read as zero.
// To cope with this case, this macro defines !LOS && !PAL as NTSC.
// Should be compatible with MAX7456 and non-problematic case.

#define VIN_IS_NTSC_alt(val)  (!STAT_IS_LOS(val) && !STAT_IS_PAL(val))

#define MAX7456_SIGNAL_CHECK_INTERVAL_MS 1000 // msec

// DMM special bits
#define CLEAR_DISPLAY 0x04
#define CLEAR_DISPLAY_VERT 0x06
#define INVERT_PIXEL_COLOR 0x08

// Special address for terminating incremental write
#define END_STRING 0xff

#define MAX7456ADD_READ         0x80
#define MAX7456ADD_VM0          0x00  //0b0011100// 00 // 00             ,0011100
#define MAX7456ADD_VM1          0x01
#define MAX7456ADD_HOS          0x02
#define MAX7456ADD_VOS          0x03
#define MAX7456ADD_DMM          0x04
#define MAX7456ADD_DMAH         0x05
#define MAX7456ADD_DMAL         0x06
#define MAX7456ADD_DMDI         0x07
#define MAX7456ADD_CMM          0x08
#define MAX7456ADD_CMAH         0x09
#define MAX7456ADD_CMAL         0x0a
#define MAX7456ADD_CMDI         0x0b
#define MAX7456ADD_OSDM         0x0c
#define MAX7456ADD_RB0          0x10
#define MAX7456ADD_RB1          0x11
#define MAX7456ADD_RB2          0x12
#define MAX7456ADD_RB3          0x13
#define MAX7456ADD_RB4          0x14
#define MAX7456ADD_RB5          0x15
#define MAX7456ADD_RB6          0x16
#define MAX7456ADD_RB7          0x17
#define MAX7456ADD_RB8          0x18
#define MAX7456ADD_RB9          0x19
#define MAX7456ADD_RB10         0x1a
#define MAX7456ADD_RB11         0x1b
#define MAX7456ADD_RB12         0x1c
#define MAX7456ADD_RB13         0x1d
#define MAX7456ADD_RB14         0x1e
#define MAX7456ADD_RB15         0x1f
#define MAX7456ADD_OSDBL        0x6c
#define MAX7456ADD_STAT         0xA0

#define NVM_RAM_SIZE            54
#define WRITE_NVR               0xA0

// Device type
#define MAX7456_DEVICE_TYPE_MAX 0
#define MAX7456_DEVICE_TYPE_AT  1

#define CHARS_PER_LINE      30 // XXX Should be related to VIDEO_BUFFER_CHARS_*?

// On shared SPI buss we want to change clock for OSD chip and restore for other devices.

#ifdef MAX7456_SPI_CLK
#define __spiBusTransactionBegin(busdev)        {spiSetDivisor((busdev)->busdev_u.spi.instance, max7456SpiClock);IOLo((busdev)->busdev_u.spi.csnPin);}
#else
#define __spiBusTransactionBegin(busdev)        IOLo((busdev)->busdev_u.spi.csnPin)
#endif

#ifdef MAX7456_RESTORE_CLK
#define __spiBusTransactionEnd(busdev)       {IOHi((busdev)->busdev_u.spi.csnPin);spiSetDivisor((busdev)->busdev_u.spi.instance, MAX7456_RESTORE_CLK);}
#else
#define __spiBusTransactionEnd(busdev)       IOHi((busdev)->busdev_u.spi.csnPin)
#endif

#ifndef MAX7456_SPI_CLK
#define MAX7456_SPI_CLK           (SPI_CLOCK_STANDARD)
#endif

busDevice_t max7456BusDevice;
busDevice_t *busdev = &max7456BusDevice;

static uint16_t max7456SpiClock = MAX7456_SPI_CLK;

uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

// We write everything in screenBuffer and then compare
// screenBuffer with shadowBuffer to upgrade only changed chars.
// This solution is faster then redrawing entire screen.

static uint8_t screenBuffer[VIDEO_BUFFER_CHARS_PAL + 40]; // For faster writes we use memcpy so we need some space to don't overwrite buffer
static uint8_t shadowBuffer[VIDEO_BUFFER_CHARS_PAL];

//Max chars to update in one idle

#define MAX_CHARS2UPDATE    100
#ifdef MAX7456_DMA_CHANNEL_TX
volatile bool dmaTransactionInProgress = false;
#endif

static uint8_t spiBuff[MAX_CHARS2UPDATE * 6];

static uint8_t  videoSignalCfg;
static uint8_t  videoSignalReg  = OSD_ENABLE; // OSD_ENABLE required to trigger first ReInit
static uint8_t  displayMemoryModeReg = 0;

static uint8_t  hosRegValue; // HOS (Horizontal offset register) value
static uint8_t  vosRegValue; // VOS (Vertical offset register) value

static bool max7456Lock         = false;
static bool fontIsLoading       = false;

static uint8_t max7456DeviceType;

static void max7456DrawScreenSlow(void);

static uint8_t max7456Send(uint8_t add, uint8_t data) {
    spiTransferByte(busdev->busdev_u.spi.instance, add);
#ifdef USE_NBD7456
    delayMicroseconds(10);
#endif
    return spiTransferByte(busdev->busdev_u.spi.instance, data);
}

#ifdef MAX7456_DMA_CHANNEL_TX
static void max7456SendDma(void* tx_buffer, void* rx_buffer, uint16_t buffer_size) {
    DMA_InitTypeDef DMA_InitStructure;
#ifdef MAX7456_DMA_CHANNEL_RX
    static uint16_t dummy[] = {0xffff};
#else
    UNUSED(rx_buffer);
#endif
    while (dmaTransactionInProgress); // Wait for prev DMA transaction
    DMA_DeInit(MAX7456_DMA_CHANNEL_TX);
#ifdef MAX7456_DMA_CHANNEL_RX
    DMA_DeInit(MAX7456_DMA_CHANNEL_RX);
#endif
    // Common to both channels
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(busdev->busdev_u.spi.instance->DR));
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_BufferSize = buffer_size;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
#ifdef MAX7456_DMA_CHANNEL_RX
    // Rx Channel
#ifdef STM32F4
    DMA_InitStructure.DMA_Memory0BaseAddr = rx_buffer ? (uint32_t)rx_buffer : (uint32_t)(dummy);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
#else
    DMA_InitStructure.DMA_MemoryBaseAddr = rx_buffer ? (uint32_t)rx_buffer : (uint32_t)(dummy);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
#endif
    DMA_InitStructure.DMA_MemoryInc = rx_buffer ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_Init(MAX7456_DMA_CHANNEL_RX, &DMA_InitStructure);
    DMA_Cmd(MAX7456_DMA_CHANNEL_RX, ENABLE);
#endif
    // Tx channel
#ifdef STM32F4
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tx_buffer; //max7456_screen;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
#else
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)tx_buffer; //max7456_screen;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
#endif
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_Init(MAX7456_DMA_CHANNEL_TX, &DMA_InitStructure);
    DMA_Cmd(MAX7456_DMA_CHANNEL_TX, ENABLE);
#ifdef MAX7456_DMA_CHANNEL_RX
    DMA_ITConfig(MAX7456_DMA_CHANNEL_RX, DMA_IT_TC, ENABLE);
#else
    DMA_ITConfig(MAX7456_DMA_CHANNEL_TX, DMA_IT_TC, ENABLE);
#endif
    // Enable SPI TX/RX request
    __spiBusTransactionBegin(busdev);
    dmaTransactionInProgress = true;
    SPI_I2S_DMACmd(busdev->busdev_u.spi.instance,
#ifdef MAX7456_DMA_CHANNEL_RX
                   SPI_I2S_DMAReq_Rx |
#endif
                   SPI_I2S_DMAReq_Tx, ENABLE);
}

void max7456_dma_irq_handler(dmaChannelDescriptor_t* descriptor) {
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
#ifdef MAX7456_DMA_CHANNEL_RX
        DMA_Cmd(MAX7456_DMA_CHANNEL_RX, DISABLE);
#endif
        // Make sure SPI DMA transfer is complete
        while (SPI_I2S_GetFlagStatus (busdev->busdev_u.spi.instance, SPI_I2S_FLAG_TXE) == RESET) {};
        while (SPI_I2S_GetFlagStatus (busdev->busdev_u.spi.instance, SPI_I2S_FLAG_BSY) == SET) {};
        // Empty RX buffer. RX DMA takes care of it if enabled.
        // This should be done after transmission finish!!!
        while (SPI_I2S_GetFlagStatus(busdev->busdev_u.spi.instance, SPI_I2S_FLAG_RXNE) == SET) {
            busdev->busdev_u.spi.instance->DR;
        }
        DMA_Cmd(MAX7456_DMA_CHANNEL_TX, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        SPI_I2S_DMACmd(busdev->busdev_u.spi.instance,
#ifdef MAX7456_DMA_CHANNEL_RX
                       SPI_I2S_DMAReq_Rx |
#endif
                       SPI_I2S_DMAReq_Tx, DISABLE);
        __spiBusTransactionEnd(busdev);
        dmaTransactionInProgress = false;
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_HTIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    }
}

#endif

uint8_t max7456GetRowsCount(void) {
    return (videoSignalReg & VIDEO_MODE_PAL) ? VIDEO_LINES_PAL : VIDEO_LINES_NTSC;
}

void max7456ReInit(void) {
    uint8_t srdata = 0;
    static bool firstInit = true;
    __spiBusTransactionBegin(busdev);
    switch (videoSignalCfg) {
    case VIDEO_SYSTEM_PAL:
        videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        break;
    case VIDEO_SYSTEM_NTSC:
        videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
        break;
    case VIDEO_SYSTEM_AUTO:
        srdata = max7456Send(MAX7456ADD_STAT, 0x00);
        if (VIN_IS_NTSC(srdata)) {
            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
        } else if (VIN_IS_PAL(srdata)) {
            videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        } else {
            // No valid input signal, fallback to default (XXX NTSC for now)
            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
        }
        break;
    }
    if (videoSignalReg & VIDEO_MODE_PAL) { //PAL
        maxScreenSize = VIDEO_BUFFER_CHARS_PAL;
    } else {              // NTSC
        maxScreenSize = VIDEO_BUFFER_CHARS_NTSC;
    }
    // Set all rows to same charactor black/white level
    max7456Brightness(0, 2);
    // Re-enable MAX7456 (last function call disables it)
    __spiBusTransactionBegin(busdev);
    // Make sure the Max7456 is enabled
    max7456Send(MAX7456ADD_VM0, videoSignalReg);
    max7456Send(MAX7456ADD_HOS, hosRegValue);
    max7456Send(MAX7456ADD_VOS, vosRegValue);
    max7456Send(MAX7456ADD_DMM, displayMemoryModeReg | CLEAR_DISPLAY);
    __spiBusTransactionEnd(busdev);
    // Clear shadow to force redraw all screen in non-dma mode.
    memset(shadowBuffer, 0, maxScreenSize);
    if (firstInit) {
        max7456DrawScreenSlow();
        firstInit = false;
    }
}


// Here we init only CS and try to init MAX for first time.
// Also detect device type (MAX v.s. AT)

bool max7456Init(const max7456Config_t *max7456Config, const vcdProfile_t *pVcdProfile, bool cpuOverclock) {
    max7456HardwareReset();
    if (!max7456Config->csTag) {
        return false;
    }
    busdev->busdev_u.spi.csnPin = IOGetByTag(max7456Config->csTag);
    if (!IOIsFreeOrPreinit(busdev->busdev_u.spi.csnPin)) {
        return false;
    }
    IOInit(busdev->busdev_u.spi.csnPin, OWNER_OSD_CS, 0);
    IOConfigGPIO(busdev->busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(busdev->busdev_u.spi.csnPin);
    spiBusSetInstance(busdev, spiInstanceByDevice(SPI_CFG_TO_DEV(max7456Config->spiDevice)));
    // Detect device type by writing and reading CA[8] bit at CMAL[6].
    // Do this at half the speed for safety.
    spiSetDivisor(busdev->busdev_u.spi.instance, MAX7456_SPI_CLK * 2);
    max7456Send(MAX7456ADD_CMAL, (1 << 6)); // CA[8] bit
    if (max7456Send(MAX7456ADD_CMAL | MAX7456ADD_READ, 0xff) & (1 << 6)) {
        max7456DeviceType = MAX7456_DEVICE_TYPE_AT;
    } else {
        max7456DeviceType = MAX7456_DEVICE_TYPE_MAX;
    }
#if defined(USE_OVERCLOCK)
    // Determine SPI clock divisor based on config and the device type.
    switch (max7456Config->clockConfig) {
    case MAX7456_CLOCK_CONFIG_HALF:
        max7456SpiClock = MAX7456_SPI_CLK * 2;
        break;
    case MAX7456_CLOCK_CONFIG_OC:
        max7456SpiClock = (cpuOverclock && (max7456DeviceType == MAX7456_DEVICE_TYPE_MAX)) ? MAX7456_SPI_CLK * 2 : MAX7456_SPI_CLK;
        break;
    case MAX7456_CLOCK_CONFIG_FULL:
        max7456SpiClock = MAX7456_SPI_CLK;
        break;
    }
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_OVERCLOCK, cpuOverclock);
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_DEVTYPE, max7456DeviceType);
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_DIVISOR, max7456SpiClock);
#else
    UNUSED(max7456Config);
    UNUSED(cpuOverclock);
#endif
    spiSetDivisor(busdev->busdev_u.spi.instance, max7456SpiClock);
    // force soft reset on Max7456
    __spiBusTransactionBegin(busdev);
    max7456Send(MAX7456ADD_VM0, MAX7456_RESET);
    __spiBusTransactionEnd(busdev);
    // Setup values to write to registers
    videoSignalCfg = pVcdProfile->video_system;
    hosRegValue = 32 - pVcdProfile->h_offset;
    vosRegValue = 16 - pVcdProfile->v_offset;
#ifdef MAX7456_DMA_CHANNEL_TX
    dmaSetHandler(MAX7456_DMA_IRQ_HANDLER_ID, max7456_dma_irq_handler, NVIC_PRIO_MAX7456_DMA, 0);
#endif
    // Real init will be made later when driver detect idle.
    return true;
}

/**
 * Sets inversion of black and white pixels.
 */
void max7456Invert(bool invert) {
    if (invert) {
        displayMemoryModeReg |= INVERT_PIXEL_COLOR;
    } else {
        displayMemoryModeReg &= ~INVERT_PIXEL_COLOR;
    }
    __spiBusTransactionBegin(busdev);
    max7456Send(MAX7456ADD_DMM, displayMemoryModeReg);
    __spiBusTransactionEnd(busdev);
}

/**
 * Sets the brighness of black and white pixels.
 *
 * @param black Black brightness (0-3, 0 is darkest)
 * @param white White brightness (0-3, 0 is darkest)
 */
void max7456Brightness(uint8_t black, uint8_t white) {
    const uint8_t reg = (black << 2) | (3 - white);
    __spiBusTransactionBegin(busdev);
    for (int i = MAX7456ADD_RB0; i <= MAX7456ADD_RB15; i++) {
        max7456Send(i, reg);
    }
    __spiBusTransactionEnd(busdev);
}

//just fill with spaces with some tricks
void max7456ClearScreen(void) {
    memset(screenBuffer, 0x20, VIDEO_BUFFER_CHARS_PAL);
}

uint8_t* max7456GetScreenBuffer(void) {
    return screenBuffer;
}

void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c) {
    screenBuffer[y * CHARS_PER_LINE + x] = c;
}

void max7456Write(uint8_t x, uint8_t y, const char *buff) {
    for (int i = 0; * (buff + i); i++) {
        if (x + i < CHARS_PER_LINE) { // Do not write over screen
            screenBuffer[y * CHARS_PER_LINE + x + i] = *(buff + i);
        }
    }
}

bool max7456DmaInProgress(void) {
#ifdef MAX7456_DMA_CHANNEL_TX
    return dmaTransactionInProgress;
#else
    return false;
#endif
}

bool max7456BuffersSynced(void) {
    for (int i = 0; i < maxScreenSize; i++) {
        if (screenBuffer[i] != shadowBuffer[i]) {
            return false;
        }
    }
    return true;
}

void max7456ReInitIfRequired(void) {
    static uint32_t lastSigCheckMs = 0;
    static uint32_t videoDetectTimeMs = 0;
    static uint16_t reInitCount = 0;
    __spiBusTransactionBegin(busdev);
    const uint8_t stallCheck = max7456Send(MAX7456ADD_VM0 | MAX7456ADD_READ, 0x00);
    __spiBusTransactionEnd(busdev);
    const timeMs_t nowMs = millis();
    if (stallCheck != videoSignalReg) {
        max7456ReInit();
    } else if ((videoSignalCfg == VIDEO_SYSTEM_AUTO)
               && ((nowMs - lastSigCheckMs) > MAX7456_SIGNAL_CHECK_INTERVAL_MS)) {
        // Adjust output format based on the current input format.
        __spiBusTransactionBegin(busdev);
        const uint8_t videoSense = max7456Send(MAX7456ADD_STAT, 0x00);
        __spiBusTransactionEnd(busdev);
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_MODEREG, videoSignalReg & VIDEO_MODE_MASK);
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_SENSE, videoSense & 0x7);
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_ROWS, max7456GetRowsCount());
        if (videoSense & STAT_LOS) {
            videoDetectTimeMs = 0;
        } else {
            if ((VIN_IS_PAL(videoSense) && VIDEO_MODE_IS_NTSC(videoSignalReg))
                    || (VIN_IS_NTSC_alt(videoSense) && VIDEO_MODE_IS_PAL(videoSignalReg))) {
                if (videoDetectTimeMs) {
                    if (millis() - videoDetectTimeMs > VIDEO_SIGNAL_DEBOUNCE_MS) {
                        max7456ReInit();
                        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_REINIT, ++reInitCount);
                    }
                } else {
                    // Wait for signal to stabilize
                    videoDetectTimeMs = millis();
                }
            }
        }
        lastSigCheckMs = nowMs;
    }
    //------------   end of (re)init-------------------------------------
}

void max7456DrawScreen(void) {
    static uint16_t pos = 0;
    if (!max7456Lock && !fontIsLoading) {
        // (Re)Initialize MAX7456 at startup or stall is detected.
        max7456Lock = true;
#ifndef USE_NBD7456
        max7456ReInitIfRequired();
#endif
        int buff_len = 0;
        for (int k = 0; k < MAX_CHARS2UPDATE; k++) {
            if (screenBuffer[pos] != shadowBuffer[pos]) {
                spiBuff[buff_len++] = MAX7456ADD_DMAH;
                spiBuff[buff_len++] = pos >> 8;
                spiBuff[buff_len++] = MAX7456ADD_DMAL;
                spiBuff[buff_len++] = pos & 0xff;
                spiBuff[buff_len++] = MAX7456ADD_DMDI;
                spiBuff[buff_len++] = screenBuffer[pos];
                shadowBuffer[pos] = screenBuffer[pos];
            }
            if (++pos >= maxScreenSize) {
                pos = 0;
                break;
            }
        }
        if (buff_len) {
#ifdef MAX7456_DMA_CHANNEL_TX
            max7456SendDma(spiBuff, NULL, buff_len);
#else
            __spiBusTransactionBegin(busdev);
#ifdef USE_NBD7456
            for(int k = 0; k < buff_len; k++) {
                delayMicroseconds(5);
                spiTransferByte(busdev->busdev_u.spi.instance, spiBuff[k]);
            }
#else // USE_NBD7456
            spiTransfer(busdev->busdev_u.spi.instance, spiBuff, NULL, buff_len);
#endif
            __spiBusTransactionEnd(busdev);
#endif // MAX7456_DMA_CHANNEL_TX
        }
        max7456Lock = false;
    }
}

static void max7456DrawScreenSlow(void) {
    bool escapeCharFound = false;
    __spiBusTransactionBegin(busdev);
    // Enable auto-increment mode and update every character in the screenBuffer.
    // The "escape" character 0xFF must be skipped as it causes the MAX7456 to exit auto-increment mode.
    max7456Send(MAX7456ADD_DMAH, 0);
    max7456Send(MAX7456ADD_DMAL, 0);
    max7456Send(MAX7456ADD_DMM, displayMemoryModeReg | 1);
    for (int xx = 0; xx < maxScreenSize; xx++) {
        if (screenBuffer[xx] == END_STRING) {
            escapeCharFound = true;
            max7456Send(MAX7456ADD_DMDI, ' ');  // replace the 0xFF character with a blank in the first pass to avoid terminating auto-increment
        } else {
            max7456Send(MAX7456ADD_DMDI, screenBuffer[xx]);
        }
        shadowBuffer[xx] = screenBuffer[xx];
    }
    max7456Send(MAX7456ADD_DMDI, END_STRING);
    max7456Send(MAX7456ADD_DMM, displayMemoryModeReg);
    // If we found any of the "escape" character 0xFF, then make a second pass
    // to update them with direct addressing
    if (escapeCharFound) {
        for (int xx = 0; xx < maxScreenSize; xx++) {
            if (screenBuffer[xx] == END_STRING) {
                max7456Send(MAX7456ADD_DMAH, xx >> 8);
                max7456Send(MAX7456ADD_DMAL, xx & 0xFF);
                max7456Send(MAX7456ADD_DMDI, END_STRING);
            }
        }
    }
    __spiBusTransactionEnd(busdev);
}


// should not be used when armed
void max7456RefreshAll(void) {
    if (!max7456Lock) {
#ifdef MAX7456_DMA_CHANNEL_TX
        while (dmaTransactionInProgress);
#endif
        max7456Lock = true;
        max7456ReInitIfRequired();
        max7456DrawScreenSlow();
        max7456Lock = false;
    }
}

void max7456WriteNvm(uint8_t char_address, const uint8_t *font_data) {
#ifdef MAX7456_DMA_CHANNEL_TX
    while (dmaTransactionInProgress);
#endif
    while (max7456Lock);
    max7456Lock = true;
    __spiBusTransactionBegin(busdev);
    // disable display
    fontIsLoading = true;
    max7456Send(MAX7456ADD_VM0, 0);
    max7456Send(MAX7456ADD_CMAH, char_address); // set start address high
    for (int x = 0; x < 54; x++) {
        max7456Send(MAX7456ADD_CMAL, x); //set start address low
        max7456Send(MAX7456ADD_CMDI, font_data[x]);
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        LED1_TOGGLE;
#endif
    }
    // Transfer 54 bytes from shadow ram to NVM
    max7456Send(MAX7456ADD_CMM, WRITE_NVR);
    // Wait until bit 5 in the status register returns to 0 (12ms)
    while ((max7456Send(MAX7456ADD_STAT, 0x00) & STAT_NVR_BUSY) != 0x00) {
#ifdef USE_NBD7456
        delay(10);
#endif
    }
    __spiBusTransactionEnd(busdev);
    max7456Lock = false;
}

#ifdef MAX7456_NRST_PIN
static IO_t max7456ResetPin        = IO_NONE;
#endif

void max7456HardwareReset(void) {
#ifdef MAX7456_NRST_PIN
#define IO_RESET_CFG      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)
    max7456ResetPin = IOGetByTag(IO_TAG(MAX7456_NRST_PIN));
    IOInit(max7456ResetPin, OWNER_OSD, 0);
    IOConfigGPIO(max7456ResetPin, IO_RESET_CFG);
    // RESET
    IOLo(max7456ResetPin);
    delay(100);
    IOHi(max7456ResetPin);
#endif
}

#endif // USE_MAX7456
