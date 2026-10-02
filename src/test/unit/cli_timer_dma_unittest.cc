/*
 * This file is part of EmuFlight.
 *
 * EmuFlight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * EmuFlight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with EmuFlight.  If not, see <http://www.gnu.org/licenses/>.
 */

// Host tests for the `timer` and `dma` CLI commands, built with USE_TIMER_MGMT (IT #1482).
//
// The commands run through the real cliProcess() line parser: each test feeds a command line
// through the stubbed serial port and checks the printed text. Expected strings come from the
// command specification (IT #1482, IT #1467), not from a recording of earlier output.
//
// Not verifiable here: real DMA stream and timer ownership, DMAMUX/channel routing, hardware
// timing. Those need HELIOSPRING, FOXEERF722V4 and STELLARH7DEV.

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <limits.h>
#include <math.h>
#include <string>

extern "C" {
    #include "platform.h"
    #include "target.h"
    #include "build/version.h"
    #include "config/feature.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/adc.h"
    #include "pg/rx.h"
    #include "pg/timerio.h"
    #include "drivers/buf_writer.h"
    #include "drivers/dma.h"
    #include "drivers/dma_reqmap.h"
    #include "drivers/timer.h"
    #include "drivers/serial.h"
    #include "drivers/serial_uart.h"
    #include "pg/serial_uart.h"
    #include "drivers/io.h"
    #include "drivers/io_impl.h"
    #include "drivers/system.h"
    #include "drivers/timer.h"
    #include "drivers/vtx_common.h"
    #include "fc/config.h"
    #include "fc/rc_adjustments.h"
    #include "fc/runtime_config.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/servos.h"
    #include "interface/cli.h"
    #include "interface/msp.h"
    #include "interface/msp_box.h"
    #include "interface/settings.h"
    #include "io/beeper.h"
    #include "io/ledstrip.h"
    #include "io/osd.h"
    #include "io/serial.h"
    #include "io/vtx_control.h"
    #include "io/vtx.h"
    #include "pg/beeper.h"
    #include "rx/rx.h"
    #include "scheduler/scheduler.h"
    #include "sensors/battery.h"

    const clivalue_t valueTable[] = {};
    const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);
    const lookupTableEntry_t lookupTables[] = {};

    PG_REGISTER(osdConfig_t, osdConfig, PG_OSD_CONFIG, 0);
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(ledStripConfig_t, ledStripConfig, PG_LED_STRIP_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 0);
    PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 0);
    PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 0);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);
    PG_REGISTER_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);
    PG_REGISTER_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers, PG_SERVO_MIXER, 0);
    PG_REGISTER(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);
    PG_REGISTER_ARRAY(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);
    PG_REGISTER_ARRAY(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
    PG_REGISTER(pidConfig_t, pidConfig, PG_PID_CONFIG, 0);
    PG_REGISTER(vtxConfig_t, vtxConfig, PG_VTX_CONFIG, 1);

    // printResource() (called by dump/diff) dereferences the PG of every resourceTable row, so each
    // PG that table reaches must exist. The ioTag fields inside are all zero ("resource ... NONE").
    typedef struct fakeResourcePg_s {
        uint8_t bytes[512];
    } fakeResourcePg_t;
#define FAKE_RESOURCE_PG(name, pgn) PG_REGISTER(fakeResourcePg_t, name, pgn, 0)
#ifdef USE_BEEPER
    FAKE_RESOURCE_PG(fakeBeeperDevConfig, PG_BEEPER_DEV_CONFIG);
#endif
    FAKE_RESOURCE_PG(fakeMotorConfig, PG_MOTOR_CONFIG);
#ifdef USE_SERVOS
    FAKE_RESOURCE_PG(fakeServoConfig, PG_SERVO_CONFIG);
#endif
#if defined(USE_PPM)
    FAKE_RESOURCE_PG(fakePpmConfig, PG_PPM_CONFIG);
#endif
#if defined(USE_PWM)
    FAKE_RESOURCE_PG(fakePwmConfig, PG_PWM_CONFIG);
#endif
#ifdef USE_RANGEFINDER_HCSR04
    FAKE_RESOURCE_PG(fakeSonarConfig, PG_SONAR_CONFIG);
#endif
    FAKE_RESOURCE_PG(fakeSerialPinConfig, PG_SERIAL_PIN_CONFIG);
#ifdef USE_I2C
    FAKE_RESOURCE_PG(fakeI2cConfig, PG_I2C_CONFIG);
#endif
    FAKE_RESOURCE_PG(fakeStatusLedConfig, PG_STATUS_LED_CONFIG);
#ifdef USE_TRANSPONDER
    FAKE_RESOURCE_PG(fakeTransponderConfig, PG_TRANSPONDER_CONFIG);
#endif
#ifdef USE_SPI
    FAKE_RESOURCE_PG(fakeSpiPinConfig, PG_SPI_PIN_CONFIG);
    FAKE_RESOURCE_PG(fakeSpiPreinitIpuConfig, PG_SPI_PREINIT_IPU_CONFIG);
    FAKE_RESOURCE_PG(fakeSpiPreinitOpuConfig, PG_SPI_PREINIT_OPU_CONFIG);
#endif
#ifdef USE_ESCSERIAL
    FAKE_RESOURCE_PG(fakeEscSerialConfig, PG_ESCSERIAL_CONFIG);
#endif
#ifdef USE_CAMERA_CONTROL
    FAKE_RESOURCE_PG(fakeCameraControlConfig, PG_CAMERA_CONTROL_CONFIG);
#endif
#ifdef USE_BARO
    FAKE_RESOURCE_PG(fakeBarometerConfig, PG_BAROMETER_CONFIG);
#endif
#ifdef USE_MAG
    FAKE_RESOURCE_PG(fakeCompassConfig, PG_COMPASS_CONFIG);
#endif
#ifdef USE_SDCARD
    FAKE_RESOURCE_PG(fakeSdcardConfig, PG_SDCARD_CONFIG);
#endif
#ifdef USE_PINIO
    FAKE_RESOURCE_PG(fakePinioConfig, PG_PINIO_CONFIG);
#endif
#if defined(USE_USB_MSC)
    FAKE_RESOURCE_PG(fakeUsbConfig, PG_USB_CONFIG);
#endif
#ifdef USE_FLASH
    FAKE_RESOURCE_PG(fakeFlashConfig, PG_FLASH_CONFIG);
#endif
#ifdef USE_MAX7456
    FAKE_RESOURCE_PG(fakeMax7456Config, PG_MAX7456_CONFIG);
#endif
#ifdef USE_RX_SPI
    FAKE_RESOURCE_PG(fakeRxSpiConfig, PG_RX_SPI_CONFIG);
#endif

    // PGs under test. The real definitions live in pg/timerio.c (needs the board's
    // TIMER_PIN_MAPPING), pg/adc.c and pg/serial_uart.c (pull in driver code); the registrations
    // here use a fixed fake board instead.
    PG_REGISTER_ARRAY_WITH_RESET_FN(timerIOConfig_t, MAX_TIMER_PINMAP_COUNT, timerIOConfig, PG_TIMER_IO_CONFIG, 1);
    PG_REGISTER_WITH_RESET_FN(adcConfig_t, adcConfig, PG_ADC_CONFIG, 1);
    PG_REGISTER_ARRAY_WITH_RESET_FN(serialUartConfig_t, UARTDEV_COUNT_MAX, serialUartConfig, PG_SERIAL_UART_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// ---------------------------------------------------------------------------------------------
// Fake board
//
//   pin  timerHardware occurrences (1-based, in table order)       default timerIOConfig
//   C08  #1 TIM3 CH3 AF2, #2 TIM8 CH3 AF3                          slot 0: occurrence 1, dmaopt 1
//   A01  #1 TIM2 CH2 AF1                                           slot 1: occurrence 1, dmaopt NONE
//   B13  #1 TIM1 CH1N AF1                                          no slot
//   A02  on the board, no timer hardware                           no slot
//   D05  not on the board (IOGetByTag() returns NULL)              no slot
//
// DMA options (what dmaGetChannelSpecByTimerValue()/ByPeripheral() accept):
//   TIM3 CH3: 0..1   TIM8 CH3: 0..2   TIM2 CH2: 0..0   TIM1 CH1: none
//   UART_TX/UART_RX: 0..1   ADC: 0..2
// ---------------------------------------------------------------------------------------------

static const ioTag_t TAG_C08 = DEFIO_TAG_MAKE(2, 8);
static const ioTag_t TAG_A01 = DEFIO_TAG_MAKE(0, 1);
static const ioTag_t TAG_B13 = DEFIO_TAG_MAKE(1, 13);
static const ioTag_t TAG_A02 = DEFIO_TAG_MAKE(0, 2);
static const ioTag_t TAG_D05 = DEFIO_TAG_MAKE(3, 5);

static TIM_TypeDef fakeTim1 = {};
static TIM_TypeDef fakeTim2 = {};
static TIM_TypeDef fakeTim3 = {};
static TIM_TypeDef fakeTim8 = {};

static timerHardware_t makeTimer(TIM_TypeDef *tim, ioTag_t tag, uint16_t channelIndex, bool nChannel, uint8_t af)
{
    timerHardware_t t = timerHardware_t();
    t.tim = tim;
    t.tag = tag;
    t.channel = (uint8_t)CC_CHANNEL_FROM_INDEX(channelIndex);
    t.output = nChannel ? (uint8_t)TIMER_OUTPUT_N_CHANNEL : (uint8_t)TIMER_OUTPUT_NONE;
    t.alternateFunction = af;
    return t;
}

extern "C" {
const timerHardware_t timerHardware[1] = {};
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    makeTimer(&fakeTim3, TAG_C08, 2, false, 2),
    makeTimer(&fakeTim8, TAG_C08, 2, false, 3),
    makeTimer(&fakeTim2, TAG_A01, 1, false, 1),
    makeTimer(&fakeTim1, TAG_B13, 0, true, 1),
};

void pgResetFn_timerIOConfig(timerIOConfig_t *config)
{
    config[0].ioTag = TAG_C08;
    config[0].index = 1;
    config[0].dmaopt = 1;
    config[1].ioTag = TAG_A01;
    config[1].index = 1;
    config[1].dmaopt = DMA_OPT_UNUSED;
}

void pgResetFn_adcConfig(adcConfig_t *config)
{
    for (int i = 0; i < ADCDEV_COUNT; i++) {
        config->dmaopt[i] = DMA_OPT_UNUSED;
    }
}

void pgResetFn_serialUartConfig(serialUartConfig_t *config)
{
    for (int i = 0; i < UARTDEV_COUNT_MAX; i++) {
        config[i].txDmaopt = DMA_OPT_UNUSED;
        config[i].rxDmaopt = DMA_OPT_UNUSED;
    }
}
}

// Controllable fakes used by the stubs at the end of the file.
static resourceOwner_e fakeDmaOwner = OWNER_FREE;
static std::string fakeRx;
static size_t fakeRxPos = 0;
static ioTag_t fakeLastIoTag = 0;

// DMA channel specs per request: {controller, stream, channel}. Only the printed numbers matter.
static dmaChannelSpec_t makeSpec(unsigned controller, unsigned stream, unsigned channel)
{
    dmaChannelSpec_t s = dmaChannelSpec_t();
    s.code = (dmaCode_t)DMA_CODE(controller, stream, channel);
    return s;
}

static const dmaChannelSpec_t uartSpecs[] = { makeSpec(1, 3, 4), makeSpec(1, 4, 7) };
static const dmaChannelSpec_t adcSpecs[] = { makeSpec(2, 0, 0), makeSpec(2, 4, 0), makeSpec(2, 2, 1) };
static const dmaChannelSpec_t tim3Ch3Specs[] = { makeSpec(1, 7, 5), makeSpec(2, 4, 5) };
static const dmaChannelSpec_t tim8Ch3Specs[] = { makeSpec(2, 1, 7), makeSpec(2, 2, 0), makeSpec(2, 3, 6) };
static const dmaChannelSpec_t tim2Ch2Specs[] = { makeSpec(1, 6, 3) };

#define ARRAY_SPEC(a, opt) (((opt) >= 0 && (opt) < (int)ARRAYLEN(a)) ? &(a)[(opt)] : NULL)

// Fixture: every test starts from default PGs, a fresh CLI session and the full fake board.
class CliTimerDmaTest : public ::testing::Test {
protected:
    void SetUp() override {
        pgResetAll();
        fakeDmaOwner = OWNER_FREE;
        fakeRx.clear();
        fakeRxPos = 0;
        static serialPort_t port = {};
        testing::internal::CaptureStdout();
        cliEnter(&port);
        testing::internal::GetCapturedStdout();
    }

    // Run one CLI line. Returns "\r\n" + everything printed after the echoed input line, so
    // `hasLine()` works on the first output line too.
    std::string run(const std::string &line) {
        fakeRx = line + "\r";
        fakeRxPos = 0;
        testing::internal::CaptureStdout();
        cliProcess();
        const std::string raw = testing::internal::GetCapturedStdout();
        const size_t echoEnd = raw.find("\r\n");
        if (echoEnd == std::string::npos) {
            return raw;
        }
        return raw.substr(echoEnd);
    }

    static bool hasLine(const std::string &out, const std::string &line) {
        return out.find("\r\n" + line + "\r\n") != std::string::npos;
    }

    static bool has(const std::string &out, const std::string &text) {
        return out.find(text) != std::string::npos;
    }

    static size_t count(const std::string &out, const std::string &text) {
        size_t n = 0;
        for (size_t pos = out.find(text); pos != std::string::npos; pos = out.find(text, pos + text.size())) {
            n++;
        }
        return n;
    }
};

#define EXPECT_LINE(out, line)    EXPECT_TRUE(hasLine((out), (line))) << "missing line: " << (line) << "\noutput:\n" << (out)
#define EXPECT_NO_LINE(out, line) EXPECT_FALSE(hasLine((out), (line))) << "unexpected line: " << (line) << "\noutput:\n" << (out)
#define EXPECT_HAS(out, text)     EXPECT_TRUE(has((out), (text))) << "missing text: " << (text) << "\noutput:\n" << (out)
#define EXPECT_LACKS(out, text)   EXPECT_FALSE(has((out), (text))) << "unexpected text: " << (text) << "\noutput:\n" << (out)

// ---------------------------------------------------------------------------------------------
// strToPin(), reached through `timer <pin>`. The command echoes the parsed port letter and pin
// number, so the printed pin shows what the parser read. (`resource <name> <pin>` also calls
// strToPin(), but its set path walks every resourceTable PG, which this target does not register.)
// ---------------------------------------------------------------------------------------------

TEST_F(CliTimerDmaTest, StrToPinAcceptsPortLetterAndPinNumber)
{
    EXPECT_LINE(run("timer A01"), "timer A01 AF1");
    EXPECT_LINE(run("timer C08"), "timer C08 AF2");
    EXPECT_LINE(run("timer H15"), "timer H15 NONE");   // highest port and pin the grammar allows
    EXPECT_LINE(run("timer B0"), "timer B00 NONE");    // pin number without a leading zero
    EXPECT_LINE(run("timer E007"), "timer E07 NONE");  // extra leading zeros
}

TEST_F(CliTimerDmaTest, StrToPinIsCaseInsensitive)
{
    EXPECT_LINE(run("timer c08"), "timer C08 AF2");
    EXPECT_LINE(run("timer h15"), "timer H15 NONE");
}

TEST_F(CliTimerDmaTest, StrToPinNoneParsesToNoPin)
{
    // NONE is a valid token (tag 0) but names no pin on the board.
    EXPECT_HAS(run("timer NONE"), "PIN NOT USED ON BOARD.");
    EXPECT_HAS(run("timer none"), "PIN NOT USED ON BOARD.");
    EXPECT_HAS(run("timer None list"), "PIN NOT USED ON BOARD.");
}

TEST_F(CliTimerDmaTest, StrToPinRejectsMalformedPin)
{
    // Trailing characters, a timer-channel style name, a hex-looking suffix, out-of-range port or
    // pin, a missing pin number, and text that is not a pin at all.
    const char *bad[] = {
        "A01X", "B3CH", "CH1", "C0x", "I16", "A16", "A100", "Z01", "AA1", "A", "01", "xyz", "A01.5", "A01-", "A 01x",
    };
    for (size_t i = 0; i < ARRAYLEN(bad); i++) {
        const std::string out = run(std::string("timer ") + bad[i]);
        EXPECT_HAS(out, "Parse error") << "pin: " << bad[i];
        EXPECT_LACKS(out, "\r\ntimer ") << "pin: " << bad[i];
    }
}

// Not tested: a signed pin number ("C+1", "C-0"). IT #1482 lists "C+1" as rejected, but strToPin()
// passes the digits to strtol(), which accepts a leading sign, so "C+1" currently parses as C01.
// Fixing that needs a cli.c change (reject a non-digit first character after the port letter);
// this stage does not edit cli.c, so no assertion for it is committed.

TEST_F(CliTimerDmaTest, StrToPinRejectsOverflowingPinNumber)
{
    // strtol() saturates at LONG_MAX / LONG_MIN; neither may wrap into the 0..15 range.
    const char *bad[] = {
        "A99999999999999999999",
        "A9223372036854775808",
        "A-9223372036854775808",
        "A-99999999999999999999",
        "A4294967297",           // 2^32 + 1: would read as pin 1 if truncated to 32 bits
        "A18446744073709551617", // 2^64 + 1
    };
    for (size_t i = 0; i < ARRAYLEN(bad); i++) {
        const std::string out = run(std::string("timer ") + bad[i]);
        EXPECT_HAS(out, "Parse error") << "pin: " << bad[i];
        EXPECT_LACKS(out, "\r\ntimer ") << "pin: " << bad[i];
    }
}

// ---------------------------------------------------------------------------------------------
// dma: dump form, show/list, device form, errors
// ---------------------------------------------------------------------------------------------

TEST_F(CliTimerDmaTest, DmaBareCommandPrintsDmaoptDump)
{
    const std::string out = run("dma");
    EXPECT_LINE(out, "dma pin C08 1");           // configured option
    EXPECT_NO_LINE(out, "dma pin A01 NONE");     // unused entries are hidden in the bare form
    EXPECT_LACKS(out, "Currently active DMA:");
}

TEST_F(CliTimerDmaTest, DmaShowAndListPrintActiveDma)
{
    const char *forms[] = { "dma show", "dma list", "dma SHOW", "dma s", "dma l" };
    for (size_t i = 0; i < ARRAYLEN(forms); i++) {
        const std::string out = run(forms[i]);
        EXPECT_HAS(out, "Currently active DMA:") << forms[i];
        EXPECT_HAS(out, "DMA1 Stream 0: FREE") << forms[i];
        EXPECT_LACKS(out, "dma pin") << forms[i];
    }
}

TEST_F(CliTimerDmaTest, DmaShowWithTrailingTextIsNotShow)
{
    // Only a prefix of "show"/"list" selects the table; extra characters make it a device name.
    const std::string out = run("dma showx");
    EXPECT_HAS(out, "BAD DEVICE: showx");
    EXPECT_LACKS(out, "Currently active DMA:");
}

TEST_F(CliTimerDmaTest, DmaUnknownDeviceIsRejected)
{
    EXPECT_HAS(run("dma bogus"), "BAD DEVICE: bogus");
    EXPECT_HAS(run("dma bogus 1 2"), "BAD DEVICE: bogus");
    EXPECT_HAS(run("dma UART 1"), "BAD DEVICE: UART");
}

TEST_F(CliTimerDmaTest, DmaDeviceIndexOutOfRange)
{
    const std::string range = "index not between 1 and " + std::to_string((int)UARTDEV_COUNT_MAX);
    EXPECT_HAS(run("dma UART_TX 0"), range);
    EXPECT_HAS(run("dma UART_TX " + std::to_string((int)UARTDEV_COUNT_MAX + 1)), range);
    EXPECT_HAS(run("dma UART_TX -1"), range);
    EXPECT_HAS(run("dma UART_TX"), range);                 // missing index
    EXPECT_HAS(run("dma UART_TX abc"), range);
    EXPECT_HAS(run("dma UART_TX 1abc"), range);            // trailing characters
    EXPECT_HAS(run("dma UART_TX 1.0"), range);
    EXPECT_HAS(run("dma UART_TX 99999999999999999999"), range);        // strtol saturates at LONG_MAX
    EXPECT_HAS(run("dma UART_TX -9223372036854775808"), range);        // LONG_MIN
    EXPECT_HAS(run("dma UART_TX -99999999999999999999"), range);
}

TEST_F(CliTimerDmaTest, DmaDeviceIndexOfAbsentUartIsBadIndex)
{
    // The test target enables UART1..UART5 only (src/test/unit/target.h), so index 6 is a valid
    // slot number but has no UART behind it.
    EXPECT_HAS(run("dma UART_TX 6"), "BAD INDEX: '6'");
    EXPECT_HAS(run("dma UART_RX 6"), "BAD INDEX: '6'");
}

TEST_F(CliTimerDmaTest, DmaDeviceNameIsCaseInsensitive)
{
    const std::string out = run("dma uart_rx 2 1");
    EXPECT_HAS(out, "# dma UART_RX 2: changed from NONE to 1");
    EXPECT_EQ(1, serialUartConfig(1)->rxDmaopt);
}

TEST_F(CliTimerDmaTest, DmaDeviceShowsSetsAndClearsOption)
{
    std::string out = run("dma UART_TX 1");
    EXPECT_LINE(out, "dma UART_TX 1 NONE");

    out = run("dma UART_TX 1 1");
    EXPECT_HAS(out, "# dma UART_TX 1: changed from NONE to 1");
    EXPECT_EQ(1, serialUartConfig(0)->txDmaopt);
    EXPECT_EQ(DMA_OPT_UNUSED, serialUartConfig(0)->rxDmaopt);   // the sibling field is untouched
    EXPECT_EQ(DMA_OPT_UNUSED, serialUartConfig(1)->txDmaopt);   // so is the next index

    out = run("dma UART_TX 1");
    EXPECT_LINE(out, "dma UART_TX 1 1");
    EXPECT_LINE(out, "# UART_TX 1: DMA1 Stream 4 Channel 7");

    out = run("dma UART_TX 1 1");
    EXPECT_HAS(out, "# dma UART_TX 1: no change: 1");

    out = run("dma UART_TX 1 none");
    EXPECT_HAS(out, "# dma UART_TX 1: changed from 1 to NONE");
    EXPECT_EQ(DMA_OPT_UNUSED, serialUartConfig(0)->txDmaopt);
}

TEST_F(CliTimerDmaTest, DmaDeviceListPrintsEveryOption)
{
    const std::string out = run("dma UART_TX 1 list");
    EXPECT_LINE(out, "# 0: DMA1 Stream 3 Channel 4");
    EXPECT_LINE(out, "# 1: DMA1 Stream 4 Channel 7");
    EXPECT_LACKS(out, "# 2:");
}

TEST_F(CliTimerDmaTest, DmaDeviceInvalidOptionIsRejectedAndKeepsValue)
{
    run("dma UART_TX 1 0");
    ASSERT_EQ(0, serialUartConfig(0)->txDmaopt);

    const char *bad[] = {
        "abc", "1x", "2", "99", "-2", "128", "-129", "",
        "99999999999999999999",       // saturates at LONG_MAX
        "-9223372036854775808",       // LONG_MIN
    };
    for (size_t i = 0; i < ARRAYLEN(bad); i++) {
        const std::string out = run(std::string("dma UART_TX 1 ") + bad[i]);
        if (bad[i][0] == '\0') {
            // no option given: the command shows the value instead of changing it
            EXPECT_LINE(out, "dma UART_TX 1 0");
        } else {
            EXPECT_HAS(out, std::string("INVALID DMA OPTION FOR UART_TX 1: '") + bad[i] + "'") << "option: " << bad[i];
        }
        EXPECT_EQ(0, serialUartConfig(0)->txDmaopt) << "option: " << bad[i];
    }
}

// ---------------------------------------------------------------------------------------------
// dma ADC rows (PR #1501): `dma ADC <1-3> [opt|list|none]`, one option per ADC device
// ---------------------------------------------------------------------------------------------

TEST_F(CliTimerDmaTest, DmaAdcRowShowsSetsAndLists)
{
    std::string out = run("dma ADC 1");
    EXPECT_LINE(out, "dma ADC 1 NONE");

    out = run("dma ADC 2 2");
    EXPECT_HAS(out, "# dma ADC 2: changed from NONE to 2");
    EXPECT_EQ(2, adcConfig()->dmaopt[ADCDEV_2]);
    EXPECT_EQ(DMA_OPT_UNUSED, adcConfig()->dmaopt[ADCDEV_1]);   // other devices untouched
    EXPECT_EQ(DMA_OPT_UNUSED, adcConfig()->dmaopt[ADCDEV_3]);

    out = run("dma adc 2");
    EXPECT_LINE(out, "dma ADC 2 2");
    EXPECT_LINE(out, "# ADC 2: DMA2 Stream 2 Channel 1");

    out = run("dma ADC 2 list");
    EXPECT_LINE(out, "# 0: DMA2 Stream 0 Channel 0");
    EXPECT_LINE(out, "# 1: DMA2 Stream 4 Channel 0");
    EXPECT_LINE(out, "# 2: DMA2 Stream 2 Channel 1");
    EXPECT_LACKS(out, "# 3:");

    out = run("dma ADC 2 none");
    EXPECT_HAS(out, "# dma ADC 2: changed from 2 to NONE");
    EXPECT_EQ(DMA_OPT_UNUSED, adcConfig()->dmaopt[ADCDEV_2]);
}

TEST_F(CliTimerDmaTest, DmaAdcRowReportsForeignStreamOwner)
{
    fakeDmaOwner = OWNER_SPI_SDI;
    const std::string out = run("dma ADC 1 1");
    EXPECT_HAS(out, "# dma ADC 1: changed from NONE to 1");
    EXPECT_HAS(out, "# ADC 1: CLAIMED BY SPI_SDI");
}

TEST_F(CliTimerDmaTest, DmaAdcRowRejectsBadIndexAndOption)
{
    const std::string range = "index not between 1 and " + std::to_string((int)ADCDEV_COUNT);
    EXPECT_HAS(run("dma ADC 0"), range);
    EXPECT_HAS(run("dma ADC " + std::to_string((int)ADCDEV_COUNT + 1)), range);
    EXPECT_HAS(run("dma ADC 2x"), range);
    EXPECT_HAS(run("dma ADC -9223372036854775808"), range);

    EXPECT_HAS(run("dma ADC 1 3"), "INVALID DMA OPTION FOR ADC 1: '3'");
    EXPECT_HAS(run("dma ADC 1 x"), "INVALID DMA OPTION FOR ADC 1: 'x'");
    EXPECT_HAS(run("dma ADC 1 99999999999999999999"), "INVALID DMA OPTION FOR ADC 1: '99999999999999999999'");
    for (int i = 0; i < ADCDEV_COUNT; i++) {
        EXPECT_EQ(DMA_OPT_UNUSED, adcConfig()->dmaopt[i]);
    }
}

TEST_F(CliTimerDmaTest, DmaAdcRowsAppearInDumpAndDiff)
{
    // dump prints every ADC device; diff prints only the devices that differ from the default.
    std::string out = run("dump");
    EXPECT_LINE(out, "dma ADC 1 NONE");
    EXPECT_LINE(out, "dma ADC 2 NONE");
    EXPECT_LINE(out, "dma ADC 3 NONE");

    run("dma ADC 3 1");
    out = run("diff");
    EXPECT_LINE(out, "dma ADC 3 1");
    EXPECT_NO_LINE(out, "dma ADC 1 NONE");
    EXPECT_NO_LINE(out, "dma ADC 2 NONE");
}

// ---------------------------------------------------------------------------------------------
// dma pin <pin> [<option>|list|none]
// ---------------------------------------------------------------------------------------------

TEST_F(CliTimerDmaTest, DmaPinRejectsInvalidPin)
{
    EXPECT_HAS(run("dma pin"), "INVALID PIN: ''");
    EXPECT_HAS(run("dma pin xyz"), "INVALID PIN: 'xyz'");
    EXPECT_HAS(run("dma pin C08X"), "INVALID PIN: 'C08X'");
    EXPECT_HAS(run("dma pin CH1"), "INVALID PIN: 'CH1'");
    EXPECT_HAS(run("dma pin C16"), "INVALID PIN: 'C16'");
    EXPECT_HAS(run("dma pin C99999999999999999999"), "INVALID PIN: 'C99999999999999999999'");
    EXPECT_HAS(run("dma pin NONE"), "INVALID PIN: 'NONE'");               // NONE is tag 0, which is no pin
}

TEST_F(CliTimerDmaTest, DmaPinRejectsPinNotOnBoard)
{
    EXPECT_HAS(run("dma pin D05"), "INVALID PIN: 'D05'");
}

TEST_F(CliTimerDmaTest, DmaPinWithoutTimerOptionIsRejected)
{
    // A02: on the board, timer hardware has no entry. B13: timer hardware exists but the pin has
    // no timerIOConfig slot. Neither has a timer option to attach a DMA option to.
    EXPECT_HAS(run("dma pin A02"), "NO TIMER OPTION SELECTED FOR A02");
    EXPECT_HAS(run("dma pin B13"), "NO TIMER OPTION SELECTED FOR B13");
    EXPECT_HAS(run("dma pin B13 0"), "NO TIMER OPTION SELECTED FOR B13");
    EXPECT_HAS(run("dma pin A02 list"), "NO TIMER OPTION SELECTED FOR A02");
}

TEST_F(CliTimerDmaTest, DmaPinShowsConfiguredAndUnusedOption)
{
    std::string out = run("dma pin C08");
    EXPECT_LINE(out, "dma pin C08 1");
    EXPECT_LINE(out, "# pin C08: DMA2 Stream 4 Channel 5");

    out = run("dma pin A01");
    EXPECT_LINE(out, "dma pin A01 NONE");
}

TEST_F(CliTimerDmaTest, DmaPinListPrintsEveryOptionOfTheSelectedTimer)
{
    std::string out = run("dma pin C08 list");
    EXPECT_LINE(out, "# 0: DMA1 Stream 7 Channel 5");
    EXPECT_LINE(out, "# 1: DMA2 Stream 4 Channel 5");
    EXPECT_LACKS(out, "# 2:");

    out = run("dma pin A01 list");
    EXPECT_LINE(out, "# 0: DMA1 Stream 6 Channel 3");
    EXPECT_LACKS(out, "# 1:");
}

TEST_F(CliTimerDmaTest, DmaPinSetsAndClearsOption)
{
    std::string out = run("dma pin C08 0");
    EXPECT_HAS(out, "# dma pin C08: changed from 1 to 0");
    EXPECT_EQ(0, timerIOConfig(0)->dmaopt);
    EXPECT_EQ(DMA_OPT_UNUSED, timerIOConfig(1)->dmaopt);        // other slots untouched

    out = run("dma pin C08 0");
    EXPECT_HAS(out, "# dma pin C08: no change: 0");

    out = run("dma pin c08 none");
    EXPECT_HAS(out, "# dma pin C08: changed from 0 to NONE");
    EXPECT_EQ(DMA_OPT_UNUSED, timerIOConfig(0)->dmaopt);
    EXPECT_EQ(TAG_C08, timerIOConfig(0)->ioTag);               // the timer option stays

    out = run("dma pin A01 0");
    EXPECT_HAS(out, "# dma pin A01: changed from NONE to 0");
    EXPECT_EQ(0, timerIOConfig(1)->dmaopt);
}

TEST_F(CliTimerDmaTest, DmaPinFollowsTheSelectedTimerOption)
{
    // C08 occurrence #2 is TIM8 CH3, which has three DMA options (0..2); occurrence #1 has two.
    EXPECT_HAS(run("dma pin C08 2"), "INVALID DMA OPTION FOR PIN C08: '2'");
    run("timer C08 af3");
    EXPECT_HAS(run("dma pin C08 2"), "# dma pin C08: changed from NONE to 2");
    EXPECT_LINE(run("dma pin C08 list"), "# 2: DMA2 Stream 3 Channel 6");
}

TEST_F(CliTimerDmaTest, DmaPinInvalidOptionIsRejectedAndKeepsValue)
{
    const char *bad[] = {
        "abc", "1x", "2", "99", "128", "-129",
        "99999999999999999999",
        "-9223372036854775808",
        "-99999999999999999999",
    };
    for (size_t i = 0; i < ARRAYLEN(bad); i++) {
        const std::string out = run(std::string("dma pin C08 ") + bad[i]);
        EXPECT_HAS(out, std::string("INVALID DMA OPTION FOR PIN C08: '") + bad[i] + "'") << "option: " << bad[i];
        EXPECT_EQ(1, timerIOConfig(0)->dmaopt) << "option: " << bad[i];
    }
}

// ---------------------------------------------------------------------------------------------
// timer
// ---------------------------------------------------------------------------------------------

TEST_F(CliTimerDmaTest, TimerBareCommandPrintsDumpWithOneBasedChannels)
{
    std::string out = run("timer");
    EXPECT_LINE(out, "timer C08 AF2");
    EXPECT_LINE(out, "# pin C08: TIM3 CH3 (AF2)");
    EXPECT_LINE(out, "timer A01 AF1");
    EXPECT_LINE(out, "# pin A01: TIM2 CH2 (AF1)");
    EXPECT_LACKS(out, "Currently active Timers:");

    run("timer B13 af1");
    out = run("timer");
    EXPECT_LINE(out, "timer B13 AF1");
    EXPECT_LINE(out, "# pin B13: TIM1 CH1N (AF1)");   // 1-based, complementary output marked N
}

TEST_F(CliTimerDmaTest, TimerShowAndListPrintHeadingNoteAndDashesInOrder)
{
    const char *forms[] = { "timer show", "timer list", "timer SHOW", "timer s", "timer l" };
    for (size_t i = 0; i < ARRAYLEN(forms); i++) {
        const std::string out = run(forms[i]);
        const size_t heading = out.find("Currently active Timers:");
        const size_t note = out.find("(reboot to update)");
        const size_t dashes = out.find("-----------------------");
        ASSERT_NE(std::string::npos, heading) << forms[i];
        ASSERT_NE(std::string::npos, note) << forms[i];
        ASSERT_NE(std::string::npos, dashes) << forms[i];
        EXPECT_LT(heading, note) << forms[i];
        EXPECT_LT(note, dashes) << forms[i];
        EXPECT_EQ(1u, count(out, "(reboot to update)")) << forms[i];
        EXPECT_LINE(out, "TIM1: FREE");
    }
}

TEST_F(CliTimerDmaTest, TimerShowListsAllocatedChannelWithOwnerAndIndex)
{
    // Allocation is process-global state in timer_common.c and has no reset call. The allocation
    // below sticks for the rest of the binary, so this test accepts "already allocated" and no
    // other test asserts that TIM2 is free.
    timerAllocate(TAG_A01, OWNER_MOTOR, RESOURCE_INDEX(2));
    ASSERT_EQ(OWNER_MOTOR, timerGetOwner(TAG_A01));
    ASSERT_EQ(RESOURCE_INDEX(2), timerGetOwnerResourceIndex(TAG_A01));

    const std::string out = run("timer show");
    EXPECT_HAS(out, "TIM2:\r\n    CH2 : MOTOR 3\r\n");
    EXPECT_LINE(out, "TIM1: FREE");
}

TEST_F(CliTimerDmaTest, ResourceShowAllPrintsRebootNoteOnceAndKeepsSections)
{
    const std::string out = run("resource show all");
    EXPECT_EQ(1u, count(out, "(reboot to update)"));
    EXPECT_HAS(out, "Currently active IO resource assignments:\r\n(reboot to update)");
    const size_t timers = out.find("Currently active Timers:");
    ASSERT_NE(std::string::npos, timers);
    EXPECT_EQ(std::string::npos, out.substr(timers, 80).find("(reboot to update)"));
    EXPECT_HAS(out, "Currently active DMA:");
    EXPECT_LT(timers, out.find("Currently active DMA:"));

    // plain `resource show` has no timer or DMA block
    const std::string plain = run("resource show");
    EXPECT_LACKS(plain, "Currently active Timers:");
    EXPECT_LACKS(plain, "Currently active DMA:");
}

TEST_F(CliTimerDmaTest, TimerPinWithoutOptionShowsCurrentSelection)
{
    std::string out = run("timer C08");
    EXPECT_LINE(out, "timer C08 AF2");
    EXPECT_LINE(out, "# pin C08: TIM3 CH3 (AF2)");

    out = run("timer c08");
    EXPECT_LINE(out, "timer C08 AF2");

    out = run("timer B13");        // no slot yet: reports NONE and does not create one
    EXPECT_LINE(out, "timer B13 NONE");
    EXPECT_EQ(IO_TAG_NONE, timerIOConfig(2)->ioTag);
}

TEST_F(CliTimerDmaTest, TimerPinListPrintsAlternateFunctionsInTableOrder)
{
    std::string out = run("timer C08 list");
    EXPECT_LINE(out, "# AF2: TIM3 CH3");
    EXPECT_LINE(out, "# AF3: TIM8 CH3");
    EXPECT_LT(out.find("# AF2:"), out.find("# AF3:"));

    out = run("timer B13 list");
    EXPECT_LINE(out, "# AF1: TIM1 CH1N");

    out = run("timer A02 list");   // on the board, no timer: nothing to list
    EXPECT_LACKS(out, "# AF");
    EXPECT_LACKS(out, "ERROR");
}

TEST_F(CliTimerDmaTest, TimerChangeAlternateFunctionResetsDmaOption)
{
    ASSERT_EQ(1, timerIOConfig(0)->dmaopt);

    std::string out = run("timer C08 af3");
    EXPECT_HAS(out, "# timer C08: changed from AF2 to AF3");
    EXPECT_EQ(2, timerIOConfig(0)->index);
    EXPECT_EQ(DMA_OPT_UNUSED, timerIOConfig(0)->dmaopt);        // a new timer option invalidates the DMA option
    EXPECT_LINE(run("dma pin C08"), "dma pin C08 NONE");
    EXPECT_LINE(run("timer C08"), "timer C08 AF3");
}

TEST_F(CliTimerDmaTest, TimerSameAlternateFunctionKeepsDmaOption)
{
    std::string out = run("timer C08 af2");
    EXPECT_HAS(out, "# timer C08: no change: AF2");
    EXPECT_EQ(1, timerIOConfig(0)->index);
    EXPECT_EQ(1, timerIOConfig(0)->dmaopt);
    EXPECT_LINE(run("dma pin C08"), "dma pin C08 1");

    out = run("timer C08 AF2");   // option keyword is case-insensitive
    EXPECT_HAS(out, "# timer C08: no change: AF2");
    EXPECT_EQ(1, timerIOConfig(0)->dmaopt);
}

TEST_F(CliTimerDmaTest, TimerNoneRemovesMappingAndDropsDmaOption)
{
    std::string out = run("timer C08 none");
    EXPECT_HAS(out, "# timer C08: changed from AF2 to NONE");
    EXPECT_EQ(IO_TAG_NONE, timerIOConfig(0)->ioTag);
    EXPECT_EQ(DMA_OPT_UNUSED, timerIOConfig(0)->dmaopt);
    EXPECT_LINE(run("timer C08"), "timer C08 NONE");
    EXPECT_HAS(run("dma pin C08"), "NO TIMER OPTION SELECTED FOR C08");

    out = run("timer C08 none");  // already removed
    EXPECT_HAS(out, "# timer C08: no change: NONE");

    out = run("timer C08 af2");   // map it again
    EXPECT_HAS(out, "# timer C08: changed from NONE to AF2");
    EXPECT_EQ(TAG_C08, timerIOConfig(0)->ioTag);
    EXPECT_EQ(DMA_OPT_UNUSED, timerIOConfig(0)->dmaopt);
}

TEST_F(CliTimerDmaTest, TimerNewPinUsesFirstEmptySlot)
{
    ASSERT_EQ(IO_TAG_NONE, timerIOConfig(2)->ioTag);
    const std::string out = run("timer B13 af1");
    EXPECT_HAS(out, "# timer B13: changed from NONE to AF1");
    EXPECT_EQ(TAG_B13, timerIOConfig(2)->ioTag);
    EXPECT_EQ(1, timerIOConfig(2)->index);
    EXPECT_EQ(DMA_OPT_UNUSED, timerIOConfig(2)->dmaopt);
}

TEST_F(CliTimerDmaTest, TimerRejectsUnknownAlternateFunction)
{
    const char *bad[] = {
        "af99", "af0", "af", "afx", "af2x", "af-1", "AF 2",
        "af99999999999999999999",
        "af-9223372036854775808",
        "af18446744073709551617",
    };
    for (size_t i = 0; i < ARRAYLEN(bad); i++) {
        const std::string out = run(std::string("timer C08 ") + bad[i]);
        EXPECT_HAS(out, "INVALID ALTERNATE FUNCTION FOR C08") << "option: " << bad[i];
        EXPECT_EQ(1, timerIOConfig(0)->index) << "option: " << bad[i];     // unchanged
        EXPECT_EQ(1, timerIOConfig(0)->dmaopt) << "option: " << bad[i];
    }
    EXPECT_HAS(run("timer C08 af99"), "INVALID ALTERNATE FUNCTION FOR C08: 'af99'");
    EXPECT_HAS(run("timer A02 af1"), "INVALID ALTERNATE FUNCTION FOR A02: 'af1'");   // no timer on this pin
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        EXPECT_NE(TAG_A02, timerIOConfig(i)->ioTag);     // the rejected pin got no slot
    }
}

TEST_F(CliTimerDmaTest, TimerRejectsNumericAndUnknownOption)
{
    // The old numeric `timer <pin> <N>` form is gone.
    EXPECT_HAS(run("timer C08 1"), "INVALID TIMER OPTION FOR C08: '1'");
    EXPECT_HAS(run("timer C08 2"), "INVALID TIMER OPTION FOR C08: '2'");
    EXPECT_HAS(run("timer C08 xyz"), "INVALID TIMER OPTION FOR C08: 'xyz'");
    EXPECT_HAS(run("timer C08 noned"), "INVALID TIMER OPTION FOR C08: 'noned'");
    EXPECT_EQ(1, timerIOConfig(0)->index);
}

TEST_F(CliTimerDmaTest, TimerRejectsMalformedPin)
{
    const char *bad[] = { "xyz", "C08X", "CH1", "C0x", "C16", "I00", "C99999999999999999999", "showx" };
    for (size_t i = 0; i < ARRAYLEN(bad); i++) {
        const std::string out = run(std::string("timer ") + bad[i] + " list");
        EXPECT_HAS(out, "Parse error") << "pin: " << bad[i];
        EXPECT_LACKS(out, "# AF") << "pin: " << bad[i];
    }
    EXPECT_HAS(run("timer showx"), "Parse error");   // not a prefix of show/list, so parsed as a pin
}

TEST_F(CliTimerDmaTest, TimerRejectsPinNotOnBoard)
{
    EXPECT_HAS(run("timer D05 list"), "PIN NOT USED ON BOARD.");
    EXPECT_HAS(run("timer D05 af1"), "PIN NOT USED ON BOARD.");
}

TEST_F(CliTimerDmaTest, TimerRejectsNewPinWhenMapIsFull)
{
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        timerIOConfigMutable(i)->ioTag = DEFIO_TAG_MAKE(4, i % 16);   // E00..E15, E00..E04: slot filler
    }
    EXPECT_HAS(run("timer B13 af1"), "PIN TIMER MAP FULL.");
    EXPECT_NE(TAG_B13, timerIOConfig(MAX_TIMER_PINMAP_COUNT - 1)->ioTag);
}

// ---------------------------------------------------------------------------------------------
// dump / diff
// ---------------------------------------------------------------------------------------------

TEST_F(CliTimerDmaTest, DumpListsTimerAndDmaSections)
{
    const std::string out = run("dump");
    const size_t timerHeader = out.find("\r\n# timer\r\n");
    const size_t dmaHeader = out.find("\r\n# dma\r\n");
    ASSERT_NE(std::string::npos, timerHeader);
    ASSERT_NE(std::string::npos, dmaHeader);
    EXPECT_LT(timerHeader, dmaHeader);
    EXPECT_LINE(out, "timer C08 AF2");
    EXPECT_LINE(out, "timer A01 AF1");
    EXPECT_LINE(out, "dma pin C08 1");
    EXPECT_LINE(out, "dma pin A01 NONE");
    EXPECT_LINE(out, "dma UART_TX 1 NONE");
}

TEST_F(CliTimerDmaTest, DiffOfDefaultConfigPrintsNoTimerOrDmaLine)
{
    const std::string out = run("diff");
    EXPECT_HAS(out, "\r\n# timer\r\n");
    EXPECT_HAS(out, "\r\n# dma\r\n");
    EXPECT_LACKS(out, "\r\ntimer ");
    EXPECT_LACKS(out, "\r\ndma ");
}

TEST_F(CliTimerDmaTest, DiffAfterTimerChangePrintsTimerAndResetDmaLine)
{
    run("timer C08 af3");
    const std::string out = run("diff");
    EXPECT_LINE(out, "timer C08 AF3");
    EXPECT_LINE(out, "dma pin C08 NONE");   // default option 1 was reset by the timer change
    EXPECT_NO_LINE(out, "timer A01 AF1");   // the unchanged pin stays silent
}

TEST_F(CliTimerDmaTest, DiffAfterTimerNoneOmitsDmaPinNone)
{
    // The removed mapping already resets dmaopt; a `dma pin C08 NONE` line would fail on replay
    // with NO TIMER OPTION SELECTED.
    run("timer C08 none");
    const std::string out = run("diff");
    EXPECT_LINE(out, "timer C08 NONE");
    EXPECT_LACKS(out, "dma pin C08");
}

TEST_F(CliTimerDmaTest, DiffAfterDmaPinNoneKeepsTimerMapping)
{
    run("dma pin C08 none");
    const std::string out = run("diff");
    EXPECT_LINE(out, "dma pin C08 NONE");
    EXPECT_LACKS(out, "timer C08");
}

TEST_F(CliTimerDmaTest, DiffAfterDmaPinChangePrintsNewOption)
{
    run("dma pin C08 0");
    const std::string out = run("diff");
    EXPECT_LINE(out, "dma pin C08 0");
    EXPECT_LACKS(out, "timer C08");
}

TEST_F(CliTimerDmaTest, DiffAfterNewTimerPinPrintsIt)
{
    run("timer B13 af1");
    const std::string out = run("diff");
    EXPECT_LINE(out, "timer B13 AF1");
    EXPECT_LINE(out, "# pin B13: TIM1 CH1N (AF1)");
}

TEST_F(CliTimerDmaTest, DiffReplayRestoresTheChangedConfig)
{
    // Round trip: the lines `diff` prints, fed back, must reproduce the same state.
    run("timer C08 af3");
    run("dma pin C08 2");
    run("dma UART_TX 2 1");
    const std::string out = run("diff");
    EXPECT_LINE(out, "timer C08 AF3");
    EXPECT_LINE(out, "dma pin C08 2");
    EXPECT_LINE(out, "dma UART_TX 2 1");

    pgResetAll();
    run("timer C08 AF3");
    run("dma pin C08 2");
    run("dma UART_TX 2 1");
    EXPECT_EQ(2, timerIOConfig(0)->index);
    EXPECT_EQ(2, timerIOConfig(0)->dmaopt);
    EXPECT_EQ(1, serialUartConfig(1)->txDmaopt);
}

// ---------------------------------------------------------------------------------------------
// help
// ---------------------------------------------------------------------------------------------

TEST_F(CliTimerDmaTest, HelpListsDmaPinFormsWhenTimerMgmtIsEnabled)
{
    const std::string out = run("help");
    EXPECT_HAS(out, "dma - show/set DMA assignments");
    EXPECT_HAS(out, "pin <pin> list | pin <pin> [<option>|none]");
    EXPECT_HAS(out, "timer - show/set timers");
    EXPECT_HAS(out, "<pin> [af<alternate function>|none]");
}

// STUBS
extern "C" {

// Every IO tag exists on the fake board except D05 and IO_TAG_NONE (as in the real IOGetByTag()).
ioRec_t ioRecs[DEFIO_IO_USED_COUNT];
int IO_GPIOPortIdx(IO_t) { return DEFIO_TAG_GPIOID(fakeLastIoTag); }
int IO_GPIOPinIdx(IO_t) { return DEFIO_TAG_PIN(fakeLastIoTag); }
IO_t IOGetByTag(ioTag_t tag)
{
    fakeLastIoTag = tag;
    return (tag == IO_TAG_NONE || tag == TAG_D05) ? NULL : (IO_t)&ioRecs[0];
}
ioRec_t *IO_Rec(IO_t) { return &ioRecs[0]; }

int tfp_sprintf(char *s, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    const int ret = vsprintf(s, fmt, args);
    va_end(args);
    return ret;
}

dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef *) { return static_cast<dmaIdentifier_e>(1); }
resourceOwner_e dmaGetOwner(dmaIdentifier_e) { return fakeDmaOwner; }
uint8_t dmaGetResourceIndex(dmaIdentifier_e) { return 0; }

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t, int8_t opt)
{
    switch (device) {
    case DMA_PERIPH_UART_TX:
    case DMA_PERIPH_UART_RX:
        return ARRAY_SPEC(uartSpecs, opt);
    case DMA_PERIPH_ADC:
        return ARRAY_SPEC(adcSpecs, opt);
    default:
        return NULL;
    }
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t opt)
{
    if (tim == &fakeTim3 && channel == CC_CHANNEL_FROM_INDEX(2)) {
        return ARRAY_SPEC(tim3Ch3Specs, opt);
    } else if (tim == &fakeTim8 && channel == CC_CHANNEL_FROM_INDEX(2)) {
        return ARRAY_SPEC(tim8Ch3Specs, opt);
    } else if (tim == &fakeTim2 && channel == CC_CHANNEL_FROM_INDEX(1)) {
        return ARRAY_SPEC(tim2Ch2Specs, opt);
    }
    return NULL;
}

int8_t timerGetNumberByIndex(uint8_t index)
{
    static const int8_t numbers[] = { 1, 2, 3, 8 };   // zero terminates the list
    return index < ARRAYLEN(numbers) ? numbers[index] : 0;
}

int8_t timerGetTIMNumber(const TIM_TypeDef *tim)
{
    if (tim == &fakeTim1) return 1;
    if (tim == &fakeTim2) return 2;
    if (tim == &fakeTim3) return 3;
    if (tim == &fakeTim8) return 8;
    return 0;
}


uint32_t serialRxBytesWaiting(const serialPort_t *) { return (uint32_t)(fakeRx.size() - fakeRxPos); }
uint8_t serialRead(serialPort_t *) { return (uint8_t)fakeRx[fakeRxPos++]; }

bufWriter_t *bufWriterInit(uint8_t *, int, bufWrite_t, void *)
{
    // cliProcess() returns early without a writer; bufWriterAppend()/Flush() are stubbed, so the
    // object is never dereferenced.
    static uint8_t storage[sizeof(bufWriter_t) + 64];
    return reinterpret_cast<bufWriter_t *>(storage);
}

// printConfig() copies the PGs, resets the live ones to defaults, prints, then restores.
void resetConfigs(void) { pgResetAll(); }
float motor_disarmed[MAX_SUPPORTED_MOTORS];

uint16_t batteryWarningVoltage;
uint8_t useHottAlarmSoundPeriod (void) { return 0; }
const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000}; // see baudRate_e

uint32_t micros(void) {return 0;}

int32_t getAmperage(void) {
    return 100;
}

uint16_t getBatteryVoltage(void) {
    return 42;
}

batteryState_e getBatteryState(void) {
    return BATTERY_OK;
}

uint8_t calculateBatteryPercentageRemaining(void) {
    return 67;
}

uint8_t getMotorCount() {
    return 4;
}


void setPrintfSerialPort(struct serialPort_s) {}

void tfp_printf(const char * expectedFormat, ...) {
    va_list args;

    va_start(args, expectedFormat);
    vprintf(expectedFormat, args);
    va_end(args);
}


void tfp_format(void *, void (*) (void *, char), const char * expectedFormat, va_list va) {
    vprintf(expectedFormat, va);
}

static const box_t boxes[] = { { 0, "DUMMYBOX", 0 } };
const box_t *findBoxByPermanentId(uint8_t) { return &boxes[0]; }
const box_t *findBoxByBoxId(boxId_e) { return &boxes[0]; }

uint32_t getBeeperOffMask(void) { return 0; }
uint32_t getPreferredBeeperOffMask(void) { return 0; }

void beeper(beeperMode_e) {}
void beeperSilence(void) {}
void beeperConfirmationBeeps(uint8_t) {}
void beeperWarningBeeps(uint8_t) {}
void beeperUpdate(timeUs_t) {}
uint32_t getArmingBeepTimeMicros(void) {return 0;}
beeperMode_e beeperModeForTableIndex(int) {return BEEPER_SILENCE;}
uint32_t beeperModeMaskForTableIndex(int idx) {UNUSED(idx); return 0;}
const char *beeperNameForTableIndex(int) {return NULL;}
int beeperTableEntryCount(void) {return 0;}
bool isBeeperOn(void) {return false;}
void beeperOffSetAll(uint8_t) {}
void setBeeperOffMask(uint32_t) {}
void setPreferredBeeperOffMask(uint32_t) {}

void beeperOffSet(uint32_t) {}
void beeperOffClear(uint32_t) {}
void beeperOffClearAll(void) {}
bool parseColor(int, const char *) {return false; }
void resetEEPROM(void) {}
void bufWriterFlush(bufWriter_t *) {}
void mixerResetDisarmedMotors(void) {}
void gpsEnablePassthrough(struct serialPort_s *) {}
bool parseLedStripConfig(int, const char *){return false; }
const char rcChannelLetters[] = "AERT12345678abcdefgh";

void parseRcChannels(const char *, rxConfig_t *){}
void mixerLoadMix(int, motorMixer_t *) {}
bool setModeColor(ledModeIndex_e, int, int) { return false; }
float convertExternalToMotor(uint16_t ){ return 1.0; }
uint8_t getCurrentPidProfileIndex(void){ return 1; }
uint8_t getCurrentControlRateProfileIndex(void){ return 1; }
void changeControlRateProfile(uint8_t) {}
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *) {}
void writeEEPROM() {}
serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e) {return NULL; }
baudRate_e lookupBaudRateIndex(uint32_t){return BAUD_9600; }
serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e){ return NULL; }
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
void serialSetBaudRate(serialPort_t *, uint32_t) {}
void serialSetMode(serialPort_t *, portMode_e) {}
void serialPassthrough(serialPort_t *, serialPort_t *, serialConsumer *, serialConsumer *) {}
uint32_t millis(void) { return 0; }
uint8_t getBatteryCellCount(void) { return 1; }
void servoMixerLoadMix(int) {}
const char * getBatteryStateString(void){ return "_getBatteryStateString_"; }

uint32_t stackTotalSize(void) { return 0x4000; }
uint32_t stackHighMem(void) { return 0x80000000; }
uint16_t getEEPROMConfigSize(void) { return 1024; }

uint8_t __config_start = 0x00;
uint8_t __config_end = 0x10;
uint16_t averageSystemLoadPercent = 0;

timeDelta_t getTaskDeltaTime(cfTaskId_e){ return 0; }
uint16_t currentRxRefreshRate = 9000;
armingDisableFlags_e getArmingDisableFlags(void) { return ARMING_DISABLED_NO_GYRO; }

const char *armingDisableFlagNames[]= {
"DUMMYDISABLEFLAGNAME"
};

void getTaskInfo(cfTaskId_e, cfTaskInfo_t *) {}
void getCheckFuncInfo(cfCheckFuncInfo_t *) {}
void schedulerResetTaskMaxExecutionTime(cfTaskId_e) {}

const char * const targetName = "UNITTEST";
const char* const buildDate = "Jan 01 2017";
const char * const buildTime = "00:00:00";
const char * const shortGitRevision = "MASTER";


void bufWriterAppend(bufWriter_t *, uint8_t ch){ printf("%c", ch); }
void serialWriteBufShim(void *, const uint8_t *, int) {}
void schedulerSetCalulateTaskStatistics(bool) {}
void setArmingDisabled(armingDisableFlags_e) {}

void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
void stopPwmAllMotors(void) {}
void systemResetToBootloader(bootloaderRequestType_e) {}
void systemReset(void) {}

void changePidProfile(uint8_t) {}
bool serialIsPortAvailable(serialPortIdentifier_e) { return false; }
void generateLedConfig(ledConfig_t *, char *, size_t) {}
bool isSerialTransmitBufferEmpty(const serialPort_t *) {return true; }
void serialWrite(serialPort_t *, uint8_t ch) { printf("%c", ch);}

void serialSetCtrlLineStateCb(serialPort_t *, void (*)(void *, uint16_t ), void *) {}
void serialSetCtrlLineStateDtrPin(serialPort_t *, ioTag_t ) {}
void serialSetCtrlLineState(serialPort_t *, uint16_t ) {}

void serialSetBaudRateCb(serialPort_t *, void (*)(serialPort_t *context, uint32_t baud), serialPort_t *) {}

char *getBoardName(void) { return NULL; };
char *getManufacturerId(void) { return NULL; };
bool boardInformationIsSet(void) { return true; };

bool setBoardName(char *newBoardName) { UNUSED(newBoardName); return true; };
bool setManufacturerId(char *newManufacturerId) { UNUSED(newManufacturerId); return true; };
bool persistBoardInformation(void) { return true; };

mspDescriptor_t mspDescriptorAlloc(void) { return 0; }
}
