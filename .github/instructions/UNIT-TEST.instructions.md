---
name: Unit Test Development Guidelines
description: Best practices and patterns for writing and maintaining unit tests in EmuFlight
applyTo: '**/src/test/unit/*.cc,**/src/test/unit/*.c,**/src/test/unit/*.h'
---

# Unit Test Development Guidelines for EmuFlight

## Core Philosophy

Unit tests in EmuFlight should:
- **Test behavior, not implementation details** — Use behavioral assertions to stay resilient across algorithm/filter changes
- **Be completely isolated** — Each test starts clean with no cross-test interference
- **Have zero production impact** — All test code is wrapped in `#ifdef UNITTEST` or exists only in test files
- **Be self-documenting** — Clear test names and assertions communicate intent without extensive comments

---

## Assertion Patterns

### ✅ Preferred: Behavioral Assertions

Use these patterns to test **what** the code does, not exact magic numbers:

```cpp
// Test responsiveness — value changed from initial state
EXPECT_NE(initialValue, updatedValue);

// Test within tolerance — filters apply effects, use tolerance
EXPECT_NEAR(actual, expected, tolerance);

// Test direction/sign — positive output for positive input
EXPECT_GT(output, 0);
EXPECT_LE(duration, maxTimeMs);

// Test activation/deactivation
EXPECT_TRUE(isActive);
EXPECT_FALSE(isDisabled);
```

**Benefits:**
- Tests survive filter tuning
- Tests survive algorithm improvements
- Tests survive PID constant adjustments
- Clearer intent (what behavior are we validating?)

### ⚠️ Use Sparingly: Exact Value Assertions

Only use `EXPECT_EQ` for **truly immutable values** (constants, protocol values, counts):

```cpp
// ✅ OK — Protocol frame header is always the same
EXPECT_EQ(frameHeader, 0x5A);

// ❌ AVOID — Magic number, will break if filtering changes
EXPECT_EQ(gyroValue, 1234);

// ✅ OK — Count is deterministic
EXPECT_EQ(itemCount, 5);
```

---

## Test Isolation Infrastructure

### Pattern: Fake Time Management

For scheduler-based or timing-sensitive tests:

```cpp
// Global fake time variable (at file scope in extern "C")
static uint32_t fakeMicros = 0;

// Reset at test start to ensure isolation
void resetFakeMicros(void) { 
    fakeMicros = 0; 
}

// Advance between calls to simulate time passage
void testAdvanceMicros(uint32_t delta) { 
    fakeMicros += delta; 
}

// Provide time to firmware
uint32_t micros(void) { 
    return fakeMicros; 
}

// Usage in test:
TEST(MyTest, TimingSensitive) {
    resetFakeMicros();  // Clean state
    
    // First operation
    doSomething();
    
    // Advance time
    testAdvanceMicros(100000);  // 100ms
    
    // Second operation with new time
    doSomethingElse();
    
    EXPECT_NE(state1, state2);
}
```

### Pattern: Struct Initialization

Always explicitly initialize structs to prevent undefined behavior:

```cpp
// ✅ GOOD — Zero-initialized with clear intent
myStruct_t config = {};

// ✅ GOOD — Explicit memset for sentinel values
uint8_t buffer[SIZE];
memset(buffer, 0xFF, sizeof(buffer));  // 0xFF sentinels detect uninitialized

// ❌ AVOID — Assumes uninitialized memory is zero
myStruct_t config;  // Undefined!
```

---

## Stub Implementation Patterns

### Simple Function Stubs

```cpp
// Empty function (for callbacks that aren't tested)
void stubCallback(void) { }

// Function with return value (sensible default)
uint32_t stubGetTime(void) { 
    return 0; 
}

// Function with output parameter
void stubGetMode(mode_t *modeOut) {
    if (modeOut) {
        *modeOut = MODE_IDLE;
    }
}

// Allocation stub
void* stubAlloc(size_t size) { 
    return malloc(size);  // Let malloc fail naturally if needed
}
```

### Conditional/Global Stubs

```cpp
// Define at file scope in extern "C"
static bool g_stubEnabled = true;

uint32_t getValue(void) {
    if (g_stubEnabled) {
        return 42;  // Value for testing
    }
    return realGetValue();  // Fallback
}

// In test:
TEST(MyTest, WithStub) {
    g_stubEnabled = true;
    EXPECT_EQ(getValue(), 42);
}

TEST(MyTest, WithoutStub) {
    g_stubEnabled = false;
    EXPECT_NE(getValue(), 42);
}
```

---

## Test File Structure

### Standard Header Section

```cpp
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

extern "C" {
    #include "platform.h"
    #include "target.h"
    
    // Module under test
    #include "path/to/module.h"
    
    // Stubs and test support
    void requiredStub1(void) { }
    uint32_t requiredStub2(void) { return 0; }
    
    // Global fake time
    static uint32_t fakeMicros = 0;
    void resetFakeMicros(void) { fakeMicros = 0; }
    void testAdvanceMicros(uint32_t delta) { fakeMicros += delta; }
    uint32_t micros(void) { return fakeMicros; }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"
```

### Test Organization

```cpp
// Logical grouping of related tests
TEST(ComponentName, BehaviorWhenX) {
    // Arrange — Setup initial state
    resetFakeMicros();
    component_t comp = {};
    componentInit(&comp);
    
    // Act — Perform operation
    componentDoSomething(&comp, inputValue);
    
    // Assert — Verify behavior
    EXPECT_NE(comp.state, INITIAL_STATE);
    EXPECT_NEAR(comp.output, expectedOutput, 5);
}

TEST(ComponentName, BehaviorWhenY) {
    // Similar structure for different condition
    resetFakeMicros();
    // ... test code ...
}
```

---

## Common Pitfalls and Solutions

### Pitfall: Exact Value Assertions Break on Filter Changes

```cpp
// ❌ BAD — Breaks when Kalman filter tuning changes
TEST(SensorTest, Value) {
    sensor_t s = {};
    sensorInit(&s);
    sensorUpdate(&s, 1000);
    EXPECT_EQ(s.filtered, 987);  // Exact magic number!
}

// ✅ GOOD — Tests behavior, survives filter tuning
TEST(SensorTest, RespondsToInput) {
    sensor_t s = {};
    sensorInit(&s);
    
    uint16_t initial = s.filtered;
    sensorUpdate(&s, 1000);
    uint16_t updated = s.filtered;
    
    EXPECT_NE(initial, updated);  // Value changed
    EXPECT_NEAR(updated, 1000, 50);  // Within reasonable tolerance
}
```

### Pitfall: Uninitialized Test Variables

```cpp
// ❌ BAD — Undefined behavior
TEST(ConfigTest, Load) {
    motorConfig_t config;  // Uninitialized!
    loadMotorConfig(&config);
    EXPECT_EQ(config.pwmFreq, 32);  // Might pass or fail randomly
}

// ✅ GOOD — Explicit initialization
TEST(ConfigTest, Load) {
    motorConfig_t config = {};  // Zero-initialized
    loadMotorConfig(&config);
    EXPECT_EQ(config.pwmFreq, 32);
}
```

### Pitfall: Cross-Test Interference

```cpp
// ❌ BAD — Shared state between tests
static int g_testValue = 0;

TEST(Suite, TestA) {
    g_testValue = 10;
    EXPECT_EQ(g_testValue, 10);
}

TEST(Suite, TestB) {
    EXPECT_EQ(g_testValue, 0);  // May fail if TestA ran first!
}

// ✅ GOOD — Each test resets state
static int g_testValue = 0;

void resetTestState(void) {
    g_testValue = 0;
}

TEST(Suite, TestA) {
    resetTestState();
    g_testValue = 10;
    EXPECT_EQ(g_testValue, 10);
}

TEST(Suite, TestB) {
    resetTestState();
    EXPECT_EQ(g_testValue, 0);
}
```

---

## Type Safety Best Practices

### Explicit Casting for Bit Operations

```cpp
// ❌ BAD — Implicit conversions, possible warnings
uint8_t byte = 0;
byte = telemetryBuf[0] << 8 | telemetryBuf[1];

// ✅ GOOD — Explicit casts show intent
uint16_t value = (uint16_t)telemetryBuf[0] << 8 | (uint16_t)telemetryBuf[1];

// ✅ GOOD — For clarity, assign intermediate
uint32_t combined = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
```

### Named Constants Over Magic Numbers

```cpp
// ❌ BAD — What does 10 mean?
mAhDrawn = testValue / 10;
voltage = testVoltage * 10;

// ✅ GOOD — Intent is clear
constexpr int kMahScale = 10;    // 0.1mAh → 1mAh conversion
constexpr int kVoltScale = 10;   // 0.1V → 0.01V conversion
mAhDrawn = testValue / kMahScale;
voltage = testVoltage * kVoltScale;
```

---

## GHST Telemetry Test Pattern (Advanced)

For tests needing scheduler-driven frame rotation:

```cpp
// Global fake time
static uint32_t fakeMicros = 0;
void resetFakeMicros(void) { fakeMicros = 0; }
void testAdvanceMicros(uint32_t delta) { fakeMicros += delta; }

// Scheduler driver helper
static bool driveGhstUntilTx(int maxTries) {
    for (int i = 0; i < maxTries; ++i) {
        testAdvanceMicros(50000);  // Send frames at 50ms intervals
        processGhst();
        if (ghstGetTelemetryBufLen() > 0) {
            return true;  // Frame transmitted
        }
    }
    return false;  // No frame after max tries
}

TEST(TelemetryGhst, Battery) {
    resetFakeMicros();
    
    // Initialize
    rxRuntimeConfig_t rxState;
    ASSERT_TRUE(ghstRxInit(rxConfig(), &rxState));
    
    // Drive scheduler until battery frame appears
    ASSERT_TRUE(driveGhstUntilTx(10));
    
    // Verify frame
    uint8_t *buf = ghstGetTelemetryBuf();
    EXPECT_EQ(buf[0], GHST_DL_PACK_STAT);
}
```

---

## File Header Template

Use this template for new test files:

```cpp
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

#include <stdint.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "target.h"
    
    #include "module/under/test.h"
    
    // Stubs
    void stubFunction(void) { }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Tests here
```

---

## Checklist for Writing Tests

- [ ] Test name clearly describes **behavior** tested (`TestAddsWhenConditionMet` not `Test1`)
- [ ] Each test is completely independent (reset all state at start)
- [ ] Assertions are behavioral, not exact magic numbers
- [ ] All structs are explicitly zero-initialized
- [ ] Stubs provided for all external dependencies
- [ ] Test isolation infrastructure used (fake time, flags, etc.)
- [ ] Type casts are explicit for bit operations
- [ ] Magic numbers extracted to named constants
- [ ] Test file header is present with copyright
- [ ] Code follows existing test patterns in codebase
- [ ] No warnings on compilation
- [ ] All tests pass when run: `make clean_test && make test .`

---

## When a Test Fails in CI

1. **Check assertions first** — Are they behavioral or exact values?
   - Exact values: Consider relaxing to `EXPECT_NEAR()` with tolerance
   - Behavioral: Verify test isolation (state reset properly)

2. **Check initialization** — All structs must be explicitly zero-initialized

3. **Check test isolation** — Did a previous test leave state behind?
   - Reset globals in test setup
   - Verify `resetFakeMicros()` called if timing-related

4. **Check dependencies** — Are all required stubs present?
   - Search for undefined references in error message
   - Add stub function to test file

5. **Check build environment** — Some tests may be platform-specific
   - CI runs on Ubuntu with clang
   - May need `#ifdef` guards for platform-specific code

---

## References

- GTest documentation: https://google.github.io/googletest/
- EmuFlight test execution: `make clean_test && make test .`
- When adding new tests: Follow pattern from existing tests in same file
- For questions: Check `src/test/unit/*.cc` for working examples

