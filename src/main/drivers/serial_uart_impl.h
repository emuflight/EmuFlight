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

#if defined(STM32H7)
#include "drivers/dma.h"
#endif

// Configuration constants

#if defined(STM32F1)
#define UARTDEV_COUNT_MAX 3
#define UARTHARDWARE_MAX_PINS 3
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     128
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     256
#endif
#elif defined(STM32F3)
#define UARTDEV_COUNT_MAX 5
#define UARTHARDWARE_MAX_PINS 4
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     128
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     256
#endif
#elif defined(STM32F4)
#define UARTDEV_COUNT_MAX 6
#define UARTHARDWARE_MAX_PINS 4
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     128
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     256
#endif
#elif defined(STM32F7) || defined(STM32H7)
#define UARTDEV_COUNT_MAX 8
#if defined(STM32H7)
#define UARTHARDWARE_MAX_PINS 5
#else
#define UARTHARDWARE_MAX_PINS 3
#endif
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     128
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     256
#endif
#else
#error unknown MCU family
#endif

// Count number of configured UARTs

#ifdef USE_UART1
#define UARTDEV_COUNT_1 1
#else
#define UARTDEV_COUNT_1 0
#endif

#ifdef USE_UART2
#define UARTDEV_COUNT_2 1
#else
#define UARTDEV_COUNT_2 0
#endif

#ifdef USE_UART3
#define UARTDEV_COUNT_3 1
#else
#define UARTDEV_COUNT_3 0
#endif

#ifdef USE_UART4
#define UARTDEV_COUNT_4 1
#else
#define UARTDEV_COUNT_4 0
#endif

#ifdef USE_UART5
#define UARTDEV_COUNT_5 1
#else
#define UARTDEV_COUNT_5 0
#endif

#ifdef USE_UART6
#define UARTDEV_COUNT_6 1
#else
#define UARTDEV_COUNT_6 0
#endif

#ifdef USE_UART7
#define UARTDEV_COUNT_7 1
#else
#define UARTDEV_COUNT_7 0
#endif

#ifdef USE_UART8
#define UARTDEV_COUNT_8 1
#else
#define UARTDEV_COUNT_8 0
#endif

#define UARTDEV_COUNT (UARTDEV_COUNT_1 + UARTDEV_COUNT_2 + UARTDEV_COUNT_3 + UARTDEV_COUNT_4 + UARTDEV_COUNT_5 + UARTDEV_COUNT_6 + UARTDEV_COUNT_7 + UARTDEV_COUNT_8)

// Per-pin AF definition needed by H7 UART driver
#if defined(STM32F7) || defined(STM32H7)
typedef struct uartPinDef_s {
    ioTag_t pin;
    uint8_t af;
} uartPinDef_t;
#endif

typedef struct uartHardware_s {
    UARTDevice_e device;    // XXX Not required for full allocation
    USART_TypeDef* reg;
#if defined(STM32F1) || defined(STM32F3)
    DMA_Channel_TypeDef *txDMAChannel;
    DMA_Channel_TypeDef *rxDMAChannel;
#elif defined(STM32F4) || defined(STM32F7)
    uint32_t DMAChannel;
    DMA_Stream_TypeDef *txDMAStream;
    DMA_Stream_TypeDef *rxDMAStream;
#elif defined(STM32H7)
#ifdef USE_DMA
    dmaResource_t *txDMAResource;
    dmaResource_t *rxDMAResource;
    uint32_t txDMAChannel;   // DMAMUX request ID (DMA_REQUEST_USARTx_TX)
    uint32_t rxDMAChannel;   // DMAMUX request ID (DMA_REQUEST_USARTx_RX)
#endif
#endif
#if defined(STM32F7) || defined(STM32H7)
    uartPinDef_t rxPins[UARTHARDWARE_MAX_PINS];
    uartPinDef_t txPins[UARTHARDWARE_MAX_PINS];
#else
    ioTag_t rxPins[UARTHARDWARE_MAX_PINS];
    ioTag_t txPins[UARTHARDWARE_MAX_PINS];
#endif
#if defined(STM32F7)
    uint32_t rcc_ahb1;
    rccPeriphTag_t rcc_apb2;
    rccPeriphTag_t rcc_apb1;
#else
    rccPeriphTag_t rcc;
#endif
#if !defined(STM32F7) && !defined(STM32H7)
    uint8_t af;
#endif
#if defined(STM32F7) || defined(STM32H7)
    uint8_t txIrq;
    uint8_t rxIrq;
#else
    uint8_t irqn;
#endif
    uint8_t txPriority;
    uint8_t rxPriority;
#if defined(STM32H7)
    volatile uint8_t *txBuffer;
    volatile uint8_t *rxBuffer;
    uint16_t txBufferSize;
    uint16_t rxBufferSize;
#endif
} uartHardware_t;

extern const uartHardware_t uartHardware[];

// uartDevice_t is an actual device instance.
// XXX Instances are allocated for uarts defined by USE_UARTx atm.

#if defined(STM32H7)
typedef enum {
    TX_PIN_ACTIVE,
    TX_PIN_MONITOR,
    TX_PIN_IGNORE
} txPinState_t;
#endif

typedef struct uartDevice_s {
    uartPort_t port;
    const uartHardware_t *hardware;
#if defined(STM32H7)
    uartPinDef_t rx;
    uartPinDef_t tx;
    txPinState_t txPinState;
#else
    ioTag_t rx;
    ioTag_t tx;
    volatile uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
    volatile uint8_t txBuffer[UART_TX_BUFFER_SIZE];
#endif
} uartDevice_t;

extern uartDevice_t *uartDevmap[];

extern const struct serialPortVTable uartVTable[];

#ifdef USE_HAL_DRIVER
void uartStartTxDMA(uartPort_t *s);
#else
void uartTryStartTxDMA(uartPort_t *s);
#endif

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options);

void uartIrqHandler(uartPort_t *s);

void uartReconfigure(uartPort_t *uartPort);

#if defined(STM32H7)
struct dmaChannelDescriptor_s;
void uartConfigureDma(uartDevice_t *uartdev);
void uartDmaIrqHandler(struct dmaChannelDescriptor_s *descriptor);
bool checkUsartTxOutput(uartPort_t *s);
void uartTxMonitor(uartPort_t *s);
#endif
