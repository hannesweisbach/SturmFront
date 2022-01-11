/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef PCA10040_H
#define PCA10040_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LEDS_NUMBER    1

#define LED_START      25
#define LED_1          25
#define LED_STOP       25

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1 }

#define BSP_LED_0      LED_1

#define BUTTONS_NUMBER 1

#define BUTTON_START   28
#define BUTTON_1       28
#define BUTTON_STOP    28
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define RX_PIN_NUMBER  14
#define TX_PIN_NUMBER  15
#define HWFC           false

//#define SPIS_MISO_PIN   28  // SPI MISO signal.
//#define SPIS_CSN_PIN    12  // SPI CSN signal.
//#define SPIS_MOSI_PIN   25  // SPI MOSI signal.
//#define SPIS_SCK_PIN    29  // SPI SCK signal.
//
//#define SPIM0_SCK_PIN   29  // SPI clock GPIO pin number.
//#define SPIM0_MOSI_PIN  25  // SPI Master Out Slave In GPIO pin number.
//#define SPIM0_MISO_PIN  28  // SPI Master In Slave Out GPIO pin number.
//#define SPIM0_SS_PIN    12  // SPI Slave Select GPIO pin number.
//
//#define SPIM1_SCK_PIN   2   // SPI clock GPIO pin number.
//#define SPIM1_MOSI_PIN  3   // SPI Master Out Slave In GPIO pin number.
//#define SPIM1_MISO_PIN  4   // SPI Master In Slave Out GPIO pin number.
//#define SPIM1_SS_PIN    5   // SPI Slave Select GPIO pin number.
//
//#define SPIM2_SCK_PIN   12  // SPI clock GPIO pin number.
//#define SPIM2_MOSI_PIN  13  // SPI Master Out Slave In GPIO pin number.
//#define SPIM2_MISO_PIN  14  // SPI Master In Slave Out GPIO pin number.
//#define SPIM2_SS_PIN    15  // SPI Slave Select GPIO pin number.


#ifdef __cplusplus
}
#endif

#endif // PCA10040_H
