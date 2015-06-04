/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * uart_extgps.h - uart driver for deck port
 */
#ifndef UART_EXTGPS_H_
#define UART_EXTGPS_H_

#include <stdbool.h>

#include "crtp.h"
#include "eprintf.h"

#define EXT_UART1               //TX1/RX1 Serial Port on LEFT Side Pins

#if defined (EXT_UART1)         //UART4 does not have flow control
// APB1 - uart4-5 & usart2-3
// APB2 - usart1,6
  #define UARTn_TYPE             UART4
  #define UARTn_PERIF            RCC_APB1Periph_UART4
  #define ENABLE_UARTn_RCC       RCC_APB1PeriphClockCmd
  #define UARTn_IRQ              UART4_IRQn

  #define UARTn_DMA_IRQ          DMA1_Stream2_IRQn
  #define UARTn_DMA_IT_TC        DMA1_IT_TC
  #define UARTn_DMA_STREAM       DMA1_Stream2

  #define UARTn_GPIO_PERIF       RCC_AHB1Periph_GPIOC
  #define UARTn_GPIO_PORT        GPIOC
  #define UARTn_GPIO_TX_PIN      GPIO_Pin_10
  #define UARTn_GPIO_RX_PIN      GPIO_Pin_11
  #define UARTn_GPIO_AF_TX_PIN   GPIO_PinSource10
  #define UARTn_GPIO_AF_RX_PIN   GPIO_PinSource11
  #define UARTn_GPIO_AF_TX       GPIO_AF_UART4
  #define UARTn_GPIO_AF_RX       GPIO_AF_UART4

#else                           //EXT_UART2
// APB1 - uart4-5 & usart2-3
// APB2 - usart1,6
  #define UARTn_TYPE             USART2
  #define UARTn_PERIF            RCC_APB1Periph_USART2
  #define ENABLE_UARTn_RCC       RCC_APB1PeriphClockCmd
  #define UARTn_IRQ              USART2_IRQn

  #define UARTn_DMA_IRQ          DMA1_Stream5_IRQn
  #define UARTn_DMA_IT_TC        DMA1_IT_TC
  #define UARTn_DMA_STREAM       DMA1_Stream5

  #define UARTn_GPIO_PERIF       RCC_AHB1Periph_GPIOA
  #define UARTn_GPIO_PORT        GPIOA
  #define UARTn_GPIO_TX_PIN      GPIO_Pin_2
  #define UARTn_GPIO_RX_PIN      GPIO_Pin_3
  #define UARTn_GPIO_AF_TX_PIN   GPIO_PinSource2
  #define UARTn_GPIO_AF_RX_PIN   GPIO_PinSource3
  #define UARTn_GPIO_AF_TX       GPIO_AF_USART2
  #define UARTn_GPIO_AF_RX       GPIO_AF_USART2

#ifdef UART_SPINLOOP_FLOWCTRL
  #define UARTn_TXEN_PERIF       RCC_AHB1Periph_GPIOB
  #define UARTn_TXEN_PORT        GPIOB
  #define UARTn_TXEN_PIN         GPIO_Pin_4
  #define UARTn_TXEN_EXTI        EXTI_Line4
#endif

#endif

/**
 * Initialize the UART.
 *
 * @note Initialize CRTP link only if USE_CRTP_UART is defined
 */
void uartExtgpsInit(void);  //called by system.c

/**
 * Initialize the UBLOX MAX M8C - assumes 9600 bits/s BaudRate
 * Turn off all NMEA output messages
 */
void uartExtgpsUbxInit(void);  //called by system.c

/**
 * Initialize the UBLOX MAX M8C
 * Enable AssistNow after ColdStart
 * Turn on UBX NAV_PVT output binary messages
 */
void uartExtgpsUbxAssist(void);  //called by system.c

/**
 * Test the UART status.
 *
 * @return true if the UART is initialized
 */
bool uartExtgpsTest(void);  //not yet called by comm.c

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 *
 * @note If UART Crtp link is activated this function does nothing
 */
void uartExtgpsSendData(uint32_t size, uint8_t* data);

/**
 * Interrupt service routine handling UART interrupts.
 */
void uartExtgpsIsr(void); //called by nvic.c

#endif /* UART_EXTGPS_H_ */
