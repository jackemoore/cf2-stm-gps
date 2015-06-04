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
 * uart.c - uart CRTP link and raw access functions
 */
#define DEBUG_MODULE "GPS"

#include <string.h>

/*ST includes */
#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "semphr.h"
#include "queue.h"

#include "config.h"
//#include "uart_syslink.h"
#include "crtp.h"
#include "cfassert.h"
#include "nvicconf.h"
#include "config.h"
#include "ledseq.h"

#include "console.h"
//#include "ablock.h"

#include "uart_extgps.h"
#include "ubx.h"
#include "log.h"
//#include "led.h"  //didn't resolve the LED_GREEN reference

#define UARTn_DATA_TIMEOUT_MS 1000
#define UARTn_DATA_TIMEOUT_TICKS (UARTn_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((uint32_t)0x00000001)

static bool isInit = false;

xSemaphoreHandle ExtgpsWaitUntilSendDone = NULL;
static xQueueHandle ExtgpsUartDataDelivery;

//static uint8_t dmaBuffer[64];
static uint8_t *outDataIsr;
static uint8_t dataIndexIsr;
static uint8_t dataSizeIsr;

static bool gpsFlag = false;

void uartExtgpsRxTask(void *param); //called by xTaskCreate() below


void uartExtgpsInit(void)
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(UARTn_GPIO_PERIF, ENABLE); //uart4-5, usart2-3
  ENABLE_UARTn_RCC(UARTn_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = UARTn_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UARTn_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin   = UARTn_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(UARTn_GPIO_PORT, &GPIO_InitStructure);

  //Map uart to alternate functions
  GPIO_PinAFConfig(UARTn_GPIO_PORT, UARTn_GPIO_AF_TX_PIN, UARTn_GPIO_AF_TX);
  GPIO_PinAFConfig(UARTn_GPIO_PORT, UARTn_GPIO_AF_RX_PIN, UARTn_GPIO_AF_RX);

  USART_InitStructure.USART_BaudRate            = 9600; //= 1000000;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
#ifdef UART_SPINLOOP_FLOWCTRL
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
#else
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
#endif
  USART_Init(UARTn_TYPE, &USART_InitStructure);

  // TODO: Enable
  // Configure Tx buffer empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = UARTn_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  vSemaphoreCreateBinary(ExtgpsWaitUntilSendDone);
  ExtgpsUartDataDelivery = xQueueCreate(40, sizeof(uint8_t));

  USART_ITConfig(UARTn_TYPE, USART_IT_RXNE, ENABLE);

  xTaskCreate(uartExtgpsRxTask, (const signed char * const)"UART-Rx",
                configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);

  //Enable UART
  USART_Cmd(UARTn_TYPE, ENABLE);

  isInit = true;
}

bool uartExtgpsTest(void)
{
  return isInit;
}

/*****************************GPS Start***************************************/

static char uartExtgpsGetc()
{
  char c;
  xQueueReceive(ExtgpsUartDataDelivery, &c, portMAX_DELAY);
  return c;
}

static void uartExtgpsRead(void *buffer, int length)
{
  int i;

  for (i=0; i<length; i++)
  {
    ((char*)buffer)[i] = uartExtgpsGetc();
  }
}

static void uartExtgpsReceiveUbx(struct ubx_message* msg, int maxPayload)
{
    bool received = false;
    uint8_t c;

    while (!received)
    {
        if ((c = uartExtgpsGetc()) != 0xb5)
            continue;
        if ((uint8_t)uartExtgpsGetc() != 0x62)
            continue;

        msg->class = uartExtgpsGetc();
        msg->id = uartExtgpsGetc();
        uartExtgpsRead(&msg->len, 2);

        if(msg->len > maxPayload)
          continue;

        uartExtgpsRead(msg->payload, msg->len);
        msg->ck_a = uartExtgpsGetc();
        msg->ck_b = uartExtgpsGetc();

        received = true;
    }
}

static uint8_t gps_fixType;
static int32_t gps_lat;
static int32_t gps_lon;
static int32_t gps_hMSL;
static uint32_t gps_hAcc;
static int32_t gps_gSpeed;
static int32_t gps_heading;

void uartExtgpsRxTask(void *param)
{
  struct ubx_message msg;
  char payload[100];
//  int value = 0;

  msg.payload = payload;

  //gpsFlag += 1;

  while(1)
  {
    uartExtgpsReceiveUbx(&msg, 100);

    if (msg.class_id == NAV_PVT) {
      gps_fixType = msg.nav_pvt->fixType;
      gps_lat = msg.nav_pvt->lat;
      gps_lon = msg.nav_pvt->lon;
      gps_hMSL = msg.nav_pvt->hMSL;
      gps_hAcc = msg.nav_pvt->hAcc;
      gps_gSpeed = msg.nav_pvt->gSpeed;
      gps_heading = msg.nav_pvt->heading;

    }

    //ledseqRun(LED_GREEN, seq_linkup); //TODO
    //ledSet(LED_GREEN_R, 0);  //TODO

    //Almanac Generation/Upload & requires ablock.h, ablock.c,
    //ablockWatchdog();

  }
}

void uartExtgpsSendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
#ifdef UART_SPINLOOP_FLOWCTRL
    while(GPIO_ReadInputDataBit(UARTn_TXEN_PORT, UARTn_TXEN_PIN) == Bit_SET);
#endif
    while (!(UARTn_TYPE->SR & USART_FLAG_TXE));
    UARTn_TYPE->DR = (data[i] & 0x00FF);
  }
}

void uartExtgpsUbxInit(void)
{
  const char raw_0_setserial[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};
  const char raw_1_enpvt[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1};

  //gpsFlag = true;

  //vTaskDelay(2000);

  uartExtgpsSendData(sizeof(raw_0_setserial), (uint8_t*)raw_0_setserial);
  vTaskDelay(1000);

  uartExtgpsSendData(sizeof(raw_1_enpvt), (uint8_t*)raw_1_enpvt);

  //gpsFlag = true;

}

void uartExtgpsUbxAssist(void)
{
//	ablockPuts("start0\n");
}

void uartExtgpsIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  uint8_t rxDataInterrupt;

  if (USART_GetITStatus(UARTn_TYPE, USART_IT_TXE))
  {
    if (outDataIsr && (dataIndexIsr < dataSizeIsr))
    {
      USART_SendData(UARTn_TYPE, outDataIsr[dataIndexIsr] & 0x00FF);
      dataIndexIsr++;
    }
    else
    {
      USART_ITConfig(UARTn_TYPE, USART_IT_TXE, DISABLE);
      xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(ExtgpsWaitUntilSendDone, &xHigherPriorityTaskWoken);
    }
  }
  USART_ClearITPendingBit(UARTn_TYPE, USART_IT_TXE);
  if (USART_GetITStatus(UARTn_TYPE, USART_IT_RXNE))
  {
    rxDataInterrupt = USART_ReceiveData(UARTn_TYPE) & 0x00FF;
    xQueueSendFromISR(ExtgpsUartDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
}

bool uartExtgpsGetDataWithTimout(uint8_t *c)
{
  if (xQueueReceive(ExtgpsUartDataDelivery, c, UARTn_DATA_TIMEOUT_TICKS) == pdTRUE)
  {
    return true;
  }
  return false;
}

void uartExtgpsSendDataIsrBlocking(uint32_t size, uint8_t* data)
{
  outDataIsr = data;
  dataSizeIsr = size;
  dataIndexIsr = 1;
  uartExtgpsSendData(1, &data[0]);
  USART_ITConfig(UARTn_TYPE, USART_IT_TXE, ENABLE);
  xSemaphoreTake(ExtgpsWaitUntilSendDone, portMAX_DELAY);
  outDataIsr = 0;
}

int uartExtgpsPutchar(int ch)
{
    uartExtgpsSendData(1, (uint8_t *)&ch);

    return (unsigned char)ch;
}

/* Loggable variables */
LOG_GROUP_START(extgps)
LOG_ADD(LOG_INT8, gpsFlag, &gpsFlag)
LOG_GROUP_STOP(extgps)

LOG_GROUP_START(gps)
LOG_ADD(LOG_UINT8, fixType, &gps_fixType)
LOG_ADD(LOG_INT32, lat, &gps_lat)
LOG_ADD(LOG_INT32, lon, &gps_lon)
LOG_ADD(LOG_INT32, hMSL, &gps_hMSL)
LOG_ADD(LOG_UINT32, hAcc, &gps_hAcc)
LOG_ADD(LOG_INT32, gSpeed, &gps_gSpeed)
LOG_ADD(LOG_INT32, heading, &gps_heading)
LOG_GROUP_STOP(gps)
