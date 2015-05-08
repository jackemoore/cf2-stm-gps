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
#include <string.h>

/*ST includes */
#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"
//#include "uart_syslink.h"
#include "crtp.h"
#include "cfassert.h"
#include "nvicconf.h"
#include "config.h"
#include "ledseq.h"

#include "uart_extgps.h"
#include "ubx.h"
#include "log.h"
//#include "led.h"  //didn't resolve the LED_GREEN reference

#define UART_DATA_TIMEOUT_MS 1000
#define UART_DATA_TIMEOUT_TICKS (UART_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((uint32_t)0x00000001)

static bool isInit = false;

xSemaphoreHandle ExtgpsWaitUntilSendDone = NULL;
static xQueueHandle ExtgpsUartDataDelivery;

static uint8_t dmaBuffer[64];
static uint8_t *outDataIsr;
static uint8_t dataIndexIsr;
static uint8_t dataSizeIsr;
static bool    isUartDmaInitialized;
static DMA_InitTypeDef DMA_InitStructureShare;
static uint32_t initialDMACount;
static uint32_t remainingDMACount;
static bool     dmaIsPaused;

static void uartExtgpsPauseDma();
static void uartExtgpsResumeDma();

static bool gpsFlag = false;

void uartExtgpsRxTask(void *param); //called by xTaskCreate() below

/**
  * Configures the UART DMA. Mainly used for FreeRTOS trace
  * data transfer.
  */
void uartExtgpsDmaInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  // USART TX DMA Channel Config
  DMA_InitStructureShare.DMA_PeripheralBaseAddr = (uint32_t)&UART_TYPE->DR;
  DMA_InitStructureShare.DMA_Memory0BaseAddr = (uint32_t)dmaBuffer;
  DMA_InitStructureShare.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructureShare.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructureShare.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructureShare.DMA_BufferSize = 0;
  DMA_InitStructureShare.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructureShare.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructureShare.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructureShare.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructureShare.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructureShare.DMA_Priority = DMA_Priority_High;
  DMA_InitStructureShare.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructureShare.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructureShare.DMA_Channel = UART_DMA_CH;

  NVIC_InitStructure.NVIC_IRQChannel = UART_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  isUartDmaInitialized = true;
}

void uartExtgpsInit(void)
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
#ifdef NRF_UART
  EXTI_InitTypeDef extiInit;
#endif

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(UART_GPIO_PERIF, ENABLE); //uart4-5, usart2-3
  ENABLE_UART_RCC(UART_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = UART_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UART_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin   = UART_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(UART_GPIO_PORT, &GPIO_InitStructure);

  //Map uart to alternate functions
  GPIO_PinAFConfig(UART_GPIO_PORT, UART_GPIO_AF_TX_PIN, UART_GPIO_AF_TX);
  GPIO_PinAFConfig(UART_GPIO_PORT, UART_GPIO_AF_RX_PIN, UART_GPIO_AF_RX);

#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)
  USART_InitStructure.USART_BaudRate            = 2000000;
  USART_InitStructure.USART_Mode                = USART_Mode_Tx;
#else
  USART_InitStructure.USART_BaudRate            = 9600; //= 1000000;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
#endif
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
#ifdef NRF_UART
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
#endif
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UART_TYPE, &USART_InitStructure);

#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)
  uartExtgpsDmaInit();
#endif

  // TODO: Enable
  // Configure Tx buffer empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = UART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  vSemaphoreCreateBinary(ExtgpsWaitUntilSendDone);
  ExtgpsUartDataDelivery = xQueueCreate(40, sizeof(uint8_t));

  USART_ITConfig(UART_TYPE, USART_IT_RXNE, ENABLE);

#ifdef NRF_UART
  Setting up TXEN pin (NRF flow control)
  RCC_AHB1PeriphClockCmd(UART_TXEN_PERIF, ENABLE);

  bzero(&GPIO_InitStructure, sizeof(GPIO_InitStructure));
  GPIO_InitStructure.GPIO_Pin = UART_TXEN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UART_TXEN_PORT, &GPIO_InitStructure);

  extiInit.EXTI_Line = UART_TXEN_EXTI;
  extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
  extiInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  extiInit.EXTI_LineCmd = ENABLE;
  EXTI_Init(&extiInit);

  NVIC_EnableIRQ(EXTI4_IRQn);
#endif

  xTaskCreate(uartExtgpsRxTask, (const signed char * const)"UART-Rx",
                configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);

  //Enable UART
  USART_Cmd(UART_TYPE, ENABLE);

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
    while(GPIO_ReadInputDataBit(UART_TXEN_PORT, UART_TXEN_PIN) == Bit_SET);
#endif
    while (!(UART_TYPE->SR & USART_FLAG_TXE));
    UART_TYPE->DR = (data[i] & 0x00FF);
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

void uartExtgpsIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  uint8_t rxDataInterrupt;

  if (USART_GetITStatus(UART_TYPE, USART_IT_TXE))
  {
    if (outDataIsr && (dataIndexIsr < dataSizeIsr))
    {
      USART_SendData(UART_TYPE, outDataIsr[dataIndexIsr] & 0x00FF);
      dataIndexIsr++;
    }
    else
    {
      USART_ITConfig(UART_TYPE, USART_IT_TXE, DISABLE);
      xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(ExtgpsWaitUntilSendDone, &xHigherPriorityTaskWoken);
    }
  }
  USART_ClearITPendingBit(UART_TYPE, USART_IT_TXE);
  if (USART_GetITStatus(UART_TYPE, USART_IT_RXNE))
  {
    rxDataInterrupt = USART_ReceiveData(UART_TYPE) & 0x00FF;
    xQueueSendFromISR(ExtgpsUartDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
}

/******************************GPS End****************************************/

bool uartExtgpsGetDataWithTimout(uint8_t *c)
{
  if (xQueueReceive(ExtgpsUartDataDelivery, c, UART_DATA_TIMEOUT_TICKS) == pdTRUE)
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
  USART_ITConfig(UART_TYPE, USART_IT_TXE, ENABLE);
  xSemaphoreTake(ExtgpsWaitUntilSendDone, portMAX_DELAY);
  outDataIsr = 0;
}

int uartExtgpsPutchar(int ch)
{
    uartExtgpsSendData(1, (uint8_t *)&ch);

    return (unsigned char)ch;
}

void uartExtgpsSendDataDmaBlocking(uint32_t size, uint8_t* data)
{
  if (isUartDmaInitialized)
  {
    xSemaphoreTake(ExtgpsWaitUntilSendDone, portMAX_DELAY);
    // Wait for DMA to be free
    while(DMA_GetCmdStatus(UART_DMA_STREAM) != DISABLE);
    //Copy data in DMA buffer
    memcpy(dmaBuffer, data, size);
    DMA_InitStructureShare.DMA_BufferSize = size;
    initialDMACount = size;
    // Init new DMA stream
    DMA_Init(UART_DMA_STREAM, &DMA_InitStructureShare);
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UART_DMA_STREAM, DMA_IT_TC, ENABLE);
    /* Enable USART DMA TX Requests */
    USART_DMACmd(UART_TYPE, USART_DMAReq_Tx, ENABLE);
    /* Clear transfer complete */
    USART_ClearFlag(UART_TYPE, USART_FLAG_TC);
    /* Enable DMA USART TX Stream */
    DMA_Cmd(UART_DMA_STREAM, ENABLE);
  }
}

static void uartExtgpsPauseDma()
{
  if (DMA_GetCmdStatus(UART_DMA_STREAM) == ENABLE)
  {
    // Disable transfer complete interrupt
    DMA_ITConfig(UART_DMA_STREAM, DMA_IT_TC, DISABLE);
    // Disable stream to pause it
    DMA_Cmd(UART_DMA_STREAM, DISABLE);
    // Wait for it to be disabled
    while(DMA_GetCmdStatus(UART_DMA_STREAM) != DISABLE);
    // Disable transfer complete
    DMA_ClearITPendingBit(UART_DMA_STREAM, UART_DMA_FLAG_TCIF);
    // Read remaining data count
    remainingDMACount = DMA_GetCurrDataCounter(UART_DMA_STREAM);
    dmaIsPaused = true;
  }
}

static void uartExtgpsResumeDma()
{
  if (dmaIsPaused)
  {
    // Update DMA counter
    DMA_SetCurrDataCounter(UART_DMA_STREAM, remainingDMACount);
    // Update memory read address
    UART_DMA_STREAM->M0AR = (uint32_t)&dmaBuffer[initialDMACount - remainingDMACount];
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UART_DMA_STREAM, DMA_IT_TC, ENABLE);
    /* Clear transfer complete */
    USART_ClearFlag(UART_TYPE, USART_FLAG_TC);
    /* Enable DMA USART TX Stream */
    DMA_Cmd(UART_DMA_STREAM, ENABLE);
    dmaIsPaused = false;
  }
}

void uartExtgpsDmaIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(UART_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(UART_DMA_STREAM, UART_DMA_FLAG_TCIF);
  USART_DMACmd(UART_TYPE, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(UART_DMA_STREAM, DISABLE);

  remainingDMACount = 0;
  xSemaphoreGiveFromISR(ExtgpsWaitUntilSendDone, &xHigherPriorityTaskWoken);
}

#ifdef NRF_UART
void uartExtgpsTxenFlowctrlIsr()
{
  EXTI_ClearFlag(UART_TXEN_EXTI);
  if (GPIO_ReadInputDataBit(UART_TXEN_PORT, UART_TXEN_PIN) == Bit_SET)
  {
    uartExtgpsPauseDma();
    //ledSet(LED_GREEN_R, 1);
  }
  else
  {
    uartExtgpsResumeDma();
    //ledSet(LED_GREEN_R, 0);
  }
}
#endif

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
