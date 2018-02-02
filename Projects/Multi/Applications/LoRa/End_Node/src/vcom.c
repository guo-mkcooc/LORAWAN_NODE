 /******************************************************************************
  * @file    vcom.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    08-September-2017
  * @brief   manages virtual com port
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
#include "hw.h"
#include "vcom.h"
#include <stdarg.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUFSIZE 256
#define RXBUFFERSIZE 128
#define TXBUFFERSIZE 3
#define USARTX_IRQn USART2_IRQn

#define USARTr_IRQn USART1_IRQn
uint8_t aTxBuffer[TXBUFFERSIZE] = {0x43, 0x03, 0x01};

// struct {
//   uint8_t rf[12][12];
//   uint8_t cnt;
//   bool lock;
// } recv_list;

Recv_list_t recv_list;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* buffer */
static char buff[BUFSIZE];
/* buffer write index*/
__IO uint16_t iw=0;
/* buffer read index*/
static uint16_t ir=0;
/* Uart Handle */
static UART_HandleTypeDef UartHandle;

static UART_HandleTypeDef RfidUartHandle;
__IO ITStatus UartReady = RESET;
__IO ITStatus UartTx = RESET;
__IO ITStatus UartRx = RESET;

uint8_t aRxBuffer[RXBUFFERSIZE];
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

void vcom_Init(void)
{
  /*## Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 921600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTX;
  
  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
  
  HAL_NVIC_SetPriority(USARTX_IRQn, 0x1, 0);
  HAL_NVIC_EnableIRQ(USARTX_IRQn);

  recv_list.cnt = 0;
  recv_list.lock = false;
}


void vcom_DeInit(void)
{
#if 1
  HAL_UART_DeInit(&UartHandle);
#endif
}

void vcom_Send( char *format, ... )
{
  va_list args;
  va_start(args, format);
  uint8_t len;
  uint8_t lenTop;
  char tempBuff[128];
  
  BACKUP_PRIMASK();
  DISABLE_IRQ();
  
  /*convert into string at buff[0] of length iw*/
  len = vsprintf(&tempBuff[0], format, args);
  
  if (iw+len<BUFSIZE)
  {
    memcpy( &buff[iw], &tempBuff[0], len);
    iw+=len;
  }
  else
  {
    lenTop=BUFSIZE-iw;
    memcpy( &buff[iw], &tempBuff[0], lenTop);
    len-=lenTop;
    memcpy( &buff[0], &tempBuff[lenTop], len);
    iw = len;
  }
  RESTORE_PRIMASK();
  
  HAL_NVIC_SetPendingIRQ(USARTX_IRQn);
    
  va_end(args);
}

/* modifes only ir*/
void vcom_Print( void)
{
  char* CurChar;
  while( ( (iw+BUFSIZE-ir)%BUFSIZE) >0 )
  {
    BACKUP_PRIMASK();
    DISABLE_IRQ();
    
    CurChar = &buff[ir];
    ir= (ir+1) %BUFSIZE;
    
    RESTORE_PRIMASK();
    
    HAL_UART_Transmit(&UartHandle,(uint8_t *) CurChar, 1, 300);    
  }
  HAL_NVIC_ClearPendingIRQ(USARTX_IRQn);
}

void vcom_Send_Lp( char *format, ... )
{
  va_list args;
  va_start(args, format);
  uint8_t len;
  uint8_t lenTop;
  char tempBuff[128];
  
  BACKUP_PRIMASK();
  DISABLE_IRQ();
  
  /*convert into string at buff[0] of length iw*/
  len = vsprintf(&tempBuff[0], format, args);
  
  if (iw+len<BUFSIZE)
  {
    memcpy( &buff[iw], &tempBuff[0], len);
    iw+=len;
  }
  else
  {
    lenTop=BUFSIZE-iw;
    memcpy( &buff[iw], &tempBuff[0], lenTop);
    len-=lenTop;
    memcpy( &buff[0], &tempBuff[lenTop], len);
    iw = len;
  }
  RESTORE_PRIMASK();  
  
  va_end(args);
}
/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/

  /* Enable USART1 clock */
  USARTX_CLK_ENABLE();
  USARTr_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/  
  vcom_IoInit( );
  rfid_IoInit( );
}

void vcom_IoInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct={0};
    /* Enable GPIO TX/RX clock */
  USARTX_TX_GPIO_CLK_ENABLE();
  USARTX_RX_GPIO_CLK_ENABLE();
    /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTX_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = USARTX_TX_AF;

  HAL_GPIO_Init(USARTX_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTX_RX_PIN;
  GPIO_InitStruct.Alternate = USARTX_RX_AF;

  HAL_GPIO_Init(USARTX_RX_GPIO_PORT, &GPIO_InitStruct);
}

void vcom_IoDeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure={0};
  
  USARTX_TX_GPIO_CLK_ENABLE();
  USARTX_RX_GPIO_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  
  GPIO_InitStructure.Pin =  USARTX_TX_PIN ;
  HAL_GPIO_Init(  USARTX_TX_GPIO_PORT, &GPIO_InitStructure );
  
  GPIO_InitStructure.Pin =  USARTX_RX_PIN ;
  HAL_GPIO_Init(  USARTX_RX_GPIO_PORT, &GPIO_InitStructure ); 
}

/**
  * @brief UART MSP DeInit
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  vcom_IoDeInit( );
  rfid_IoDeInit( );
}

/***********************@BRIEF USART1 INIT ************  rfid reader communication *************/
void led_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, SET);  // green
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);  //  red
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET); // blue

}

void rfid_Init(void)
{
  /*## Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 921600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  RfidUartHandle.Instance = USARTr;

  RfidUartHandle.Init.BaudRate = 115200;
  RfidUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  RfidUartHandle.Init.StopBits = UART_STOPBITS_1;
  RfidUartHandle.Init.Parity = UART_PARITY_NONE;
  RfidUartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  RfidUartHandle.Init.Mode = UART_MODE_TX_RX;

  if (HAL_UART_Init(&RfidUartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  led_init( );
}

void rfid_DeInit(void)
{
#if 1
  HAL_UART_DeInit(&RfidUartHandle);
#endif
}

void rfid_IoInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Enable GPIO TX/RX clock */
  USARTr_TX_GPIO_CLK_ENABLE();
  USARTr_RX_GPIO_CLK_ENABLE();
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTr_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = USARTr_TX_AF;

  HAL_GPIO_Init(USARTr_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTr_RX_PIN;
  GPIO_InitStruct.Alternate = USARTr_RX_AF;

  HAL_GPIO_Init(USARTr_RX_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the NVIC for UART ########################################*/
  /* NVIC for USART */
  HAL_NVIC_SetPriority(USARTr_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(USARTr_IRQn);
  __HAL_UART_ENABLE_IT(&RfidUartHandle, UART_IT_IDLE);
}

void rfid_IoDeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  USARTr_TX_GPIO_CLK_ENABLE();
  USARTr_RX_GPIO_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  GPIO_InitStructure.Pin = USARTr_TX_PIN;
  HAL_GPIO_Init(USARTr_TX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = USARTr_RX_PIN;
  HAL_GPIO_Init(USARTr_RX_GPIO_PORT, &GPIO_InitStructure);
}

void rfid_data_pram(uint8_t *pdata)
{
  uint8_t l = 0;
  uint8_t temp[12];
  uint8_t flagsame = 0;
  if ((pdata[0] != 0x44) || (pdata[1] != 0x16)){
    return ;
  }

  l = pdata[2];
  for (uint8_t i = 0; i < l ; i++){

    memset1((uint8_t *)&temp[0], 0, 12);
    memcpy1((char *)&temp[0], (char *)&pdata[10 + i * 22], 12);

    if (recv_list.lock == false) {
      if (recv_list.cnt == 0) {
        memcpy1((char *)&recv_list.rf[0][0], (char *)&temp[0], 12);
        recv_list.cnt += 1;
      }
      else {
        for (uint8_t p = 0; p < recv_list.cnt; p++){
          if (memcmp((char *)&temp, (char *)&recv_list.rf[p][0], 12) == 0) {
            flagsame = 1;
            }
        }
        if (flagsame != 1) {
          memcpy1(&recv_list.rf[recv_list.cnt][0], &temp[0], 12);
          recv_list.cnt += 1;
        }
        flagsame = 0;
      }
    }
  }

}

void rfid_reader(void)
{
  // __HAL_UART_ENABLE_IT(&RfidUartHandle, UART_IT_IDLE);
  // __HAL_UART_CLEAR_IDLEFLAG(&RfidUartHandle);
  uint32_t crs = 0;
  memset1(aRxBuffer, 0, RXBUFFERSIZE);
  if (HAL_UART_Receive_IT(&RfidUartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
    // vcom_Send("HAL not OK .. \r\n");
    Error_Handler();
  }
  __HAL_UART_CLEAR_IDLEFLAG(&RfidUartHandle);
  // vcom_Send("HAL recv OK .. \r\n");
  while (UartRx != SET)
  {
    if (crs > 1000000){
      crs = 0;
      break;
    }
    crs++;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);
    // vcom_Send("HAL not ready .. \r\n");
  }
  UartRx = RESET;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);

  __HAL_UART_DISABLE_IT(&RfidUartHandle, UART_IT_RXNE);
  __HAL_UART_DISABLE_IT(&RfidUartHandle, UART_IT_PE);
  __HAL_UART_DISABLE_IT(&RfidUartHandle, UART_IT_IDLE);
  __HAL_UART_DISABLE_IT(&RfidUartHandle, UART_IT_ERR);
  RfidUartHandle.State = HAL_UART_STATE_READY;


  vcom_Send("Recving Scandata: [");
  for (uint8_t i = 0; i < RXBUFFERSIZE; i++)
  {
    PRINTF("%02X ", aRxBuffer[i]);
  }
  PRINTF("]\r\n");

  rfid_data_pram((uint8_t *)aRxBuffer);

  vcom_Send("recv %d pramdata: [", recv_list.cnt);
  for (uint8_t i = 0; i < 12; i++)
  {
    for (uint8_t j = 0; j < 12; j++)
    {
      PRINTF("%02X ", recv_list.rf[i][j]);
    }
  }
  PRINTF("]\r\n");

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
}

void rfid_set(void)
{
  if (HAL_UART_Transmit_IT(&RfidUartHandle, (uint8_t *)aTxBuffer, TXBUFFERSIZE) != HAL_OK)
  {
    Error_Handler();
  }
  while (UartTx != SET)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
  UartTx = RESET;
  vcom_Send("Scaning cmd: %02X %02X %02X \r\n", aTxBuffer[0], aTxBuffer[1], aTxBuffer[2]);

}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  vcom_Send("TxcpltCallback .. \r\n");
  UartTx = SET;

}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Set transmission flag: trasfer complete*/
  vcom_Send("RxcpltCallback ..\r\n");
  UartReady = SET;

}

/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles UART interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA
  *         used for USART data transmission
  */
void USARTr_IRQHandler(void)
{

  HAL_UART_IRQHandler(&RfidUartHandle);

  if (__HAL_UART_GET_FLAG(&RfidUartHandle, UART_FLAG_IDLE) != RESET) {
    __HAL_UART_CLEAR_IDLEFLAG(&RfidUartHandle);
    UartRx = SET;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET); // green
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
