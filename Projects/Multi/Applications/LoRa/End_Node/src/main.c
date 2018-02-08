 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic lora driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    08-September-2017
  * @brief   this is the main!
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

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*!
 * CAYENNE_LPP is myDevices Application server.
 */
//#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73

#define LPP_APP_PORT 99

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1
/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG                    DISABLE
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            3

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static RfidState_t RfidState = RFID_STATE_INIT;

static TimerEvent_t RfidScanTimer;

/* call back when LoRa will transmit a frame*/
static void LoraTxData( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed);

/* call back when LoRa has received a frame*/
static void LoraRxData( lora_AppData_t *AppData);

/* Private variables ---------------------------------------------------------*/
/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LoraTxData,
                                               LoraRxData};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;

extern Recv_list_t recv_list;

#define RFID_SCAN_TIME        100   //100 ms

#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent( void );
#endif
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {TX_ON_TIMER,
                                    APP_TX_DUTYCYCLE,
                                    CLASS_A,
                                    LORAWAN_ADR_ON,
                                    DR_0,
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

void OnRfidScanTimerEvent(void)
{
  // PRINTF("DEVICE OnRfidScanTimerEvent : [");
  TimerStop(&RfidScanTimer);
  PRINTF("DEVICE OnRfidScanTimerEvent rlist %d : [", recv_list.cnt);
  for (uint8_t i = 0; i < 5; i++)
  {
    for (uint8_t j = 0; j < 12; j++) {
      PRINTF("%02X ", recv_list.rf[i][j]);
    }
  }
  PRINTF("]\r\n");

  RfidState = RFID_STATE_SCANING;

}

void rfid_fsm(void)
{
  switch (RfidState){
    case RFID_STATE_INIT:
    {
      TimerInit(&RfidScanTimer, OnRfidScanTimerEvent);

      RfidState = RFID_STATE_SCANING;
      break;
    }
    case RFID_STATE_SCANING:
    {

      rfid_set( );
      PRINTF("rfid_readering  ..\r\n");
      rfid_reader( );
      TimerSetValue(&RfidScanTimer, RFID_SCAN_TIME); /* 1s */

      TimerStart(&RfidScanTimer);

      PRINTF("rfid_reader  done ..\r\n");
      RfidState = RFID_STATE_IDLE;
      break;
    }
    case RFID_STATE_IDLE:
    {
      break;
    }
  }
}

int main( void )
{
  /* STM32 HAL library initialization*/
  HAL_Init( );
  
  /* Configure the system clock*/
  SystemClock_Config( );

  /* Configure the debug mode*/
  DBG_Init( );
  
  /* Configure the hardware*/
  HW_Init( );
  
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  
  /* Configure the Lora Stack*/
  lora_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  PRINTF("VERSION: %X\n\r", VERSION);

  // rfid_set();
  // PRINTF("rfid_reader ..\r\n");
  // rfid_reader( );

  /* main loop*/

  PRINTF("loop .. \r\n");
  // int cl = 0;
  while( 1 )
  {

    rfid_fsm();
    // cl++;
    // if (cl == 1000000) {
    //   PRINTF("LOOP ...... \r\n");
    //   cl =0;
    // }
    /* run the LoRa class A state machine*/
    lora_fsm( );
    
//     DISABLE_IRQ( );
//     /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
//      * and cortex will not enter low power anyway  */
//     if ( lora_getDeviceState( ) == DEVICE_STATE_SLEEP )
//     {
// #ifndef LOW_POWER_DISABLE
//       LowPower_Handler( );
// #endif
//     }
//     ENABLE_IRQ();
    
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}

static void LoraTxData( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed)
{
  PRINTF("LoRaTxData \n\r");
  /* USER CODE BEGIN 3 */
  
  uint8_t i = 0;
  recv_list.lock = true;

  uint8_t cnt = recv_list.cnt;
  if ((cnt != 0) && (cnt < 7)) {
    AppData->Buff[i++] = 0xC3;
    AppData->Buff[i++] = cnt;
    for (uint8_t c = 0; c < cnt; c++){
      // memcpy1(AppData->Buff[i], &recv_list.rf[c][0], 12);

        for (uint8_t j = 0; j < 12; j++)
        {
          AppData->Buff[i++] = recv_list.rf[c][j];
        }

    }

    for (uint8_t p = 0; p < 12; p++)
    {
      for (uint8_t j = 0; j < 12; j++)
      {
        recv_list.rf[p][j] = 0;
      }
    }
    AppData->BuffSize = i;

    recv_list.cnt = 0;

  }

  if (cnt >= 7) {
    AppData->Buff[i++] = 0xC3;
    AppData->Buff[i++] = 6;
    for (uint8_t c = 0; c < 6; c++)
    {
      // memcpy1((uint8_t *)AppData->Buff[i], &recv_list.rf[c][0], 12);
      // i += 12;
      for (uint8_t j = 0; j < 12; j++)
      {
        AppData->Buff[i++] = recv_list.rf[c][j];
      }
    }

    AppData->BuffSize = i;

    // memcpy1(&recv_list.rf[0], &recv_list.rf[6], (cnt - 6) * 12);
    for (uint8_t p = 0; p < cnt - 6; p++)
    {
      for (uint8_t j = 0; j < 12; j++)
      {
        recv_list.rf[p][j] = recv_list.rf[p+6][j];
      }
    }

    for (uint8_t p = 1; p < 7; p++) {
      for(uint8_t j = 0; j < 12; j++) {
        recv_list.rf[cnt-p][j] = 0;
      }
    }

    recv_list.cnt = cnt - 6;
  }

  if (cnt == 0) {
    AppData->Buff[i++] = 'P';
    AppData->Buff[i++] = 'I';
    AppData->Buff[i++] = 'N';
    AppData->Buff[i++] = 'G';

    AppData->BuffSize = i;
  }

  AppData->Port = LORAWAN_APP_PORT;

  *IsTxConfirmed = LORAWAN_CONFIRMED_MSG;

  recv_list.lock = false;
  /* USER CODE END 3 */
  PRINTF("*****LoRatx: %d port: %d  [", AppData->BuffSize, AppData->Port);
  for (uint8_t p = 0; p < i; p++){
    PRINTF("%02X ", AppData->Buff[p]);
  }
  PRINTF("]\r\n");
}

static void LoraRxData( lora_AppData_t *AppData )
{
  /* USER CODE BEGIN 4 */
  switch (AppData->Port)
  {
  case LORAWAN_APP_PORT:
    if( AppData->BuffSize == 1 )
    {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        LED_Off( LED_BLUE ) ; 
        
      }
      else
      {
        PRINTF("LED ON\n\r");
        LED_On( LED_BLUE ) ; 
      }
      //GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
    }
    break;
  case LPP_APP_PORT:
  {
    AppLedStateOn= (AppData->Buff[2] == 100) ?  0x01 : 0x00;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        LED_Off( LED_BLUE ) ; 
        
      }
      else
      {
        PRINTF("LED ON\n\r");
        LED_On( LED_BLUE ) ; 
      }
    break;
  }
  default:
    break;
  }
  /* USER CODE END 4 */
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent( void )
{
  LED_Off( LED_RED1 ) ; 
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
