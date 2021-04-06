/************************************************************************************//**
* \file         Demo/ARMCM3_STM32F1_Olimex_STM32P103_GCC/Prog/boot.c
* \brief        Demo program bootloader interface source file.
* \ingroup      Prog_ARMCM3_STM32F1_Olimex_STM32P103_GCC
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2012  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "header.h"                                    /* generic header               */
#include <string.h>

/* Single byte to store input */
char rxBuffer[24];
uint8_t rxByte;
uint16_t rxIdx;
uint8_t rxFlag;

/****************************************************************************************
* Function prototypes
****************************************************************************************/
#if (BOOT_COM_RS232_ENABLE > 0)
static void BootComRs232Init(void);
static void BootComRs232CheckActivationRequest(void);
#endif
#if (BOOT_COM_CAN_ENABLE > 0)
static void BootComCanInit(void);
static void BootComCanCheckActivationRequest(void);
#endif

/************************************************************************************//**
** \brief     Initializes the communication interface.
** \return    none.
**
****************************************************************************************/
void BootComInit(void)
{
#if (BOOT_COM_RS232_ENABLE > 0)
  BootComRs232Init();
#endif
#if (BOOT_COM_CAN_ENABLE > 0)
  BootComCanInit();
#endif
} /*** end of BootComInit ***/


/************************************************************************************//**
** \brief     Receives the CONNECT request from the host, which indicates that the
**            bootloader should be activated and, if so, activates it.
** \return    none.
**
****************************************************************************************/
void BootComCheckActivationRequest(void)
{
#if (BOOT_COM_RS232_ENABLE > 0)
  BootComRs232CheckActivationRequest();
#endif
#if (BOOT_COM_CAN_ENABLE > 0)
  BootComCanCheckActivationRequest();
#endif
} /*** end of BootComCheckActivationRequest ***/



/************************************************************************************//**
** \brief     Bootloader activation function.
** \return    none.
**
****************************************************************************************/
void BootActivate(void)
{
  /* perform software reset to activate the bootoader again */
  NVIC_SystemReset();
} /*** end of BootActivate ***/


#if (BOOT_COM_RS232_ENABLE > 0)
/****************************************************************************************
*     U N I V E R S A L   A S Y N C H R O N O U S   R X   T X   I N T E R F A C E
****************************************************************************************/

/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Timeout time for the reception of a CTO packet. The timer is started upon
 *         reception of the first packet byte.
 */
#define RS232_CTO_RX_PACKET_TIMEOUT_MS (100u)


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief UART handle to be used in API calls. */
static UART_HandleTypeDef rs232Handle;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static unsigned char Rs232ReceiveByte(unsigned char *data);


/************************************************************************************//**
** \brief     Initializes the UART communication interface.
** \return    none.
**
****************************************************************************************/
static void BootComRs232Init(void)
{
  /* Configure UART peripheral. */
  rs232Handle.Instance          = USART2;
  rs232Handle.Init.BaudRate     = BOOT_COM_RS232_BAUDRATE;
  rs232Handle.Init.WordLength   = UART_WORDLENGTH_8B;
  rs232Handle.Init.StopBits     = UART_STOPBITS_1;
  rs232Handle.Init.Parity       = UART_PARITY_NONE;
  rs232Handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  rs232Handle.Init.Mode         = UART_MODE_TX_RX;
  rs232Handle.Init.OverSampling = UART_OVERSAMPLING_16;
  /* Initialize the UART peripheral. */
  HAL_UART_Init(&rs232Handle);
} /*** end of BootComRs232Init ***/


void debugPrint(char _out[]){
  HAL_UART_Transmit(&rs232Handle, (uint8_t *) _out, strlen(_out), 10); 
}

void debugPrintln(char _out[]){
  HAL_UART_Transmit(&rs232Handle, (uint8_t *) _out, strlen(_out), 10); 
  char newline[2] = "\r\n"; 
  HAL_UART_Transmit(&rs232Handle, (uint8_t *) newline, 2, 10); 
}


/* UART2 Interrupt Service Routine */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&rs232Handle);
}

/* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* Transmit one byte with 100 ms timeout */
    //HAL_UART_Transmit(&rs232Handle, &rxByte, 1, 100);

		if(rxByte == 10 || rxIdx >= 23) {
      if ((rxBuffer[0] == 0x4D || rxBuffer[0] == 0x6D) && rxBuffer[1] == 0x39 && rxBuffer[2] == 0x39 && rxBuffer[3] == 0x37)
      {
        debugPrintln("Resetting...");
        HAL_Delay(2000);
        BootActivate();
      } else if ((rxBuffer[0] == 0x4D || rxBuffer[0] == 0x6D) && rxBuffer[1] == 0x31 && rxBuffer[2] == 0x30 && rxBuffer[3] == 0x35)
      {
        debugPrintln("T:19.11 /0.00 B:20.13 /0.00 @:0 B@:0");
      } else {
        debugPrint("echo: ");
        debugPrintln(rxBuffer);
      }
      for(int i=0;i<24;i++) rxBuffer[i] = 0;
			rxIdx = 0;
		}
		else {
			rxBuffer[rxIdx] = rxByte;
			rxIdx++;
		}
    /* Receive one byte in interrupt mode */ 
    HAL_UART_Receive_IT(&rs232Handle, &rxByte, 1);
  }
  
}

void Rs232InterruptInit(void)
{
  HAL_UART_Receive_IT(&rs232Handle, &rxByte, 1);
}

/************************************************************************************//**
** \brief     Receives the CONNECT request from the host, which indicates that the
**            bootloader should be activated and, if so, activates it.
** \return    none.
**
****************************************************************************************/
static void BootComRs232CheckActivationRequest(void)
{
  static unsigned char xcpCtoReqPacket[BOOT_COM_RS232_RX_MAX_DATA+1];
  static unsigned char xcpCtoRxLength;
  static unsigned char xcpCtoRxInProgress = 0;
  static unsigned long xcpCtoRxStartTime = 0;

  /* start of cto packet received? */
  if (xcpCtoRxInProgress == 0)
  {
    /* store the message length when received */
    if (Rs232ReceiveByte(&xcpCtoReqPacket[0]) == 1)
    {
      debugPrintln("Data...");
      /* check that the length has a valid value. it should not be 0 */
      if ( (xcpCtoReqPacket[0] > 0) &&
           (xcpCtoReqPacket[0] <= BOOT_COM_RS232_RX_MAX_DATA) )
      {
        /* store the start time */
        xcpCtoRxStartTime = TimerGet();
        /* indicate that a cto packet is being received */
        xcpCtoRxInProgress = 1;
        /* reset packet data count */
        xcpCtoRxLength = 0;
      }
    }
  }
  else
  {
    /* store the next packet byte */
    if (Rs232ReceiveByte(&xcpCtoReqPacket[xcpCtoRxLength+1]) == 1)
    {

      /* increment the packet data count */
      xcpCtoRxLength++;

      /* check to see if the entire packet was received */
      if (xcpCtoRxLength == xcpCtoReqPacket[0])
      {
        /* done with cto packet reception */
        xcpCtoRxInProgress = 0;

        /* check if this was an XCP CONNECT command */
        if ((xcpCtoReqPacket[1] == 0x0a) && (xcpCtoRxLength == 2))
        {
          /* connection request received so start the bootloader */
          debugPrintln("Resetting...");
          //BootActivate();
        }
      }
    }
    else
    {
      /* check packet reception timeout */
      if (TimerGet() > (xcpCtoRxStartTime + RS232_CTO_RX_PACKET_TIMEOUT_MS))
      {
        /* cancel cto packet reception due to timeout. note that this automatically
         * discards the already received packet bytes, allowing the host to retry.
         */
        xcpCtoRxInProgress = 0;
      }
    }
  }
} /*** end of BootComRs232CheckActivationRequest ***/


/************************************************************************************//**
** \brief     Receives a communication interface byte if one is present.
** \param     data Pointer to byte where the data is to be stored.
** \return    1 if a byte was received, 0 otherwise.
**
****************************************************************************************/
static unsigned char Rs232ReceiveByte(unsigned char *data)
{
  HAL_StatusTypeDef result;

  /* receive a byte in a non-blocking manner */
  result = HAL_UART_Receive(&rs232Handle, data, 1, 0);
  /* process the result */
  if (result == HAL_OK)
  {
    /* success */
    return 1;
  }
  /* error occurred */
  return 0;
} /*** end of Rs232ReceiveByte ***/
#endif /* BOOT_COM_RS232_ENABLE > 0 */



/*********************************** end of boot.c *************************************/
