/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"

#include "main.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

#if (SERIAL_PORT_INSTANCE == 1)
  #define SERIAL_PORT                     USART1
#elif (SERIAL_PORT_INSTANCE == 2)
  #define SERIAL_PORT                     USART2
#endif

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
  if (xRxEnable)
    LL_USART_EnableIT_RXNE(SERIAL_PORT);
  else
    LL_USART_DisableIT_RXNE(SERIAL_PORT);

  if (xTxEnable)
  {
#ifdef RS485_DE_Pin
    LL_GPIO_SetOutputPin(RS485_DE_GPIO_Port, RS485_DE_Pin);
#endif
    LL_USART_EnableIT_TXE(SERIAL_PORT);
  }
  else
    LL_USART_DisableIT_TXE(SERIAL_PORT);
}

// инициализация в uart_init()
BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
    eMBParity eParity)
{
  (void) ucPORT;
  (void) ulBaudRate;
  (void) ucDataBits;
  (void) eParity;
  return TRUE;
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
  /* Put a byte in the UARTs transmit buffer. This function is called
   * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
   * called. */
  LL_USART_TransmitData8(SERIAL_PORT, ucByte);
  return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR *pucByte)
{
  /* Return the byte in the UARTs receive buffer. This function is called
   * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
   */
  *pucByte = LL_USART_ReceiveData8(SERIAL_PORT);
  return TRUE;
}
