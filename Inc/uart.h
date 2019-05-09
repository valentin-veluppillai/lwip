/*
 * uart.h
 *	simple usart interface code for stm32
 *	desigend to be used with freertos
 *	v 1.0
 *
 *	Usage:
 *
 *	Things to do in cubemx:
 *	initialize your uart (make sure to set the ST_USART and ST_USART_LL accordingly
 *	initialize freertos
 *	activate the usart interrupt in nvic
 *
 *	delete the line declaring huartx, i declare it in uart.c (x is an number, default is 3)
 *
 *	include uart.h in main.h
 *
 *	add this to your variable declaration
 *	########
#ifdef UART_H_
extern SemaphoreHandle_t mutexSerialCom;
extern SemaphoreHandle_t mutexUART3;
extern QueueHandle_t queueRxST, queueTxST;
extern UART_HandleTypeDef huart3;
#endif
 *	########
 *	add the following code to USARTx_IRQ_Handler() in stm32#xx_it.c
 *	########
if(LL_USART_IsActiveFlag_RXNE(ST_USART_LL) && LL_USART_IsEnabledIT_RXNE(ST_USART_LL))
	UART_CharReception_Callback();
HAL_UART_IRQHandler(&huart3);
 *	########
 *	Add this to your startup task
 *	########
xTaskCreate(uartTask, "rxtxST", 100, NULL, 10, NULL);
xTaskCreate(commandParserTask, "commandParser", 100+ARG_SIZE*ARG_COUNT+CMD_SIZE, NULL, 10, NULL);
uartInit();
stprint("\n\r\033cBOOTED\n------\n");
 *	########
 *  Created on: 12 Mar 2019
 *      Author: val
 */

#ifndef UART_H_
#define UART_H_

#include "main.h"

#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_usart.h"
#include "stm32f7xx_ll_pwr.h"

#define ST_USART huart3
#define ST_USART_LL USART3
#define DBG_LINE_SIZE 32
#define RETURN_CHAR '\r'
#define CMD_CHAR '#'
#define SEP_CHAR ' '
#define CMD_COUNT 2
#define CMD_SIZE 5
#define ARG_COUNT 1
#define ARG_SIZE 20

void stprint(char* format, ...);
void stclear();
void isrprint(char* format, ...);
void uartTask();
void commandParserTask();
void uartInit();
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void UART_CharReception_Callback();

#endif /* UART_H_ */
