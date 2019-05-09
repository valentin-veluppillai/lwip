/*
 * uart.c
 *
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
 *  Created on: 19 Mar 2019
 *      Author: val
 */

#include "uart.h"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdarg.h>

UART_HandleTypeDef huart3;
SemaphoreHandle_t mutexSerialCom;
SemaphoreHandle_t mutexUART3;
QueueHandle_t queueRxST, queueTxST;
int flagUSART3TxCplt = 1;
int flagUSART3RxCplt = 0;

char serverLine1[DBG_LINE_SIZE] = "";
char serverLine2[DBG_LINE_SIZE] = "";

void uartTask() {
	uint8_t byteToSend[DBG_LINE_SIZE];
	int waitingBytes = 0;
	while(1) {
		 	if(0 < (waitingBytes = uxQueueMessagesWaiting( queueTxST ))) {
				if(waitingBytes > DBG_LINE_SIZE)	waitingBytes = DBG_LINE_SIZE;
				for(int i = 0; i < waitingBytes; i++)	xQueueReceive(queueTxST, byteToSend + i, 0);
				while(!flagUSART3TxCplt);
				flagUSART3TxCplt = 0;
				HAL_UART_Transmit_IT(&ST_USART, byteToSend, waitingBytes);
		 	}
	}
	vTaskDelete(NULL);
}

void uartInit() {
	mutexSerialCom = xSemaphoreCreateMutex();
	mutexUART3 = xSemaphoreCreateMutex();
	queueRxST = xQueueCreate( 100, 1);
	queueTxST = xQueueCreate( 100, 1);
	xSemaphoreGive(mutexSerialCom);
	LL_USART_EnableIT_RXNE(ST_USART_LL);
}



void commandParserTask(){
	uint8_t byteToProcess;

	uint8_t cmdBuffer[CMD_SIZE];
	uint8_t* cmdPointer = cmdBuffer;
	uint8_t argBuffer[ARG_SIZE][ARG_COUNT];
	uint8_t* argPointer = argBuffer[0];
	int argCounter = 0;
	int cmdCollectFlag = 0;
	int argCollectFlag = 0;
	int cmdCompleteFlag = 0;
	int echoFlag = 1;

	char newline = '\n';
	while(1){
		if(xQueueReceive(queueRxST, &byteToProcess, 100/portTICK_PERIOD_MS) == pdTRUE){
			if(byteToProcess == CMD_CHAR){
				cmdCollectFlag = 1;
				cmdPointer = cmdBuffer;
				//stprint("\ncmdCollect\n");
			}
			else if(cmdCollectFlag){
				if(byteToProcess == SEP_CHAR){
					cmdCollectFlag = 0;
					argCollectFlag = 1;
					argPointer = argBuffer[0];
					*cmdPointer = '\0';
					argCounter = 0;
					//stprint("\nargCollect\n");
				}
				else if(byteToProcess == RETURN_CHAR){
					cmdCollectFlag = 0;
					cmdCompleteFlag = 1;
					*cmdPointer = '\0';
					**argBuffer = '\0';
					//stprint("\ncmdComplete\n");
				}
				else *(cmdPointer++) = byteToProcess;
			}
			else if(argCollectFlag){
				if(byteToProcess == SEP_CHAR){
					argPointer = argBuffer[++argCounter];
				}
				if(byteToProcess == RETURN_CHAR){
					argCollectFlag = 0;
					cmdCompleteFlag = 1;
					*argPointer = '\0';
					//stprint("\ncmdComplete\n");
				}
				else *(argPointer++) = byteToProcess;
			}
			if(cmdCompleteFlag){
				stprint("\nCOMMAND REC\n%s\nARGS\n%s\n", cmdBuffer, argBuffer);
				cmdCompleteFlag = 0;
				if(0 == strcmp(cmdBuffer, "cls")) stclear();
				else if(0 == strcmp(cmdBuffer, "rbt")) { stprint("REBOOTING..."); NVIC_SystemReset(); }
				else if(0 == strcmp(cmdBuffer, "eon")) echoFlag = 1;
				else if(0 == strcmp(cmdBuffer, "eoff")) echoFlag = 0;
				else if(0 == strcmp(cmdBuffer, "sd1")) strcpy(serverLine1, argBuffer);
				else if(0 == strcmp(cmdBuffer, "sd2")) strcpy(serverLine2, argBuffer);
				else stprint("\nUnknown command: %s", cmdBuffer);
			}

			if(echoFlag){
				while(xSemaphoreTake(mutexSerialCom, 100/portTICK_PERIOD_MS) != pdTRUE);
				if(byteToProcess == RETURN_CHAR)	xQueueSendToBack(queueTxST, &newline, NULL);
				else	xQueueSendToBack(queueTxST, &byteToProcess, NULL);
				xSemaphoreGive(mutexSerialCom);
			}
		}
	}
	vTaskDelete(NULL);
}

void stprint(char* format, ...){
	char buffer[DBG_LINE_SIZE];
	va_list args;
	va_start (args, format);
	vsprintf (buffer,format, args);
	while(xSemaphoreTake(mutexSerialCom, 100/portTICK_PERIOD_MS) != pdTRUE);
	for(int i = 0; i < strlen(buffer); i++)
		xQueueSendToBack(queueTxST, buffer + i, NULL);
	xSemaphoreGive(mutexSerialCom);
	va_end (args);
}

void stclear(){
	stprint("\r\033c");
}

void isrprint(char* format, ...){
	char buffer[DBG_LINE_SIZE];
	va_list args;
	va_start (args, format);
	vsprintf (buffer,format, args);
	while(xSemaphoreTakeFromISR(mutexSerialCom, 100/portTICK_PERIOD_MS) != pdTRUE);
	for(int i = 0; i < strlen(buffer); i++)
		xQueueSendToBackFromISR(queueTxST, buffer + i, NULL);
	xSemaphoreGiveFromISR(mutexSerialCom, NULL);
	va_end (args);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	flagUSART3TxCplt = 1;
	HAL_GPIO_WritePin(LDTX_GPIO_Port, LDTX_Pin, GPIO_PIN_RESET);
}

void UART_CharReception_Callback(){
	HAL_GPIO_WritePin(LDRX_GPIO_Port, LDRX_Pin, GPIO_PIN_SET);
	uint8_t byteToSave = LL_USART_ReceiveData8(ST_USART_LL);
	xQueueSendToBackFromISR(queueRxST, &byteToSave, NULL);
	//xQueueSendToBackFromISR(queueTxST, &byteToSave, NULL); //hardcoded echo
}
