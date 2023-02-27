/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved
    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.
    This file is part of the FreeRTOS distribution.
    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.
    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html
    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************
    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wwrong?".  Have you
    defined configASSERT()?
    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.
    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.
    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.
    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.
    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.
    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.
    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.
This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.
This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.
Here is a description of the project's functionality:
The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.
The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html
The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html
An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html
The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.
The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.
The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().
The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100
#define TRAFFIC_GREEN GPIO_Pin_0
#define TRAFFIC_AMBER GPIO_Pin_1
#define TRAFFIC_RED GPIO_Pin_2

#define NEW_CAR 0x80000000
#define QUEUE_MASK   0b11111111000000000000000000000000	// Vehicles behind stop line
#define PAST_MASK    0b00000000111111111110000000000000	// Vehicles past stop line
//#define LIGHT_MASK   0b00000000000000000001110000000000	// Lights (Green, Yellow, Red)
#define VEHICLE_MASK (QUEUE_MASK | PAST_MASK)			// All vehicles


/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static xQueueHandle xQueue = NULL;
static xQueueHandle xFlowQueue = NULL;
static xQueueHandle xTrafficLightQueue = NULL;
SemaphoreHandle_t	xMutexLight;

static void prvSetupHardware( void );
static void prvDisplayBoard( void *pvParameters );
static void prvCarTraffic( void *pvParameters );
static void prvCarTrafficCreator( void *pvParameters );
static void prvTrafficFlow( void *pvParameters );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */

xQueueHandle xQueue_handle = 0;

void writeBoard( uint32_t value )
{
	for(int i = 0; i<32;i++){
			uint32_t nextBit = 0x1 & (value >> i);
			GPIO_Write(GPIOB, nextBit);
			GPIO_ToggleBits(GPIOB, GPIO_Pin_1);
		}
}

uint16_t get_ADC_Value() {
	uint16_t adc_value;
	uint16_t converted_value;
	ADC_SoftwareStartConv(ADC1);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	adc_value = ADC_GetConversionValue(ADC1);
	converted_value = adc_value / 1000;

	return converted_value;
}

uint16_t changeLight(uint16_t light) {
	uint16_t nextLight;

	if(light == TRAFFIC_GREEN) {
		nextLight = TRAFFIC_AMBER;
	} else if(light == TRAFFIC_AMBER) {
		nextLight = TRAFFIC_RED;
	} else if(light == TRAFFIC_RED) {
		nextLight = TRAFFIC_GREEN;
	}

	return nextLight;
}

uint32_t advanceCars(int newCar, uint32_t boardState) {
	if(newCar == 1) {
		return 0x00000000 | (VEHICLE_MASK & (boardState >> 1)) | NEW_CAR;
	}

	return 0x00000000 | (VEHICLE_MASK & (boardState >> 1));
}

uint32_t stopAdvanceCars(int newCar, uint32_t boardState) {
		// Find rightmost 0 in vehicles before light
		// Set all leftwards bits to 1s
		uint32_t stopLoc = (boardState | ~QUEUE_MASK);

		// Finds lest significant zero
		stopLoc = ~stopLoc & (stopLoc+1);

		// Get value of position
		stopLoc = (int)log2(stopLoc) & 0xFF;
		if (stopLoc == 0) {
			stopLoc = 32;
		}

		// Create mask for just those vehicles before the light with room to move ahead
		uint32_t dynMask = 0x00000000;
		for (int i=0; i < 32-stopLoc; i++) {
			dynMask = ((dynMask >> 1) | 0x80000000);
		}

		if(newCar == 1) {
			return 0x00000000
							| ((dynMask & boardState) >> 1) 				// Shifts unblocked vehicles
							| (~dynMask & QUEUE_MASK & boardState)			// Places blocked vehicles
							| (PAST_MASK & ((PAST_MASK & boardState) >> 1)) // Shifts vehicles that are past the stop line
							| NEW_CAR;										// Places new vehicle
		}

		// Build new boardState
		return 0x00000000
				| ((dynMask & boardState) >> 1) 				// Shifts unblocked vehicles
				| (~dynMask & QUEUE_MASK & boardState)			// Places blocked vehicles
				| (PAST_MASK & ((PAST_MASK & boardState) >> 1)); // Shifts vehicles that are past the stop line
}

/*-----------------------------------------------------------*/

int main(void)
{
	prvSetupHardware();
	xMutexLight = xSemaphoreCreateMutex();
	    if( xMutexLight == NULL )
	    {
	        printf("ERROR: LIGHT SEMAPHORE NOT CREATED. \n"); 	/* There was insufficient FreeRTOS heap available for the semaphore to be created successfully. */
	    }
	    else
	    {
	    	xSemaphoreGive( xMutexLight ); // need to give semaphore after it is defined
	    }
	xQueue = xQueueCreate( 	mainQUEUE_LENGTH,			/* The number of items the queue can hold. */
								sizeof( uint32_t ) );		/* The size of each item the queue holds. */
	xFlowQueue = xQueueCreate( 	mainQUEUE_LENGTH,			/* The number of items the queue can hold. */
									sizeof( uint16_t ) );		/* The size of each item the queue holds. */
	xTrafficLightQueue = xQueueCreate( 	mainQUEUE_LENGTH,			/* The number of items the queue can hold. */
										sizeof( uint16_t ) );		/* The size of each item the queue holds. */
	vQueueAddToRegistry( xQueue, "MainQueue" );
	vQueueAddToRegistry( xFlowQueue, "FlowQueue" );
	u_int32_t defaultBoardState = (0xC0000000);
	uint16_t defaultFlow = 0;
	uint16_t defaultLight = TRAFFIC_RED;

	xQueueSend( xTrafficLightQueue, &defaultLight, 0);
	xQueueSend( xQueue, &defaultBoardState, 0);
	xQueueSend( xFlowQueue, &defaultFlow, 0);
	xTaskCreate( prvTrafficFlow, "TrafficFlow", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( prvCarTrafficCreator, "CarTrafficCreator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( prvDisplayBoard, "DisplayBoardTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	vTaskStartScheduler();
	return 0;
}

static void prvCarTrafficCreator(void *pvParameters) {
	uint16_t flow = 4;
	uint32_t boardstate;
	uint16_t light;
	int newCar = 0;

	while(1) {
		vTaskDelay(500);
		xQueueReceive(xFlowQueue, &flow, portMAX_DELAY);
		xQueueReceive(xQueue, &boardstate, portMAX_DELAY);
		xQueueReceive(xTrafficLightQueue, &light, portMAX_DELAY);


		int prob = rand() % 4;
		//car_value = (rand() % 100) < 100/(4 - flow);
//		if(prob < flow) {
//			boardstate = advanceCars(1, boardstate);
//		} else {
//			boardstate = advanceCars(0, boardstate);
//		}

		if(light == TRAFFIC_GREEN) {
			boardstate = advanceCars(newCar, boardstate);
		} else {
			boardstate = stopAdvanceCars(newCar, boardstate);
		}


//		boardstate = advanceCars(0, boardstate);

		xQueueSend(xFlowQueue, &flow, 0);
		xQueueSend(xQueue, &boardstate, 0);
		xQueueSend(xTrafficLightQueue, &light, 0);
	}
}

static void prvTrafficFlow(void *pvParameters) {
	uint16_t flow_value = 0;
	u_int16_t ulReceivedValue;

	while(1) {
		vTaskDelay(250);
		xQueueReceive( xFlowQueue, &ulReceivedValue, portMAX_DELAY );
		flow_value = get_ADC_Value();
		xQueueSend(xFlowQueue, &flow_value, 0);
	}
}

static void

static void prvTrafficLight(void *pvParameters) {
	xTimerHandle xTrafficLightTimer = NULL;

	xTrafficLightTimer = xTimerCreate("LightTimer")
}


static void prvDisplayBoard(void *pvParameters) {
	uint32_t ulReceivedValue;

	while(1) {
		// Run every 10mx

		vTaskDelay(100);
//		GPIO_SetBits(GPIOD, TRAFFIC_GREEN);
//		GPIO_SetBits(GPIOD, TRAFFIC_AMBER);
//		GPIO_SetBits(GPIOD, TRAFFIC_RED);
//		GPIO_SetBits(GPIOD, )

		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

		// Write to board
		writeBoard(ulReceivedValue);

		xQueueSend( xQueue, &ulReceivedValue, 0);
	}
}

void vGreentLightTimer(xTimerHandle xTimer) {
	GPIO_ResetBits(GPIOD, TRAFFIC_GREEN);
	GPIO_SetBits(GPIOD, TRAFFIC_AMBER);
	
	if(xSemaphoreTake(xMutexLight, (TickType_t)0) == pdTRUE) {
		xQueueSend
		xSemaphoreGive(xMutexLight);
	}
}

/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.
	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.
	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	GPIO_InitTypeDef Shift1;
	GPIO_InitTypeDef Traffic_Lights;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);


	Shift1.GPIO_Mode = GPIO_Mode_OUT;
	Shift1.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//	Shift1.GPIO_OType = GPIO_OType_PP;
	Shift1.GPIO_PuPd = GPIO_PuPd_DOWN;
//	Shift1.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &Shift1);

	Traffic_Lights.GPIO_Mode = GPIO_Mode_OUT;
	Traffic_Lights.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	Traffic_Lights.GPIO_OType = GPIO_OType_PP;
	Traffic_Lights.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &Traffic_Lights);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//Enable clock for ADC Ports
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//Enable GPIO pin to read analog input

	GPIO_InitTypeDef GPIO_initStructre;
	GPIO_initStructre.GPIO_Pin = GPIO_Pin_1; // Provide input to channel 10 of ADC i.e GPIO Pin 0 of Port C
	GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //GPIO Pin as analog Mode
	GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_initStructre);// GPIO Initialization

	ADC_DeInit();
	ADC_InitTypeDef adc;

	adc.ADC_ContinuousConvMode = ENABLE;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_ExternalTrigConv = ENABLE;
	adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	adc.ADC_NbrOfConversion = 1;
	adc.ADC_Resolution = ADC_Resolution_12b;
	adc.ADC_ScanConvMode = ENABLE;

	ADC_Init(ADC1, &adc);
	ADC_Cmd(ADC1, ENABLE);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);
}
