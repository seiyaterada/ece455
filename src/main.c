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

#include "ddTaskHeader.h"

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100

#define DD_PRIORITY_UNSCHEDULED 1

#define TASK_1_EXEC_TIME 95
#define TASK_1_PERIOD 500

#define TASK_2_EXEC_TIME 150
#define TASK_2_PERIOD 500

#define TASK_3_EXEC_TIME 250
#define TASK_3_PERIOD 750

static void Periodic_Task_1( void *pvParameters );
static void Task_1_Generator( void *pvParameters );
static void DD_Scheduler( void *pvParameters );
static void Monitor_Task( void *pvParameters );

static dd_task_list active;
static dd_task_list overdue;
static dd_task_list complete;


static xQueueHandle xMessageQueue = NULL;
static xQueueHandle xMonitorQueue = NULL;
xQueueHandle xQueue_handle = 0;


TaskHandle_t Periodic_task_gen_handle_1;
/*-----------------------------------------------------------*/

int main(void)
{
	create_task_list(&active);
	create_task_list(&overdue);
	create_task_list(&complete);

	xMessageQueue = xQueueCreate(5, sizeof( message ));
	vQueueAddToRegistry(xMessageQueue, "MessageQueue");
	xMonitorQueue = xQueueCreate(5, sizeof( message ));
	vQueueAddToRegistry(xMonitorQueue, "MonitorQueue");

	xTaskCreate(DD_Scheduler, "DD Scheduler Task", configMINIMAL_STACK_SIZE, NULL, DD_TASK_PRIORITY_SCHEDULER, NULL);
	xTaskCreate(Monitor_Task, "Monitor Task", configMINIMAL_STACK_SIZE, NULL, DD_TASK_PRIORITY_MONITOR, NULL);
	xTaskCreate(Task_1_Generator, "Periodic Task 1", configMINIMAL_STACK_SIZE, NULL,DD_TASK_PRIORITY_GENERATOR, &Periodic_task_gen_handle_1);

	vTaskStartScheduler();
	return 0;
}


/*-----------------------------------------------------------*/

uint32_t dd_create_task(dd_task_node task) {
	printf("DD_CREATE\n");
	if(task == NULL) {
		printf("ERROR: task null");
		return 0;
	}

	xTaskCreate(task->t_function,
				task->task_name,
				configMINIMAL_STACK_SIZE,
				(void*)task,
				DD_TASK_PRIORITY_MINIMUM,
				&(task->t_handle)
	);

	vTaskSuspend(task->t_handle);

	message newMessage = {
			CREATE_TASK,
			xTaskGetCurrentTaskHandle(),
			task
	};

	if(xMessageQueue != NULL) {
		if(xQueueSend(xMessageQueue, &newMessage, portMAX_DELAY) != pdPASS) {
			printf("ERROR: Couldn't send request");
			return 0;
		}
	} else {
		printf("ERROR: Queue does not exist");
		return 0;
	}

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	vTaskResume(task->t_handle);

	return 1;
}

/*-----------------------------------------------------------*/

uint32_t dd_delete_task(uint32_t task_id) {
	printf("DD_DELETE\n");
	if(task_id == 0) {
		printf("ERROR: Null ID");
		return 0;
	}

	message newMessage = {
			DELETE_TASK,
			task_id,
			NULL
	};


	if(xMessageQueue != NULL) {
			if(xQueueSend(xMessageQueue, &newMessage, portMAX_DELAY) != pdPASS) {
				printf("ERROR: Couldn't send request");
				return 0;
			}
		} else {
			printf("ERROR: Queue does not exist");
			return 0;
	}

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//	vTaskDelete(task->t_handle);

	return 1;
}


/*-----------------------------------------------------------*/

uint32_t dd_return_active_list( void )
{
	printf("DD_ACTIVE_LIST\n");
	message newMessage = {
			GET_ACTIVE,
			NULL,
			NULL
	};

	if(xMessageQueue != NULL) {
		if(xQueueSend(xMessageQueue, &newMessage, portMAX_DELAY) != pdPASS) {
			printf("ERROR: Couldn't send request");
			return 0;
		}
	} else {
		printf("ERROR: Queue does not exist");
		return 0;
	}

	if(xMonitorQueue != NULL) {
		if(xQueueReceive(xMonitorQueue, &newMessage, portMAX_DELAY) == pdTRUE) {
			printf("Active: \n%s\n", (char*)(newMessage.data));
			vPortFree(newMessage.data);
			newMessage.data = NULL;
		}
	}

	return 1;
}

/*-----------------------------------------------------------*/

uint32_t dd_return_overdue_list( void  )
{
	printf("DD_OVERDUE_LIST\n");
	message newMessage = {
				GET_OVERDUE,
				NULL,
				NULL
		};

		if(xMessageQueue != NULL) {
			if(xQueueSend(xMessageQueue, &newMessage, portMAX_DELAY) != pdPASS) {
				printf("ERROR: Couldn't send request");
				return 0;
			}
		} else {
			printf("ERROR: Queue does not exist");
			return 0;
		}

		if(xMonitorQueue != NULL) {
			if(xQueueReceive(xMonitorQueue, &newMessage, portMAX_DELAY) == pdTRUE) {
				printf("Overdue: \n%s\n,", (char*)(newMessage.data));
				vPortFree(newMessage.data);
				newMessage.data = NULL;
			}
		}

		return 1;
}

uint32_t dd_return_complete_list( void )
{
	printf("DD_COMPLETE_LIST\n");
	message newMessage = {
				GET_COMPLETE,
				NULL,
				NULL
		};

		if(xMessageQueue != NULL) {
			if(xQueueSend(xMessageQueue, &newMessage, portMAX_DELAY) != pdPASS) {
				printf("ERROR: Couldn't send request");
				return 0;
			}
		} else {
			printf("ERROR: Queue does not exist");
			return 0;
		}

		if(xMonitorQueue != NULL) {
			if(xQueueReceive(xMonitorQueue, &newMessage, portMAX_DELAY) == pdTRUE) {
				printf("Complete: \n%s\n,", (char*)(newMessage.data));
				vPortFree(newMessage.data);
				newMessage.data = NULL;
			}
		}

		return 1;
}


/*-----------------------------------------------------------*/

static void DD_Scheduler( void *pvParameters )
{
	message res_message;
	dd_task_node task_node = NULL;

	printf("Start Scheduler\n");
	while(1) {
		if(xQueueReceive(xMessageQueue, (void*)&res_message, portMAX_DELAY) == pdTRUE) {
			printf("New message\n");
			transfer_overdue_list(&active, &overdue);

			while(overdue.list_length > 5) {
				printf("\nRemoving items from overdue\n");
				remove_head(&overdue);
			}

			switch(res_message.type) {
				case(CREATE_TASK): {
					printf("Create Task\n");
					task_node = (dd_task_node)res_message.data;
					insert(task_node, &active);

					xTaskNotifyGive(res_message.sender);
					break;
				}

				case(DELETE_TASK): {
					printf("Delete Task\n");
					task_node = (dd_task_node)res_message.data;
					insert(task_node, &complete);
					removeNode(task_node->task_id, &active, false);

					free_node(task_node);

					xTaskNotifyGive(res_message.sender);
					break;
				}

				case(GET_ACTIVE): {
					printf("Get active Task\n");
					res_message.data = (void*)format_list(&active);

//					if(uxQueueSpacesAvailable(xMonitorQueue) == 0) {
//						xQueueReset(xMonitorQueue);
//					}

					if(xMonitorQueue != NULL) {
						if(xQueueSend(xMonitorQueue, &res_message, (TickType_t) portMAX_DELAY) != pdPASS) {
							printf("ERROR: couldn't send request");
							break;
						}
					}

					break;
				}

				case(GET_OVERDUE): {
					printf("Get overdue Task\n");
					res_message.data = (void*)format_list(&overdue);

					if(uxQueueSpacesAvailable(xMonitorQueue) == 0) {
						xQueueReset(xMonitorQueue);
					}

					if(xMonitorQueue != NULL) {
						if(xQueueSend(xMonitorQueue, &res_message, (TickType_t) portMAX_DELAY) != pdPASS) {
							printf("ERROR: couldn't send request");
							break;
						}
					}

					break;
				}

				case(GET_COMPLETE): {
					printf("Get complete Task\n");
					res_message.data = (void*)format_list(&complete);

					if(uxQueueSpacesAvailable(xMonitorQueue) == 0) {
						xQueueReset(xMonitorQueue);
					}

					if(xMonitorQueue != NULL) {
						if(xQueueSend(xMonitorQueue, &res_message, (TickType_t) portMAX_DELAY) != pdPASS) {
							printf("ERROR: couldn't send request");
							break;
						}
					}

					break;
				}
			}
		}
	}
}


static void Monitor_Task( void *pvParameters )
{
	printf("Start Monitor Task\n");
//	vTaskDelay(1000);
	while(1) {
//		printf("\nMonitoring Task: Current Time = %u, Priority = %u\n", (unsigned int)xTaskGetTickCount(), (unsigned int)uxTaskPriorityGet( NULL ));
		dd_return_active_list();
		dd_return_overdue_list();
		dd_return_complete_list();

		vTaskDelay(100);
	}
}

static void Task_1_Generator( void *pvParameters )
{
	printf("Start Task1\n");
	while (1) {

    dd_task_node task = allocate();

    task->t_function = Periodic_Task_1;
    task->task_name = "Periodic_Task_1";
    task->type = PERIODIC;

    TickType_t time = xTaskGetTickCount();
    task->release_time = time;
    task->absolute_deadline = time + TASK_1_PERIOD;

    dd_create_task(task);

    // Delay THIS generator task. Not the created one
    vTaskDelay(TASK_1_PERIOD);
  }
}

void Periodic_Task_1(void *pvParameters) {
  printf("In Periodic task 1\n");
  TickType_t current_tick = xTaskGetTickCount();
  TickType_t previous_tick = 0;
  TickType_t execution_time = TASK_1_EXEC_TIME / portTICK_PERIOD_MS;
//  printf("Task 1 (F-Task Priority: %u; Tick: %u)\n",
//         (unsigned int)uxTaskPriorityGet(NULL), (unsigned int)current_tick);
  while (execution_time) {
    current_tick = xTaskGetTickCount();
    if (current_tick == previous_tick)
      continue;
    previous_tick = current_tick;
    execution_time--;
  }
  // There's no need to vTaskDelay here. If we're done we're done.
  dd_delete_task(1);
}


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

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}
