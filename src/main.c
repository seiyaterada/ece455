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

// Test 1
#define TASK_1_EXEC_TIME 95
#define TASK_1_PERIOD 500

#define TASK_2_EXEC_TIME 150
#define TASK_2_PERIOD 500

#define TASK_3_EXEC_TIME 250
#define TASK_3_PERIOD 750

// Test 2
//#define TASK_1_EXEC_TIME 95
//#define TASK_1_PERIOD 250
//
//#define TASK_2_EXEC_TIME 150
//#define TASK_2_PERIOD 500
//
//#define TASK_3_EXEC_TIME 250
//#define TASK_3_PERIOD 750

// Test 3
//#define TASK_1_EXEC_TIME 100
//#define TASK_1_PERIOD 500
//
//#define TASK_2_EXEC_TIME 200
//#define TASK_2_PERIOD 500
//
//#define TASK_3_EXEC_TIME 200
//#define TASK_3_PERIOD 500

static void Periodic_Task_1( void *pvParameters );
static void Task_1_Generator( void *pvParameters );
static void Periodic_Task_2( void *pvParameters );
static void Task_2_Generator( void *pvParameters );
static void Periodic_Task_3( void *pvParameters );
static void Task_3_Generator( void *pvParameters );
static void DD_Scheduler( void *pvParameters );
static void Monitor_Task( void *pvParameters );

static void prvSetupHardware(void);

static dd_task_list active;
static dd_task_list overdue;
static dd_task_list complete;


static xQueueHandle xMessageQueue = NULL;
static xQueueHandle xMonitorQueue = NULL;
static xQueueHandle xCreateQueue = NULL;
static xQueueHandle xDeleteQueue = NULL;
xQueueHandle xQueue_handle = 0;


TaskHandle_t Periodic_task_gen_handle_1;
TaskHandle_t Periodic_task_gen_handle_2;
TaskHandle_t Periodic_task_gen_handle_3;
/*-----------------------------------------------------------*/

int main(void)
{
	prvSetupHardware();

	// Create task lists
	create_task_list(&active);
	create_task_list(&overdue);
	create_task_list(&complete);

	// Create queues for messages, monitor, create, and delete
	xMessageQueue = xQueueCreate(10, sizeof( message ));
	vQueueAddToRegistry(xMessageQueue, "MessageQueue");
	xMonitorQueue = xQueueCreate(2, sizeof( message ));
	vQueueAddToRegistry(xMonitorQueue, "MonitorQueue");
	xCreateQueue = xQueueCreate(10, sizeof( message ));
	vQueueAddToRegistry(xCreateQueue, "CreateQueue");
	xDeleteQueue = xQueueCreate(10, sizeof( message ));
	vQueueAddToRegistry(xDeleteQueue, "DeleteQueue");

	// Create DDS, Monitor and Periodic Tasks
	xTaskCreate(DD_Scheduler, "DD Scheduler Task", configMINIMAL_STACK_SIZE, NULL, DD_TASK_PRIORITY_SCHEDULER, NULL);
	xTaskCreate(Monitor_Task, "Monitor Task", configMINIMAL_STACK_SIZE, NULL, DD_TASK_PRIORITY_MONITOR, NULL);
	xTaskCreate(Task_1_Generator, "Periodic Task 1", configMINIMAL_STACK_SIZE, NULL,DD_TASK_PRIORITY_GENERATOR, &Periodic_task_gen_handle_1);
	xTaskCreate(Task_2_Generator, "Periodic Task 2", configMINIMAL_STACK_SIZE, NULL,DD_TASK_PRIORITY_GENERATOR, &Periodic_task_gen_handle_2);
	xTaskCreate(Task_3_Generator, "Periodic Task 3", configMINIMAL_STACK_SIZE, NULL,DD_TASK_PRIORITY_GENERATOR, &Periodic_task_gen_handle_3);

	vTaskStartScheduler();
	return 0;
}


/*-----------------------------------------------------------*/

uint32_t dd_create_task(dd_task_node task) {
	// Check if the task is null
	if(task == NULL) {
		printf("ERROR: task null");
		return 0;
	}

	// Create the task
	xTaskCreate(task->t_function,
				task->task_name,
				configMINIMAL_STACK_SIZE,
				(void*)task,
				DD_TASK_PRIORITY_MINIMUM,
				&(task->t_handle)
	);

	// Check that the task was created successfully
	if(task->t_handle == NULL) {
		printf("ERROR: CREATE task error\n");
		return 0;
	}

	// Suspend task until DDS resumes again
	vTaskSuspend(task->t_handle);

	// Create message to be sent to DDS
	message newMessage = {
			CREATE_TASK,
			xTaskGetCurrentTaskHandle(),
			task
	};

	// Send message to DDS through message queue
	if(xMessageQueue != NULL) {
		if(xQueueSend(xMessageQueue, &newMessage, portMAX_DELAY) != pdPASS) {
			printf("ERROR: Couldn't send request");
			return 0;
		}
	} else {
		printf("ERROR: Queue does not exist");
		return 0;
	}

	// Get response back from DDS and resume task
	if(xCreateQueue != NULL) {
		if(xQueueReceive(xCreateQueue, &newMessage, portMAX_DELAY) == pdTRUE) {
			vTaskResume(task->t_handle);
			return 1;
		}
	}


	return 0;
}

/*-----------------------------------------------------------*/

uint32_t dd_delete_task(uint32_t task_id) {
	// Check if task Id is null
	if(task_id == 0) {
		printf("ERROR: Null ID");
		return 0;
	}

	// Create message to be sent to DDS
	message newMessage = {
			DELETE_TASK,
			&task_id,
			task_id
	};


	// Send message to DDS through message queue
	if(xMessageQueue != NULL) {
			if(xQueueSend(xMessageQueue, &newMessage, portMAX_DELAY) != pdPASS) {
				printf("ERROR: Couldn't send request");
				return 0;
			}
		} else {
			printf("ERROR: Queue does not exist");
			return 0;
	}

	// Wait for DDS to receive response
	if(xDeleteQueue != NULL) {
		if(xQueueReceive(xDeleteQueue, &newMessage, portMAX_DELAY) == pdTRUE) {
			return 1;
		}
	}

	return 0;
}


/*-----------------------------------------------------------*/

uint32_t dd_return_active_list( void )
{
	// Create message to send to DDS
	message newMessage = {
			GET_ACTIVE,
			NULL,
			NULL
	};

	// Send message to DDS through message queue
	if(xMessageQueue != NULL) {
		if(xQueueSend(xMessageQueue, &newMessage, portMAX_DELAY) != pdPASS) {
			printf("ERROR: Couldn't send request");
			return 0;
		}
	} else {
		printf("ERROR: Queue does not exist");
		return 0;
	}

	// Wait to receive response from DDS then print active list
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
	// Create message to send to DDS
	message newMessage = {
				GET_OVERDUE,
				NULL,
				NULL
		};

	// Send message to DDS through message queue
	if(xMessageQueue != NULL) {
		if(xQueueSend(xMessageQueue, &newMessage, portMAX_DELAY) != pdPASS) {
			printf("ERROR: Couldn't send request");
			return 0;
		}
	} else {
		printf("ERROR: Queue does not exist");
		return 0;
	}

	// Wait for response from DDS then print overdue list
	if(xMonitorQueue != NULL) {
		if(xQueueReceive(xMonitorQueue, &newMessage, portMAX_DELAY) == pdTRUE) {
			printf("Overdue: \n%s\n", (char*)(newMessage.data));
			vPortFree(newMessage.data);
			newMessage.data = NULL;
		}
	}

	return 1;
}

uint32_t dd_return_complete_list( void )
{
	// Create message to send to DDS
	message newMessage = {
				GET_COMPLETE,
				NULL,
				NULL
		};

	// Send message to DDS through message queue
	if(xMessageQueue != NULL) {
		if(xQueueSend(xMessageQueue, &newMessage, portMAX_DELAY) != pdPASS) {
			printf("ERROR: Couldn't send request");
			return 0;
		}
	} else {
		printf("ERROR: Queue does not exist");
		return 0;
	}

	// Wait for response from DDS then print complete list
	if(xMonitorQueue != NULL) {
		if(xQueueReceive(xMonitorQueue, &newMessage, portMAX_DELAY) == pdTRUE) {
			printf("Complete: \n%s\n", (char*)(newMessage.data));
			vPortFree(newMessage.data);
			newMessage.data = NULL;
		}
	}

	return 1;
}


/*-----------------------------------------------------------*/

static void DD_Scheduler( void *pvParameters )
{
	// Create response message and task node variables
	message res_message;
	dd_task_node task_node = NULL;

	printf("Start Scheduler\n");
	while(1) {
		// Wait for message to arrive
		if(xQueueReceive(xMessageQueue, (void*)&res_message, portMAX_DELAY) == pdTRUE) {
			// Transfer any overdue tasks from active to overdue if there are any
			transfer_overdue_list(&active, &overdue);

			// Only keep the most recent 5 tasks in overdue list
			while(overdue.list_length > 5) {
				printf("\nRemoving items from overdue\n");
				remove_head(&overdue);
			}

			// Check what kind of message it received
			switch(res_message.type) {
				case(CREATE_TASK): {
					// Get task node from message
					task_node = (dd_task_node)res_message.data;
					// Insert task into correct spot in active list
					insert(task_node, &active);

					// Send response back to dd_create through create queue
					if(xCreateQueue != NULL) {
						if(xQueueSend(xCreateQueue, &res_message, (TickType_t) portMAX_DELAY) != pdPASS) {
							printf("ERROR: couldn't send request");
							break;
						}
					}
					break;
				}

				case(DELETE_TASK): {
					// Get task ID from message
					uint32_t task_id = (uint32_t)res_message.data; // Data is task_id
					// Find the task node from the active list by looking at ID
					task_node = findNode(task_id, &active);
					if(task_node->task_id == 0) {
						break;
					}
					// Remove the node from the active list and set priorities accordingly
					removeNode(task_node->task_id, &active, false);

					// Get current time and make that the completion time for current task
					TickType_t current_time = xTaskGetTickCount();
					task_node->completion_time = current_time;

					// Insert the task into the completed list
					insert(task_node, &complete);

					// Delete task
					vTaskDelete(task_node->t_handle);

//					free_node(task_node);

					// Send response back to dd_create via delete queue
					if(xDeleteQueue != NULL) {
						if(xQueueSend(xDeleteQueue, &res_message, (TickType_t) portMAX_DELAY) != pdPASS) {
							printf("ERROR: couldn't send request");
							break;
						}
					}
					break;
				}

				case(GET_ACTIVE): {
					// Store formatted list in message data
					res_message.data = (void*)format_list(&active);

					// Check if there is enough room in queue, if not reset
					if(uxQueueSpacesAvailable(xMonitorQueue) == 0) {
						xQueueReset(xMonitorQueue);
					}

					// Send back formatted list to dd task to print list
					if(xMonitorQueue != NULL) {
						if(xQueueSend(xMonitorQueue, &res_message, (TickType_t) portMAX_DELAY) != pdPASS) {
							printf("ERROR: couldn't send request");
							break;
						}
					}

					break;
				}

				case(GET_OVERDUE): {
					// Store formatted list in message data
					res_message.data = (void*)format_list(&overdue);

					// Check if there is enough room in queue, if not reset
					if(uxQueueSpacesAvailable(xMonitorQueue) == 0) {
						xQueueReset(xMonitorQueue);
					}

					// Send back formatted list to dd task to print list
					if(xMonitorQueue != NULL) {
						if(xQueueSend(xMonitorQueue, &res_message, (TickType_t) portMAX_DELAY) != pdPASS) {
							printf("ERROR: couldn't send request");
							break;
						}
					}

					break;
				}

				case(GET_COMPLETE): {
					// Store formatted list in message data
					res_message.data = (void*)format_complete_list(&complete);

					// Check if there is enough room in queue, if not reset
					if(uxQueueSpacesAvailable(xMonitorQueue) == 0) {
						xQueueReset(xMonitorQueue);
					}

					// Send back formatted list to dd task to print lists
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
		// Call the dd list tasks to print the lists
		dd_return_active_list();
		dd_return_overdue_list();
		dd_return_complete_list();

		vTaskDelay(100);
	}
}

static void Task_1_Generator( void *pvParameters )
{
	while (1) {

	// Allocate space for new task
    dd_task_node task = allocate_node();
    TickType_t deadline = TASK_1_PERIOD;

    // Define task variables
    task->t_function = Periodic_Task_1;
    task->task_name = "1_Periodic_Task";
    task->type = PERIODIC;
    task->task_id = 1;

    // Get current time for release and deadline times
    TickType_t current_time = xTaskGetTickCount();
    task->release_time = current_time;
    task->absolute_deadline = current_time + deadline;

    // Start the dd task
    dd_create_task(task);

    // Delay THIS generator task. Not the created one
    vTaskDelay(TASK_1_PERIOD);
  }
}

void Periodic_Task_1(void *pvParameters) {
  // Initialize times to track period for task 1
  dd_task_node self = (dd_task_node)pvParameters;
  TickType_t current_tick = 0;
  TickType_t previous_tick = 0;
  TickType_t execution_time = TASK_1_EXEC_TIME / portTICK_PERIOD_MS;
  TickType_t relative_deadline = 0;
  TickType_t start_time = 0;
  uint32_t execution_counter = 0;

  while(1) {
	  current_tick = xTaskGetTickCount();
	  previous_tick = current_tick;
	  start_time = current_tick;
	  execution_counter = 0;

	  // Mock execution of task incrementing the counter until it reaches the execution time
	  while(execution_counter < execution_time) {
		  current_tick = xTaskGetTickCount();
		  if(current_tick != previous_tick) {
			  execution_counter++;
		  }
		  previous_tick = current_tick;
	  }
//	  relative_deadline = self->absolute_deadline - current_tick;
//	  if(relative_deadline != 0) {
//		  vTaskDelayUntil(&current_tick, relative_deadline);
//	  }
	  // Start dd task
	  dd_delete_task(1);
  }
  // There's no need to vTaskDelay here. If we're done we're done.
}

static void Task_2_Generator( void *pvParameters )
{
	while (1) {

    dd_task_node task = allocate_node();
    TickType_t deadline = TASK_2_PERIOD;

    task->t_function = Periodic_Task_2;
    task->task_name = "2_Periodic_Task";
    task->type = PERIODIC;
    task->task_id = 2;

    TickType_t current_time = xTaskGetTickCount();
    task->release_time = current_time;
    task->absolute_deadline = current_time + deadline;

    dd_create_task(task);

    // Delay THIS generator task. Not the created one
    vTaskDelay(TASK_2_PERIOD);
  }
}

void Periodic_Task_2(void *pvParameters) {
  dd_task_node self = (dd_task_node)pvParameters;
  TickType_t current_tick = 0;
  TickType_t previous_tick = 0;
  TickType_t execution_time = TASK_2_EXEC_TIME / portTICK_PERIOD_MS;
  TickType_t relative_deadline = 0;
  TickType_t start_time = 0;
  uint32_t execution_counter = 0;

  while(1) {
	  current_tick = xTaskGetTickCount();
	  previous_tick = current_tick;
	  start_time = current_tick;
	  execution_counter = 0;

	  while(execution_counter < execution_time) {
		  current_tick = xTaskGetTickCount();
		  if(current_tick != previous_tick) {
			  execution_counter++;
		  }
		  previous_tick = current_tick;
	  }
//	  relative_deadline =self->absolute_deadline - current_tick;
//	  if(relative_deadline != 0) {
//		  vTaskDelayUntil(&current_tick, relative_deadline);
//	  }
	  dd_delete_task(2);
  }
  // There's no need to vTaskDelay here. If we're done we're done.
}

static void Task_3_Generator( void *pvParameters )
{
	while (1) {

    dd_task_node task = allocate_node();
    TickType_t deadline = TASK_3_PERIOD;

    task->t_function = Periodic_Task_3;
    task->task_name = "3_Periodic_Task";
    task->type = PERIODIC;
    task->task_id = 3;

    TickType_t current_time = xTaskGetTickCount();
    task->release_time = current_time;
    task->absolute_deadline = current_time + deadline;

    dd_create_task(task);

    // Delay THIS generator task. Not the created one
    vTaskDelay(TASK_3_PERIOD);
  }
}

void Periodic_Task_3(void *pvParameters) {
  dd_task_node self = (dd_task_node)pvParameters;
  TickType_t current_tick = 0;
  TickType_t previous_tick = 0;
  TickType_t execution_time = TASK_3_EXEC_TIME / portTICK_PERIOD_MS;
  TickType_t relative_deadline = 0;
  TickType_t start_time = 0;
  uint32_t execution_counter = 0;

  while(1) {
	  current_tick = xTaskGetTickCount();
	  previous_tick = current_tick;
	  start_time = current_tick;
	  execution_counter = 0;

	  while(execution_counter < execution_time) {
		  current_tick = xTaskGetTickCount();
		  if(current_tick != previous_tick) {
			  execution_counter++;
		  }
		  previous_tick = current_tick;
	  }
//	  relative_deadline =self->absolute_deadline - current_tick;
//	  if(relative_deadline != 0) {
//		  vTaskDelayUntil(&current_tick, relative_deadline);
//	  }
	  dd_delete_task(3);
  }
  // There's no need to vTaskDelay here. If we're done we're done.
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
