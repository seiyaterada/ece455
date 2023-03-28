/*
 * ddTaskHeader.h
 *
 *  Created on: Mar 14, 2023
 *      Author: kaidenrivett
 */

#ifndef DDTASKHEADER_H_
#define DDTASKHEADER_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

# define DD_TASK_PRIORITY_IDLE           (0)
# define DD_TASK_PRIORITY_MINIMUM        (1)
# define DD_TASK_PRIORITY_MONITOR        (4)
# define DD_TASK_PRIORITY_EXECUTION_BASE (3)
# define DD_TASK_PRIORITY_HIGH			 (4)
# define DD_TASK_PRIORITY_GENERATOR      ( configMAX_PRIORITIES - 3 )
# define DD_TIMER_PRIORITY               ( configMAX_PRIORITIES - 2 )
# define DD_TASK_PRIORITY_SCHEDULER      ( configMAX_PRIORITIES - 1 ) // set to the highest priority, defined in FreeRTOSConfig.h
# define DD_TASK_RANGE (DD_TASK_PRIORITY_SCHEDULER - 1 - DD_TASK_PRIORITY_EXECUTION_BASE)

enum task_type {UNDEFINED, PERIODIC,APERIODIC};
enum message_type {CREATE_TASK, DELETE_TASK, GET_ACTIVE, GET_COMPLETE, GET_OVERDUE};

typedef struct dd_task {
	TaskHandle_t t_handle;
	TaskFunction_t t_function;
	enum task_type type;
	uint32_t task_id;
	const char* task_name;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
	struct dd_task* next;
	struct dd_task* prev;
} dd_task;

typedef dd_task* dd_task_node;

typedef struct dd_task_list {
	uint32_t list_length;
	dd_task_node head;
	dd_task_node tail;
} dd_task_list;

typedef dd_task_list* dd_task_list_node;

typedef struct message {
	enum message_type type;
	TaskHandle_t sender;
	void *data;
} message;

dd_task_node allocate_node();

bool free_node(dd_task_node node);


void create_task_list(dd_task_list_node task_list);

void insert(dd_task_node task, dd_task_list_node task_list);

//void insert_complete(dd_task_node task, dd_task_list_node task_list);

dd_task_node findNode(uint32_t taskId, dd_task_list_node task_list);

void removeNode(uint32_t taskId, dd_task_list_node task_list, bool clear);

void remove_head(dd_task_list_node task_list);

void transfer_overdue_list(dd_task_list_node active, dd_task_list_node overdue);

char* format_complete_list(dd_task_list_node task_list);

char* format_list(dd_task_list_node task_list);

#endif /* DDTASKHEADER_H_ */
