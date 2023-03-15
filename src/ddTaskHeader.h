/*
 * ddTaskHeader.h
 *
 *  Created on: Mar 14, 2023
 *      Author: kaidenrivett
 */

#ifndef DDTASKHEADER_H_
#define DDTASKHEADER_H_

#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

enum task_type {PERIODIC,APERIODIC};
enum message_type {CREATE_TASK, DELETE_TASK, GET_ACTIVE, GET_COMPLETE, GET_OVERDUE};

typedef struct dd_task {
	TaskHandle_t t_handle;
	enum task_type type;
	uint32_t task_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
} dd_task;

typedef struct message {
	enum message_type type;
	struct dd_task task;
} message;

typedef struct Task_Node {
  dd_task *task;
  struct Task_Node *next;
  struct Task_Node *prev;
} Task_Node;

typedef struct List_Head {
  Task_Node *head;
  Task_Node *cursor;
  Task_Node *cursor_prev;
//  unsigned int length;
//  unsigned int add_count;
} List_Head;

List_Head list_head();




#endif /* DDTASKHEADER_H_ */
