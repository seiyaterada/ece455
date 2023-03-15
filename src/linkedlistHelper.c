/*
 * linkedlistHelper.c
 *
 *  Created on: Mar 14, 2023
 *      Author: kaidenrivett
 */

#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

#include "ddTaskHeader.h"

//List_Head *list_head() {
//  List_Head *leader =
//      (List_Head *)pvPortMalloc(sizeof(List_Head));
//  leader->head = NULL;
//  leader->cursor = NULL;
//  leader->cursor_prev = NULL;
////  leader->length = 0;
////  leader->add_count = 0;
//  return leader;
//}

Task_Node *new_node(dd_task *task) {
	Task_Node *node = (Task_Node *)pvPortMalloc(sizeof(Task_Node));
	node->next = NULL;
	node->prev = NULL;
	node->task = task;

	return node;
}

void add_node(List_Head *head, Task_Node *node) {
	if()

	while(current->cursor)
}
