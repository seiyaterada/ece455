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

void insertAtBegin(struct Task_Node **start_ref, struct dd_task *task) 
{ 
    struct Task_Node *ptr1 = (struct Task_Node*)malloc(sizeof(struct Task_Node)); 
    ptr1->task = task; 
    ptr1->next = *start_ref; 
    *start_ref = ptr1; 
} 


void bubbleSort(struct Task_Node *start) 
{ 
    int swapped, i; 
    struct Task_Node *ptr1; 
    struct Task_Node *lptr = NULL; 
  
    /* Checking for empty list */
    if (start == NULL) 
        return; 
  
    do
    { 
        swapped = 0; 
        ptr1 = start; 
  
        while (ptr1->next != lptr) 
        { 
            if (ptr1->data > ptr1->next->data) 
            { 
                swap(ptr1, ptr1->next); 
                swapped = 1; 
            } 
            ptr1 = ptr1->next; 
        } 
        lptr = ptr1; 
    } 
    while (swapped); 
} 

void swap(struct Task_Node *a, struct Task_Node *b) 
{ 
    struct dd_task temp = a->task; 
    a->task = b->task; 
    b->task = temp; 
} 