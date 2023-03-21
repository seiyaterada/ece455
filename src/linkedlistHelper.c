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

void insert(struct Task_Node** headRef, dd_task *task) {
    // Create a new node with the given data
    struct Task_Node* newNode = (struct Task_Node*)malloc(sizeof(struct Task_Node));
    newNode->task = task;
    newNode->next = NULL;

    // If the list is empty or the new data is less than the head node's data,
    // insert the new node at the beginning of the list
    if (*headRef == NULL || task.absolute_deadline < (*headRef)->task.absolute_deadline) {
        newNode->next = *headRef;
        *headRef = newNode;
    } else {
        // Traverse the list to find the appropriate position to insert the new node
        struct Task_Node* curr = *headRef;
        while (curr->next != NULL && task.absolute_deadline > curr->next->task.absolute_deadline) {
            curr = curr->next;
        }
        newNode->next = curr->next;
        curr->next = newNode;
    }
}

// Remove the first node from the list and return its data
dd_task* pop(struct Task_Node** headRef) {
	if (*headRef == NULL) {
		return NULL;
	}
	struct Task_Node* temp = *headRef;
	*headRef = (*headRef)->next;
	dd_task* task = temp->task;
	free(temp);
	return task;
}

// How to use
// create new head
// struct Task_Node* head = NULL
// insert(&head, task)
