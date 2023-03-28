

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

#include "ddTaskHeader.h"

dd_task_node allocate_node() {
	// Allocate space for new task node
	dd_task_node newNode = (dd_task_node)pvPortMalloc(sizeof(dd_task));

	newNode->t_handle = NULL;
	newNode->t_function = NULL;
	newNode->type = UNDEFINED;
	newNode->absolute_deadline = 0;
	newNode->completion_time = 0;
	newNode->task_id = 0;
	newNode->task_name = "";
	newNode->next = NULL;
	newNode->prev = NULL;

	return newNode;
}

bool free_node(dd_task_node node) {
	// Free the space used for task node once we are done with it
	if(node == NULL) {
		printf("ERROR: No node to free");
		return false;
	}

	node->t_handle = NULL;
	node->t_function = NULL;
	node->type = UNDEFINED;
	node->absolute_deadline = 0;
	node->completion_time = 0;
	node->task_id = 0;
	node->task_name = "";
	node->next = NULL;
	node->prev = NULL;

	vPortFree((void*) node);

	return true;
}

void create_task_list(dd_task_list_node task_list) {
	// Initialize a task list
	if(task_list == NULL) {
		printf("ERROR: No list given");
		return;
	}

	task_list->list_length = 0;
	task_list->head = NULL;
	task_list->tail = NULL;
//	printf("Created task list\n");
}

//// Inserts task into completed list
//void insert_complete(dd_task_node task, dd_task_list_node task_list) {
//	if(task_list->list_length == 0) {
//		task_list->list_length = 1;
//		task_list->head = task;
//		task_list->tail = task;
//		return;
//	}
//
//	dd_task_node insertTask = task;
//
//	dd_task_node temp = task_list->head;
//
//	task_list->head = insertTask;
//	insertTask->next = temp;
//	insertTask->prev = temp->prev;
//	temp->prev = insertTask;
//	(task_list->list_length)++;
//	return;
//}


// Insert task into list
void insert(dd_task_node task, dd_task_list_node task_list) {
	// If list is empty
	if(task_list->list_length == 0) {
		task_list->list_length = 1;
		task_list->head = task;
		task_list->tail = task;
		vTaskPrioritySet(task->t_handle, DD_TASK_PRIORITY_HIGH);
		return;
	}

	// If list is not empty iterate through list and place task in right place
	dd_task_node itr = task_list->head;

	while(itr != NULL) {
		if(task->absolute_deadline < itr->absolute_deadline) { // Found location
			if(itr == task_list->head) { // New task has earliest deadline
				task_list->head = task;
			}

			task->next = itr;
			task->prev = itr->prev;
			itr->prev->next = task;
			itr->prev = task;

			(task_list->list_length)++;

			vTaskPrioritySet(task->t_handle, DD_TASK_PRIORITY_HIGH);
			return;
		} else {
			if(itr->next == NULL) { // Reached end of list
				task->next = NULL;
				task->prev = itr;
				itr->next = task;
				task_list->tail = task;

				(task_list->list_length)++;
				vTaskPrioritySet(task->t_handle, DD_TASK_PRIORITY_EXECUTION_BASE);
				vTaskPrioritySet(task_list->head->t_handle, DD_TASK_PRIORITY_HIGH);
				return;
			}

			vTaskPrioritySet(itr->t_handle, DD_TASK_PRIORITY_EXECUTION_BASE);
			itr = itr->next;
		}
	}
}

// Finds node in list
dd_task_node findNode(uint32_t taskId, dd_task_list_node task_list) {
	// Check if list is empty
	if(task_list->list_length == 0) {
		printf("ERROR: List empty");
		return;
	}

	dd_task_node itr = task_list->head;

	// Iterate through list to find task
	while(itr != NULL) {
		if(itr->task_id == taskId) {
			return itr;
		}
		itr = itr->next;
	}
	printf("ERROR: Could not find node\n");
	return;
}

// Remove task from list based on task ID
void removeNode(uint32_t taskId, dd_task_list_node task_list, bool clear) {
	// Check if list is empty
	if(task_list->list_length == 0) {
		printf("ERROR: List empty");
		return;
	}

	dd_task_node itr = task_list->head;

	// Grab highest prio
	uint32_t prio = uxTaskPriorityGet(itr->t_handle);

	while(itr != NULL) {
		if(itr->task_id == taskId) {
			if(task_list->list_length == 1) { // If list only has one item
				task_list->head = NULL;
				task_list->tail = NULL;
			} else if(task_list->head->task_id == taskId) { // If task is at head of list
				task_list->head = itr->next;
				vTaskPrioritySet(task_list->head->t_handle, DD_TASK_PRIORITY_HIGH);
				itr->next->prev = NULL;
			} else if(task_list->tail->task_id == taskId) { // If task is at tail of list
				task_list->head = itr->prev;
				itr->prev->next = NULL;
			} else{
				itr->prev->next = itr->next;
				itr->next->prev = itr->prev;
			}

			task_list->list_length--;

			itr->prev = NULL;
			itr->next = NULL;

			if(clear) {
				free_node(itr);
			}

			return;
		}

		// Decrement the prio of other tasks
		prio--;
		vTaskPrioritySet(itr->t_handle, prio);
		itr = itr->next;
	}
}

// Removes the head of the list
void remove_head(dd_task_list_node task_list) {
	// Check if list is empty
	if(task_list->list_length == 0) {
		printf("ERROR: List empty");
		return;
	}

	dd_task_node head = task_list->head;

	// If list has one item
	if(task_list->list_length == 1) {
		task_list->head = NULL;
		task_list->tail = NULL;
	} else {
		task_list->head = head->next;
		head->next->prev = NULL;
	}

	task_list->list_length--;

	head->prev = NULL;
	head->next = NULL;

	// Free the space
	free_node(head);
}

// Transfers any overdue nodes from active to overdue list
void transfer_overdue_list(dd_task_list_node active, dd_task_list_node overdue) {
	dd_task_node itr = active->head;
	uint32_t itrId = itr->task_id;

	// Get the current time
	TickType_t current_time = xTaskGetTickCount();

	// Iterate through list
	while(itr != NULL) {
		// Check if deadline is overdue
		if(itr->absolute_deadline < current_time) {
			// If it is remove the node and add to overdue lsit
			removeNode(itrId, active, false);

			if(overdue->list_length == 0) { // If head
				overdue->list_length = 1;
				overdue->head = itr;
				overdue->tail = itr;
			} else {
				dd_task_node temp = overdue->tail;
				overdue->tail = itr;
				temp->next = itr;
				itr->prev = temp;

				overdue->list_length++;
			}

			if(itr->type == PERIODIC) {
				vTaskSuspend(itr->t_handle);
				vTaskDelete(itr->t_handle);
			}
		} else {
			return;
		}

		itr = itr->next;
		itrId = itr->task_id;
	}
}

char* format_complete_list(dd_task_list_node task_list) {
	uint32_t size = task_list->list_length;
	uint32_t buffer = ((configMAX_TASK_NAME_LEN + 50) * (size + 1));

	char* output = (char*)pvPortMalloc(buffer);
	output[0] = '\0';

	if(size == 0) {
		char buffer[20] = ("List is empty.\n");
    	strcat( output, buffer );
    	return output;
	}

	dd_task_node itr = task_list->head; // start from head
	    while( itr != NULL )
	    {
	        char itr_buffer[70];
	        sprintf( itr_buffer, "Task Name = %s, Completion = %u \n", itr->task_name, (unsigned int) itr->completion_time ); // need typecast to unsigned int to avoid warning.
	        strcat( output, itr_buffer );

	        itr = itr->next;         // Go to the next element in the list
	    }
	    return output;
}

char* format_list(dd_task_list_node task_list) {
	uint32_t size = task_list->list_length;
	uint32_t buffer = ((configMAX_TASK_NAME_LEN + 50) * (size + 1));

	char* output = (char*)pvPortMalloc(buffer);
	output[0] = '\0';

	if(size == 0) {
		char buffer[20] = ("List is empty.\n");
    	strcat( output, buffer );
    	return output;
	}

	dd_task_node itr = task_list->head; // start from head
	    while( itr != NULL )
	    {
	        char itr_buffer[70];
	        sprintf( itr_buffer, "Task Name = %s, Deadline = %u \n", itr->task_name, (unsigned int) itr->absolute_deadline ); // need typecast to unsigned int to avoid warning.
	        strcat( output, itr_buffer );

	        itr = itr->next;         // Go to the next element in the list
	    }
	    return output;
}
// How to use
// create new head
// struct Task_Node* head = NULL
// insert(&head, task)
