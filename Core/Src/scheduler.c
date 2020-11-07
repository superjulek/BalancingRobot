/*
 * scheduler.c
 *
 *  Created on: Oct 3, 2020
 *      Author: juliusz
 */

#include "scheduler.h"
#include "config.h"
#include "general.h"
#include "stdlib.h"
#include "bluetooth_communicator.h"

typedef struct private_scheduler_t private_scheduler_t;

struct private_scheduler_t
{

	scheduler_t public;

	event_t queue[QUEUE_SIZE];

	/* Index of first event to handle in queue */
	uint8_t first_event;

	/* Index of last event to handle in queue */
	uint8_t last_event;
};

static void handle_next_event(scheduler_t *public)
{
	private_scheduler_t *this = (private_scheduler_t *)public;

	if (this->first_event == this->last_event)
		return;
	this->queue[this->first_event].action();

	this->first_event++;

	if (this->first_event == QUEUE_SIZE)
		this->first_event = 0;
}

static void add_to_queue(scheduler_t *public, event_t event)
{
	private_scheduler_t *this = (private_scheduler_t *)public;

	this->queue[this->last_event] = event;
	this->last_event++;
	if (this->last_event == QUEUE_SIZE)
		this->last_event = 0;
	if (this->last_event == this->first_event)
	{
		bt_send_message(&huart1, "QUE OVERFLOW");
	}
}

scheduler_t *scheduler_create(void)
{
	private_scheduler_t *this = malloc(sizeof(private_scheduler_t));
	*this = (private_scheduler_t){
		.public = {
			.handle_next_event = handle_next_event,
			.add_to_queue = add_to_queue,
		},
		.first_event = 0,
		.last_event = 0,
	};
	return &(this->public);
}
