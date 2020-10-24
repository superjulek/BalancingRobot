/*
 * scheduler.h
 *
 *  Created on: Oct 3, 2020
 *      Author: juliusz
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include "main.h"
#include "event.h"

/*##### TO BE DEFINED #####*/
/**
 * QUEUE_SIZE - max number of events in que
 */
/*#########################*/

typedef struct scheduler_t scheduler_t;

struct scheduler_t {

	/* Take next event from queue and handle it */
	void (*handle_next_event) (scheduler_t *this);

	/* Add new event to queue */
	void (*add_to_queue) (scheduler_t *this, event_t event);


};

scheduler_t *scheduler_create(void);

#endif /* INC_SCHEDULER_H_ */
