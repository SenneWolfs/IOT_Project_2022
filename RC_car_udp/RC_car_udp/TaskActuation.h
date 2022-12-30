/*
 * TaskActuation.h
 *
 *  Created on: Dec 09, 2022
 *      Author: Eduardo Bemelmans
 */

#ifndef TASKACTUATION_H_
#define TASKACTUATION_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ActuationData.h"


/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void TaskActuation(void *arg);

/* Actuation Client task handle. */
extern TaskHandle_t TaskActuationHandle;
extern QueueHandle_t queue_actuation_handle;
extern TimerHandle_t timer_handle_actuation;
extern actuation_data_msg_t actuation_data;

#endif /* TASKACTUATION_H_ */
