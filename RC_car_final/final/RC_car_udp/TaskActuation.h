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
extern QueueHandle_t queue_actuation_handle_pld;
extern QueueHandle_t queue_actuation_handle_servo;
extern TimerHandle_t timer_handle_actuation;
extern actuation_data_msg_t actuation_data;

extern QueueHandle_t queue_speed_handle;
extern QueueHandle_t queue_servo_handle;

#endif /* TASKACTUATION_H_ */
