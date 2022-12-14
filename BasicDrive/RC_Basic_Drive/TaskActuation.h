/*
 * TaskCapsense.h
 *
 *  Created on: Dec 09, 2022
 *      Author: eduar
 */

#ifndef TASKACTUATION_H_
#define TASKACTUATION_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void TaskActuation(void *arg);

/* UDP Client task handle. */
extern TaskHandle_t TaskActuationHandle;

#endif /* TASKACTUATION_H_ */