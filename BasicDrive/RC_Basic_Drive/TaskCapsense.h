/*
 * TaskCapsense.h
 *
 *  Created on: Nov 14, 2022
 *      Author: eduar
 */

#ifndef TASKCAPSENSE_H_
#define TASKCAPSENSE_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


/*******************************************************************************
 * Enumeration
 ******************************************************************************/
typedef enum
{
    CAPSENSE_SCAN,
    CAPSENSE_PROCESS
} capsense_command_t;

/******************************************************************************
* Global variables
******************************************************************************/
extern QueueHandle_t queue_capsense_buttons_handle;
extern QueueHandle_t queue_capsense_slider_handle;
extern QueueHandle_t capsense_command_q;


/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void TaskCapsense(void *arg);

#endif /* TASKCAPSENSE_H_ */
