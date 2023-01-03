/*
 * inputs.c
 *
 *  Created on: 29 dec. 2022
 *      Author: jethr
 */

#include "inputs.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "cy_retarget_io.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"



void task_inp() {
	int val_bat = 83;
	int val_speed = 35;
	int val_serv = 94;

	for(;;) {
		xQueueSendToBack(queue_battery_handle, &val_bat, 0u);
		xQueueSendToBack(queue_speed_handle, &val_speed, 0u);
		xQueueSendToBack(queue_servo_handle, &val_serv, 0u);

		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}
