/*
 * TaskActuation.h
 *
 *  Created on: Dec 09, 2022
 *      Author: Eduardo Bemelmans
 */


#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* FreeRTOS header file. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "SensorData.h"

/* PWM Frequency = 2Hz */
#define PWM_FREQUENCY (50u)
/* PWM Duty-cycle = 50% */
#define PWM_DUTY_CYCLE (7.6f)


void TaskActuation(void *arg)
{
    /* PWM object */
    cyhal_pwm_t pwm_bldc_motor;
    cyhal_pwm_t pwm_servo_motor;
    /* API return code */
    cy_rslt_t result;

    /* Initialize the BLDC MOTOR PWM */
    result = cyhal_pwm_init(&pwm_bldc_motor, P9_0, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_init failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
    result = cyhal_pwm_set_duty_cycle(&pwm_bldc_motor, PWM_DUTY_CYCLE, PWM_FREQUENCY);

    /* Initialize the SERVO MOTOR PWM */
    result = cyhal_pwm_init(&pwm_servo_motor, P9_1, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_init failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
    result = cyhal_pwm_set_duty_cycle(&pwm_servo_motor, PWM_DUTY_CYCLE, PWM_FREQUENCY);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_set_duty_cycle failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }

    /* Start the BLDC MOTOR PWM */
    result = cyhal_pwm_start(&pwm_bldc_motor);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_start failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
    result = cyhal_pwm_set_duty_cycle(&pwm_bldc_motor, PWM_DUTY_CYCLE, PWM_FREQUENCY);

    /* Start the SERVO MOTOR PWM */
    result = cyhal_pwm_start(&pwm_servo_motor);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_start failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
    result = cyhal_pwm_set_duty_cycle(&pwm_servo_motor, PWM_DUTY_CYCLE, PWM_FREQUENCY);

    int capsense_button_count = 0;
    int capsense_button_count_prev = 0;

    sensor_data_msg_t* sensor_data = malloc(sizeof(sensor_data_msg_t));


    int sliderVal = 0;

    float deltaDC = 0.024f;
    float pwmBLDCDutyCycle = PWM_DUTY_CYCLE;
    float pwmServoDutyCycle = PWM_DUTY_CYCLE;

    for (;;)
    {
        // printf("Task Actuation: queue receive capsense data...\r\n");
        xQueueReceive(queue_controller_handle, (void*)sensor_data, portMAX_DELAY);

        switch (controller_data_msg.id)
        {
            case 0:
                
            break;
            default:

            break;
        }


    }
}
