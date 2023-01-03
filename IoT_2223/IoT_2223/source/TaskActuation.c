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

#include "ActuationData.h"
#include "TaskActuation.h"
#include "ControllerData.h"
#include "TaskController.h"

/* PWM Frequency = 2Hz */
#define PWM_FREQUENCY (50u)
/* PWM Duty-cycle = 50% */
#define PWM_DUTY_CYCLE (7.6f)

TimerHandle_t timer_handle_actuation;
actuation_data_msg_t actuation_data;

QueueHandle_t queue_speed_handle;
QueueHandle_t queue_servo_handle;

void timer_callback_actuation(TimerHandle_t xTimer)
{
    actuation_data_msg_t *pwms = (actuation_data_msg_t *)pvTimerGetTimerID(xTimer);
    int pwmP = actuation_data.pwmPLD*100;
    int pwmS = actuation_data.pwmServo*100;
	xQueueSendToBack(queue_speed_handle, &pwmP, 0UL);
    xQueueSendToBack(queue_servo_handle, &pwmS, 0UL);
}

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

    controller_data_msg_t* controller_data_msg;

    controller_data_msg = malloc(sizeof(controller_data_msg_t));

    controller_data_msg->id = 0;
    controller_data_msg->value = 0;


    float deltaDC = 0.024f;
    float pwmBLDCDutyCycle = PWM_DUTY_CYCLE;
    float pwmServoDutyCycle = PWM_DUTY_CYCLE;

    timer_handle_actuation = xTimerCreate("Timer Actuation", pdMS_TO_TICKS(10000UL), pdTRUE, &actuation_data, timer_callback_actuation);
	xTimerStart(timer_handle_actuation, 0);

    printf("Task Actuation: peripherals configured.\r\n");
    for (;;)
    {
        printf("Task Actuation: queue receive capsense data...\r\n");
        xQueueReceive(queue_controller_handle, (void*)controller_data_msg, portMAX_DELAY);
        printf("Task Actuation: Received controller data: id = %d, data = %d\r\n", 
            controller_data_msg->id, controller_data_msg->value);
        switch (controller_data_msg->id)
        {
            case 0: // empty
            break;
            case 202: // right_trigger        
                pwmBLDCDutyCycle = PWM_DUTY_CYCLE + deltaDC*controller_data_msg->value;
                cyhal_pwm_set_duty_cycle(&pwm_bldc_motor, pwmBLDCDutyCycle, PWM_FREQUENCY);
                printf("Task Actuation: BLDC duty cycle = %f\r\n", pwmBLDCDutyCycle);    
            break;
            case 201: // left_trigger
                pwmBLDCDutyCycle = PWM_DUTY_CYCLE;
                cyhal_pwm_set_duty_cycle(&pwm_bldc_motor, pwmBLDCDutyCycle, PWM_FREQUENCY);
                printf("Task Actuation: BLDC duty cycle = %f\r\n", pwmBLDCDutyCycle);
                printf("Task Actuation: RC CAR is halted.\r\n");
            break;
            case 101: // l_thumb_x
                pwmServoDutyCycle = PWM_DUTY_CYCLE + 2.0f*deltaDC*controller_data_msg->value;
                cyhal_pwm_set_duty_cycle(&pwm_servo_motor, pwmServoDutyCycle, PWM_FREQUENCY);
                printf("Task Actuation: Servo duty cycle = %f\r\n", pwmServoDutyCycle);
			break;
            default: // scam!
            break;
        }
        actuation_data.pwmPLD = pwmBLDCDutyCycle;
        actuation_data.pwmServo = pwmServoDutyCycle;

    }
}
