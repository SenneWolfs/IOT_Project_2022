


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
#include "TaskCapsense.h"

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
        xQueueReceive(queue_capsense_slider_handle, (void*)sensor_data, portMAX_DELAY);
        xQueueReceive(queue_capsense_buttons_handle, &capsense_button_count, portMAX_DELAY);

        if (capsense_button_count != capsense_button_count_prev)
        {
            switch (capsense_button_count)
            {
                case -1:
                    pwmServoDutyCycle = PWM_DUTY_CYCLE - deltaDC*100.0f;
                break;
                case 0:
                    pwmServoDutyCycle = PWM_DUTY_CYCLE;
                break;
                case 1:
                    pwmServoDutyCycle = PWM_DUTY_CYCLE + deltaDC*100.0f;
                break;
                default:
                break;
            }
            cyhal_pwm_set_duty_cycle(&pwm_servo_motor, pwmServoDutyCycle, PWM_FREQUENCY);
            printf("Task Actuation: Servo duty cycle = %f\r\n", pwmServoDutyCycle);
        }
        capsense_button_count_prev = capsense_button_count;

        if (sliderVal != sensor_data->data)
        {
            printf("Task Actuation: Received data: id = %d, data = %d\r\n", sensor_data->id, sensor_data->data);
            pwmBLDCDutyCycle = PWM_DUTY_CYCLE + deltaDC*(float)sensor_data->data;
            cyhal_pwm_set_duty_cycle(&pwm_bldc_motor, pwmBLDCDutyCycle, PWM_FREQUENCY);
            printf("Task Actuation: BLDC duty cycle = %f\r\n", pwmBLDCDutyCycle);
        }
        sliderVal = sensor_data->data;


    }
}