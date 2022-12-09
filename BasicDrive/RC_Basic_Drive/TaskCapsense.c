/*
 * TaskCapsense.c
 *
 *  Created on: Nov 14, 2022
 *      Author: eduar
 */

#include "TaskCapsense.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "SensorData.h"

/* FreeRTOS header file. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/*******************************************************************************
* Global constants
*******************************************************************************/

#define CAPSENSE_INTERRUPT_PRIORITY    (7u)
#define EZI2C_INTERRUPT_PRIORITY    (6u)    /* EZI2C interrupt priority must be
                                             * higher than CapSense interrupt
                                             */
#define CAPSENSE_SCAN_INTERVAL_MS    (10u)   /* in milliseconds*/

QueueHandle_t queue_capsense_buttons_handle;
QueueHandle_t queue_capsense_slider_handle;
QueueHandle_t capsense_command_q;
/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static uint32_t capsense_init(void);
static void tuner_init(void);
static void process_touch(void);
static void capsense_isr(void);
static void capsense_end_of_scan_callback(cy_stc_active_scan_sns_t* active_scan_sns_ptr);
static void capsense_timer_callback(TimerHandle_t xTimer);
void handle_error(void);

TimerHandle_t scan_timer_handle;
cy_stc_scb_ezi2c_context_t ezi2c_context;
cyhal_ezi2c_t sEzI2C;
cyhal_ezi2c_slave_cfg_t sEzI2C_sub_cfg;
cyhal_ezi2c_cfg_t sEzI2C_cfg;

/* SysPm callback params */
cy_stc_syspm_callback_params_t callback_params =
{
    .base       = CYBSP_CSD_HW,
    .context    = &cy_capsense_context
};

cy_stc_syspm_callback_t capsense_deep_sleep_cb =
{
    Cy_CapSense_DeepSleepCallback,
    CY_SYSPM_DEEPSLEEP,
    (CY_SYSPM_SKIP_CHECK_FAIL | CY_SYSPM_SKIP_BEFORE_TRANSITION | CY_SYSPM_SKIP_AFTER_TRANSITION),
    &callback_params,
    NULL,
    NULL
};




void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

void TaskCapsense(void* param)
{
    BaseType_t rtos_api_result;
    cy_status status;
    capsense_command_t capsense_cmd;

    /* Remove warning for unused parameter */
    (void)param;

    /* Initialize timer for periodic CapSense scan */
    scan_timer_handle = xTimerCreate ("Scan Timer", CAPSENSE_SCAN_INTERVAL_MS,
                                      pdTRUE, NULL, capsense_timer_callback);

    /* Setup communication between Tuner GUI and PSoC 6 MCU */
    tuner_init();

    /* Initialize CapSense block */
    status = capsense_init();
    if(CY_RET_SUCCESS != status)
    {
        CY_ASSERT(0u);
    }

    /* Start the timer */
    xTimerStart(scan_timer_handle, 0u);

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a CapSense command has been received over queue */
        rtos_api_result = xQueueReceive(capsense_command_q, &capsense_cmd,
                                        portMAX_DELAY);

        /* Command has been received from capsense_cmd */
        if(rtos_api_result == pdTRUE)
        {
            /* Check if CapSense is busy with a previous scan */
            if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
            {
                switch(capsense_cmd)
                {
                    case CAPSENSE_SCAN:
                    {
                        /* Start scan */
                        Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
                        break;
                    }
                    case CAPSENSE_PROCESS:
                    {
                        /* Process all widgets */
                        Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
                        process_touch();

                        /* Establishes synchronized operation between the CapSense
                         * middleware and the CapSense Tuner tool.
                         */
                        Cy_CapSense_RunTuner(&cy_capsense_context);
                        break;
                    }
                    /* Invalid command */
                    default:
                    {
                        break;
                    }
                }
            }
        }
        /* Task has timed out and received no data during an interval of
         * portMAXDELAY ticks.
         */
        else
        {
            /* Handle timeout here */
        }
    }
}

static void process_touch(void)
{
    /* Variables used to store touch information */
    uint32_t button0_status = 0;
    uint32_t button1_status = 0;
    uint16_t slider_pos = 0;
    uint8_t slider_touched = 0;
    cy_stc_capsense_touch_t *slider_touch;

    /* Variables used to store previous touch information */
    static uint32_t button0_status_prev = 0;
	static uint32_t button1_status_prev = 0;
	static uint16_t slider_pos_prev = 0;

	int button0_val_prev = -1;
	int button1_val_prev = -1;
	int slider_val_prev = -1;

    sensor_data_msg_t capsense_data_msg;
    capsense_data_msg.id = 102;
    capsense_data_msg.data = 0;

    static int button_count = 0;

    static int slider_val = 0;

	/* Process touch input */

	/* Get button 0 status */
	button0_status = Cy_CapSense_IsSensorActive(
		CY_CAPSENSE_BUTTON0_WDGT_ID,
		CY_CAPSENSE_BUTTON0_SNS0_ID,
		&cy_capsense_context);

	/* Get button 1 status */
	button1_status = Cy_CapSense_IsSensorActive(
		CY_CAPSENSE_BUTTON1_WDGT_ID,
		CY_CAPSENSE_BUTTON1_SNS0_ID,
		&cy_capsense_context);

	/* Get slider status */
	slider_touch = Cy_CapSense_GetTouchInfo(
		CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,
		&cy_capsense_context);
	slider_pos = slider_touch->ptrPosition->x;
	slider_touched = slider_touch->numPosition;


	if ((0u != button0_status) &&
		(0u == button0_status_prev))
	{
        button_count = -1;
    }


	if ((0u != button1_status) &&
		(0u == button1_status_prev))
	{
		button_count = 1;
	}

    if (button0_status == 0u && button1_status == 0)
    {
        button_count = 0;
    }

	/* Detect the new touch on slider */
	

    if ((0u != slider_touched) && (slider_pos_prev != slider_pos ))
	{
    	slider_val = (slider_pos * 100) / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution;
	}
    capsense_data_msg.data = slider_val;
    xQueueSend(queue_capsense_slider_handle, &capsense_data_msg, 0UL);
    xQueueSend(queue_capsense_buttons_handle, &button_count, 0UL);
	// printf("Task Capsense: button_count: %d, slider_val: %d\r\n", button_count, capsense_data_msg.data);


	/* Update previous touch status */
	button0_status_prev = button0_status;
	button1_status_prev = button1_status;
	slider_pos_prev = slider_pos;
}

static uint32_t capsense_init(void)
{
    uint32_t status = CYRET_SUCCESS;

    /* CapSense interrupt configuration parameters */
    static const cy_stc_sysint_t capSense_intr_config =
    {
        .intrSrc = csd_interrupt_IRQn,
        .intrPriority = CAPSENSE_INTERRUPT_PRIORITY,
    };

    /*Initialize CapSense Data structures */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    cyhal_system_set_isr(csd_interrupt_IRQn, csd_interrupt_IRQn, CAPSENSE_INTERRUPT_PRIORITY, &capsense_isr);
    NVIC_ClearPendingIRQ(capSense_intr_config.intrSrc);
    NVIC_EnableIRQ(capSense_intr_config.intrSrc);

    /* Initialize the CapSense deep sleep callback functions. */
    Cy_CapSense_Enable(&cy_capsense_context);
    Cy_SysPm_RegisterCallback(&capsense_deep_sleep_cb);
    /* Register end of scan callback */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E,
                                              capsense_end_of_scan_callback, &cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }
    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    return status;
}

static void capsense_end_of_scan_callback(cy_stc_active_scan_sns_t* active_scan_sns_ptr)
{
    BaseType_t xYieldRequired;

    (void)active_scan_sns_ptr;

    /* Send command to process CapSense data */
    capsense_command_t commmand = CAPSENSE_PROCESS;
    xYieldRequired = xQueueSendToBackFromISR(capsense_command_q, &commmand, 0u);
    portYIELD_FROM_ISR(xYieldRequired);
}

static void capsense_timer_callback(TimerHandle_t xTimer)
{
    Cy_CapSense_Wakeup(&cy_capsense_context);
    capsense_command_t command = CAPSENSE_SCAN;
    BaseType_t xYieldRequired;

    (void)xTimer;

    /* Send command to start CapSense scan */
    xYieldRequired = xQueueSendToBackFromISR(capsense_command_q, &command, 0u);
    portYIELD_FROM_ISR(xYieldRequired);
}

static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}


static void tuner_init(void)
{
    cy_rslt_t result;
    /* Configure Capsense Tuner as EzI2C Slave */
    sEzI2C_sub_cfg.buf = (uint8 *)&cy_capsense_tuner;
    sEzI2C_sub_cfg.buf_rw_boundary = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.buf_size = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.slave_address = 8U;

    sEzI2C_cfg.data_rate = CYHAL_EZI2C_DATA_RATE_400KHZ;
    sEzI2C_cfg.enable_wake_from_sleep = true;
    sEzI2C_cfg.slave1_cfg = sEzI2C_sub_cfg;
    sEzI2C_cfg.sub_address_size = CYHAL_EZI2C_SUB_ADDR16_BITS;
    sEzI2C_cfg.two_addresses = false;
    result = cyhal_ezi2c_init( &sEzI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL, &sEzI2C_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

}
