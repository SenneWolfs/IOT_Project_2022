/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"

#include "cy_retarget_io.h"

#include "FreeRTOS.h"
#include <task.h>
#include <queue.h>

#include "SensorData.h"
#include "TaskCapsense.h"
#include "TaskActuation.h"
#include "TaskBattery.h"


#define TASKCAPSENSE_STACK_SIZE        (1024)
#define TASKCAPSENSE_PRIORITY          (1)

#define TASKACTUATION_STACK_SIZE        (1024)
#define TASKACTUATION_PRIORITY          (1)

#define TASKBATTERY_STACK_SIZE        (1024)
#define TASKBATTERY_PRIORITY          (1)

#define SINGLE_ELEMENT_QUEUE (1u)

TaskHandle_t TaskCapsenseHandle;

TaskHandle_t TaskActuationHandle;

TaskHandle_t TaskBatteryHandle;

uint8_t uart_read_value;

// extern QueueHandle_t queue_capsense_handle;
// extern QueueHandle_t capsense_command_q;

int main(void)
{
	cy_rslt_t cy_result;

		/* Initialize the device and board peripherals */
		cy_result = cybsp_init();

		/* Board init failed. Stop program execution */
		if (cy_result != CY_RSLT_SUCCESS)
		{
			CY_ASSERT(0);
		}

		/* Enable global interrupts */
		__enable_irq();

		/* Initialize retarget-io to use the debug UART port */
		cy_result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
		printf("Application startup... \r\n");

		/* Initialize the User LED */
		cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

		printf("Peripherals initialized.");

		/* Init QSPI and enable XIP to get the Wi-Fi firmware from the QSPI NOR flash */
		#if defined(CY_DEVICE_PSOC6A512K)
		const uint32_t bus_frequency = 50000000lu;
		cy_serial_flash_qspi_init(smifMemConfigs[0], CYBSP_QSPI_D0, CYBSP_QSPI_D1,
									  CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC,
									  CYBSP_QSPI_SCK, CYBSP_QSPI_SS, bus_frequency);

		    cy_serial_flash_qspi_enable_xip(true);
		#endif




		printf("\x1b[2J\x1b[;H");
		printf("============================================================\n");
		printf("IoT Project 2022: RC CAR ALL PERIPHERALS: FREERTOS\n");
		printf("============================================================\n\n");

		capsense_command_q = xQueueCreate(SINGLE_ELEMENT_QUEUE,
		                                      sizeof(capsense_command_t));

		xTaskCreate(TaskCapsense, "Task Capsense", TASKCAPSENSE_STACK_SIZE, NULL, TASKCAPSENSE_PRIORITY, &TaskCapsenseHandle);
		xTaskCreate(TaskActuation, "Task Actuation", TASKCAPSENSE_STACK_SIZE, NULL, TASKCAPSENSE_PRIORITY, &TaskActuationHandle);
		xTaskCreate(TaskBattery, "Task Battery", TASKBATTERY_STACK_SIZE, NULL, TASKBATTERY_PRIORITY, &TaskBatteryHandle);

		queue_capsense_buttons_handle = xQueueCreate(1, sizeof(int));
		queue_capsense_slider_handle = xQueueCreate(1, sizeof(sensor_data_msg_t));
		queue_battery_handle = xQueueCreate(1, sizeof(sensor_data_msg_t));

		vTaskStartScheduler();

		CY_ASSERT(0);
}

/* [] END OF FILE */
