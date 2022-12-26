/*
 * TaskBattery.c
 *
 *  Created on: 23 Dec 2022
 *      Author: Eduardo and Senne
 */

#include "TaskBattery.h"
#include "cycfg_capsense.h"
#include "timers.h"

/* FreeRTOS header file. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdlib.h>

QueueHandle_t queue_battery_handle;

/* ADC Object */
cyhal_adc_t adc_obj;
/* ADC Channel 0 Object */
cyhal_adc_channel_t adc_chan_0_obj;


// Default ADC configuration
const cyhal_adc_config_t adc_config = {
        .continuous_scanning=false, // Continuous Scanning is disabled
        .average_count=1,           // Average count disabled
        .vref=CYHAL_ADC_REF_VDDA,   // VREF for Single ended channel set to VDDA
        .vneg=CYHAL_ADC_VNEG_VSSA,  // VNEG for Single ended channel set to VSSA
        .resolution = 12u,          // 12-bit resolution
        .ext_vref = NC,             // No connection
        .bypass_pin = NC };       // No connection

/*******************************************************************************
*       Enumerated Types
*******************************************************************************/
/* ADC Channel constants*/
enum ADC_CHANNELS
{
  CHANNEL_0 = 0,
  CHANNEL_1,
  NUM_CHANNELS
} adc_channel;

void TaskBattery(void *arg)
{
	/* Variable to capture return value of functions */
	cy_rslt_t result;

	/* Initialize Channel 0 */
	adc_single_channel_init();

	/* Update ADC configuration */
	result = cyhal_adc_configure(&adc_obj, &adc_config);
	if(result != CY_RSLT_SUCCESS)
	{
		printf("ADC configuration update failed. Error: %ld\n", (long unsigned int)result);
		CY_ASSERT(0);
	}

	for (;;)
	{

		/* Sample input voltage at channel 0 */
		adc_single_channel_process();

		/* 200ms delay between scans */
		// cyhal_system_delay_ms(ADC_SCAN_DELAY_MS);
	}
}

/*******************************************************************************
 * Function Name: adc_single_channel_init
 *******************************************************************************
 *
 * Summary:
 *  ADC single channel initialization function. This function initializes and
 *  configures channel 0 of ADC.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void adc_single_channel_init(void)
{
    /* Variable to capture return value of functions */
    cy_rslt_t result;

    /* Initialize ADC. The ADC block which can connect to the channel 0 input pin is selected */
    result = cyhal_adc_init(&adc_obj, VPLUS_CHANNEL_0, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /* ADC channel configuration */
    const cyhal_adc_channel_config_t channel_config = {
            .enable_averaging = false,  // Disable averaging for channel
            .min_acquisition_ns = ACQUISITION_TIME_NS, // Minimum acquisition time set to 1us
            .enabled = true };          // Sample this channel when ADC performs a scan

    /* Initialize a channel 0 and configure it to scan the channel 0 input pin in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adc_obj, VPLUS_CHANNEL_0,
                                          CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC single ended channel initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    printf("ADC is configured in single channel configuration\r\n\n");
    printf("Provide input voltage at the channel 0 input pin. \r\n\n");
}

/*******************************************************************************
 * Function Name: adc_single_channel_process
 *******************************************************************************
 *
 * Summary:
 *  ADC single channel process function. This function reads the input voltage
 *  and prints the input voltage on UART.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void adc_single_channel_process(void)
{
    /* Variable to store ADC conversion result from channel 0 */
    int32_t adc_result_0 = 0;

    /* Variables to compute the battery level percentage. (3S LiPo Battery) */
    /*
     * Absolute minimum = 3*3.0 V = 9.00 V
     * Absolute maximum = 3*4.2 V = 12.6 V
     *
     * Recommended minimum = 3*3.2 V = 9.60 V
     * Recommended maximum = 3*4.1 V = 12.3 V
     *
     * Nominal = 3*3.8 V = 11.4 V (= 50%) ~ 2732 mV
     *
     * Linearization:
     * 		minimum = Nominal - 3*0.4 V = 10.2 V ~ minVoltage = 2391 mV
     * 		maximum = Nominal + 3*0.4 V = 12.6 V ~ maxVoltage = 2769 mV
     *
     *
     */
    static int32_t minVoltage = 2391;
    static int32_t maxVoltage = 2769;
    static int old_sensor_battery_data = 0;
    sensor_data_msg_t sensor_battery;
    sensor_battery.id = 100;

    /* Read input voltage, take n samples and convert it to millivolts and print input voltage */
    for (int i = 0; i < ADC_MAX_SAMPLES_N; ++i)
    {
    	adc_result_0 += (cyhal_adc_read_uv(&adc_chan_0_obj) / MICRO_TO_MILLI_CONV_RATIO);
    }
    adc_result_0 /= ADC_MAX_SAMPLES_N; 

    //printf("Channel 0 input: %4ldmV\r\n", (long int)adc_result_0);
    sensor_battery.data = map(adc_result_0, minVoltage, maxVoltage, 0, 100);
    if (old_sensor_battery_data != sensor_battery.data)
    {
    	printf("Battery level: %d%%\r\n", sensor_battery.data);
		printf("Battery ADC: %d\r\n", adc_result_0);
    }
    old_sensor_battery_data = sensor_battery.data;
    xQueueSend(queue_battery_handle, &sensor_battery, 0UL);
    // printf("Task Battery: Battery data sent.\r\n");
}

int map(int x, int inMin, int inMax, int outMin, int outMax)
{
	if (x == 0)
	{
		return 0;
	}
	return (x - inMin)*(outMax - outMin)/(inMax - inMin) + outMin;
}



