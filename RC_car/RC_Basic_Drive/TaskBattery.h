
/*
 * TaskCapsense.h
 *
 *  Created on: Dec 23, 2022
 *      Author: Eduardo and Senne
 */

#ifndef TASKBATTERY_H_
#define TASKBATTERY_H_

#include "SensorData.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Macro for ADC Channel configuration*/
#define SINGLE_CHANNEL 1

/*
 * Macro to choose between single channel and multiple channel configuration of
 * ADC. Single channel configuration uses channel 0 in single ended mode.
 * Multiple channel configuration uses two channels, channel 0 in single ended
 * mode and channel 1 in differential mode.
 *
 * The default configuration is set to use single channel.
 * To use multiple channel configuration set ADC_EXAMPLE_MODE macro to MULTI_CHANNEL.
 *
 */
#define ADC_EXAMPLE_MODE SINGLE_CHANNEL

#define VPLUS_CHANNEL_0  (P10_2)


/* Conversion factor */
#define MICRO_TO_MILLI_CONV_RATIO        (1000u)

/* Acquistion time in nanosecond */
#define ACQUISITION_TIME_NS              (1000u)

/* ADC Scan delay in millisecond */
#define ADC_SCAN_DELAY_MS                (1000u)
/* ADC maximum samples*/
#define ADC_MAX_SAMPLES_N                (100)


/* Single channel initialization function*/
void adc_single_channel_init(void);

/* Function to read input voltage from channel 0 */
void adc_single_channel_process(void);

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/******************************************************************************
* Global variables
******************************************************************************/
extern QueueHandle_t queue_battery_handle;
extern sensor_data_msg_t sensor_battery;

/* ADC Object */
extern cyhal_adc_t adc_obj;
/* ADC Channel 0 Object */
extern cyhal_adc_channel_t adc_chan_0_obj;


// Default ADC configuration
extern const cyhal_adc_config_t adc_config;

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void TaskBattery(void *arg);
int map(int x, int inMin, int inMax, int outMin, int outMax);

/* Battery Client task handle. */
extern TaskHandle_t TaskBatteryHandle;

#endif /* TASKBATTERY_H_ */
