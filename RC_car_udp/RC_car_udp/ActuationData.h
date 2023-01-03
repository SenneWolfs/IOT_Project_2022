#ifndef ACTUATIONDATA_H_
#define ACTUATIONDATA_H_

/*
 * ActuationData.h
 *
 *  Created on: Dec 30, 2022
 *      Author: Senne Wolfs
 */

typedef struct actuation_data_msg
{
	float pwmPLD;
	float pwmServo;
} actuation_data_msg_t;

#endif /* ACTUATIONDATA_H_ */

/* [] END OF FILE */
