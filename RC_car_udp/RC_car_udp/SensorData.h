#ifndef SENSORDATA_H_
#define SENSORDATA_H_

/*
 * SensorData.h
 *
 *  Created on: Nov 16, 2022
 *  Edited on: Dec 23, 2022
 *      Author: Eduardo Bemelmans
 */

// id = 100 -> Battery, id = 200 -> Motorspeed, id = 201 -> SteeringAngle

typedef struct sensor_data_msg
{
    int id;
    int data;
} sensor_data_msg_t;

#endif /* SENSORDATA_H_ */

/* [] END OF FILE */
