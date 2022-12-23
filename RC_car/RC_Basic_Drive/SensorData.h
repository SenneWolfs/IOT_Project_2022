#ifndef SENSORDATA_H_
#define SENSORDATA_H_

// id = 100 -> Battery, id = 200 -> Motorspeed, id = 201 -> SteeringAngle

typedef struct sensor_data_msg
{
    int id;
    int data;
} sensor_data_msg_t;

#endif /* SENSORDATA_H_ */

/* [] END OF FILE */
