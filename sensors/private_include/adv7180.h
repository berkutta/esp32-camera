/*
 *
 * SC031GS DVP driver.
 *
 */
#ifndef __ADV7180_H__
#define __ADV7180_H__

#include "sensor.h"

/**
 * @brief Detect sensor pid
 *
 * @param slv_addr SCCB address
 * @param id Detection result
 * @return
 *     0:       Can't detect this sensor
 *     Nonzero: This sensor has been detected
 */
int adv7180_detect(int slv_addr, sensor_id_t *id);

/**
 * @brief initialize sensor function pointers
 *
 * @param sensor pointer of sensor
 * @return
 *      Always 0
 */
int adv7180_init(sensor_t *sensor);

#endif // __SC031GS_H__
