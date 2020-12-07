/*
 * MPU.h
 *
 *  Created on: Oct 25, 2020
 *      Author: juliusz
 */

#ifndef INC_MPU_H_
#define INC_MPU_H_

#include "general.h"

#include "stm32f4xx_hal.h"

/*##### TO BE DEFINED #####*/
/**
 * I2C_TIMEOUT - timeout for I2C connections
 * 
 * CALIBRATION_ROUND - number of gyro calibration rounds
 * 
 * ACC_PART - accelerometer part in angle calculation
 * 
 * MOUNT_ERROR - mounting error set at startup
 *
 * GYRO_Z_AXIS_ERROR - angle of robot Z axis overlap on gyro Y axis in rad
 */
/*#########################*/

typedef struct MPU_t MPU_t;
typedef struct MPU_reading_t MPU_reading_t;

struct MPU_t
{

    /**
     * Reset MPU
     */
    void (*reset)(MPU_t *public);

    /**
     * Get acc angle
     */
    float (*get_acc_angle)(MPU_t *public);

    /**
     * Get angle fro mcomplementary filter
     */
    float (*get_comp_angle)(MPU_t *public);

    /**
     * Calibrate gyro
     */
    HAL_StatusTypeDef (*calibrate_gyro)(MPU_t *pbulic);

    /**
     * Reset last angle
     */
    void (*reset_last_angle)(MPU_t *public);

    /**
     * Set last angle
     */
    void (*set_last_angle)(MPU_t *public, float new_last_angle);

    /**
     * Reset mount error
     */
    void (*reset_mount_error)(MPU_t *public);

    /**
     * Set value of mount error
     */
    void (*set_mount_error)(MPU_t *public, float mount_error);
};

struct MPU_reading_t
{
    float x;
    float y;
    float z;
    float rx;
    float ry;
    float rz;
};

MPU_t *MPU_create(I2C_HandleTypeDef *I2C, uint16_t frequency, pin_t power_pin, uint8_t address);

#endif /* INC_MPU_H_ */
