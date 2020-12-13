/*
 * MPU.c
 *
 *  Created on: Oct 25, 2020
 *      Author: juliusz
 */

#include "MPU.h"
#include "MPU_defines.h"
#include "config.h"
#include "bluetooth_communicator.h"

#include "i2c.h"
#include <math.h>
#include <stdio.h>

/* Variables for charts */
extern float accelerometer_angle;
extern float gyroscope_angle;

typedef struct private_MPU_t private_MPU_t;

struct private_MPU_t
{

    MPU_t public;

    I2C_HandleTypeDef *I2C;

    uint16_t frequency;

    pin_t power_pin;

    uint8_t address;

    float gyro_output_rate;

    float gyro_scale;

    float acc_scale;

    int16_t rx_cal;
    int16_t ry_cal;
    int16_t rz_cal;

    float mount_error;

    float last_angle;
};

static void DeviceReset(private_MPU_t *this, uint8_t Reset)
{
    uint8_t tmp;
    HAL_I2C_Mem_Read(this->I2C, this->address, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
    tmp &= ~(1 << MPU6050_PWR1_DEVICE_RESET_BIT);
    tmp |= ((Reset & 0x1) << MPU6050_PWR1_DEVICE_RESET_BIT);
    HAL_I2C_Mem_Write(this->I2C, this->address, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

static void SetSleepEnabled(private_MPU_t *this, uint8_t Enable)
{
    uint8_t tmp;
    HAL_I2C_Mem_Read(this->I2C, this->address, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
    tmp &= ~(1 << MPU6050_PWR1_SLEEP_BIT);
    tmp |= ((Enable & 0x1) << MPU6050_PWR1_SLEEP_BIT);
    HAL_I2C_Mem_Write(this->I2C, this->address, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

static void SetClockSource(private_MPU_t *this, uint8_t Source)
{
    uint8_t tmp;
    HAL_I2C_Mem_Read(this->I2C, this->address, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
    tmp &= 0xF8;
    tmp |= (Source & 0x7);
    HAL_I2C_Mem_Write(this->I2C, this->address, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

static void SetDlpf(private_MPU_t *this, uint8_t Value)
{
    uint8_t tmp;
    HAL_I2C_Mem_Read(this->I2C, this->address, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
    tmp &= 0xF8;
    tmp |= (Value & 0x7);
    HAL_I2C_Mem_Write(this->I2C, this->address, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
    switch (Value)
    {
    case MPU6050_DLPF_BW_256:
        this->gyro_output_rate = 8000;
        break;
    default:
        this->gyro_output_rate = 1000;
    }
}

static void SetFullScaleGyroRange(private_MPU_t *this, uint8_t Range)
{
    uint8_t tmp;
    HAL_I2C_Mem_Read(this->I2C, this->address, MPU6050_RA_GYRO_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
    tmp &= 0xE7;
    tmp |= ((Range & 0x7) << 3);
    HAL_I2C_Mem_Write(this->I2C, this->address, MPU6050_RA_GYRO_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);

    switch (Range)
    {
    case MPU6050_GYRO_FS_250:
        this->gyro_scale = 0.007633;
        break;
    case MPU6050_GYRO_FS_500:
        this->gyro_scale = 0.015267;
        break;
    case MPU6050_GYRO_FS_1000:
        this->gyro_scale = 0.030487;
        break;
    case MPU6050_GYRO_FS_2000:
        this->gyro_scale = 0.060975;
        break;
    default:
        break;
    }
}

static void SetFullScaleAccelRange(private_MPU_t *this, uint8_t Range)
{
    uint8_t tmp;
    HAL_I2C_Mem_Read(this->I2C, this->address, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
    tmp &= 0xE7;
    tmp |= ((Range & 0x7) << 3);
    HAL_I2C_Mem_Write(this->I2C, this->address, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);

    switch (Range)
    {
    case MPU6050_ACCEL_FS_2:
        this->acc_scale = 0.000061;
        break;
    case MPU6050_ACCEL_FS_4:
        this->acc_scale = 0.000122;
        break;
    case MPU6050_ACCEL_FS_8:
        this->acc_scale = 0.000244;
        break;
    case MPU6050_ACCEL_FS_16:
        this->acc_scale = 0.0004882;
        break;
    default:
        break;
    }
}

static MPU_reading_t GetAllRAW(private_MPU_t *this)
{
    uint8_t tmp[14];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(this->I2C, this->address, MPU6050_RA_ACCEL_XOUT_H, 1, tmp, 14, I2C_TIMEOUT);

    int16_t x = (((int16_t)tmp[0]) << 8) | tmp[1];
    int16_t y = (((int16_t)tmp[2]) << 8) | tmp[3];
    int16_t z = (((int16_t)tmp[4]) << 8) | tmp[5];

    int16_t rx = ((((int16_t)tmp[8]) << 8) | tmp[9]) - this->rx_cal;
    int16_t ry = ((((int16_t)tmp[10]) << 8) | tmp[11]) - this->ry_cal;
    int16_t rz = ((((int16_t)tmp[12]) << 8) | tmp[13]) - this->rz_cal;

    MPU_reading_t result;
    result.x = (float)x;
    result.y = (float)y;
    result.z = (float)z;

    result.rx = rx;
    result.ry = ry;
    result.rz = rz;

    return result;
}

static MPU_reading_t GetAllScaled(private_MPU_t *this)
{
    MPU_reading_t result = GetAllRAW(this);

    result.x *= this->acc_scale;
    result.y *= this->acc_scale;
    result.z *= this->acc_scale;

    result.rx *= this->gyro_scale;
    result.ry *= this->gyro_scale;
    result.rz *= this->gyro_scale;

    return result;
}

static void reset(MPU_t *public)
{
    private_MPU_t *this = (private_MPU_t *)public;

    bt_send_message(&huart1, "RESET MPU");
    MX_I2C1_Init();
    write_pin(this->power_pin, 0);
    HAL_Delay(10);
    write_pin(this->power_pin, 1);
    HAL_Delay(10);
    DeviceReset(this, 1);
    SetSleepEnabled(this, 0);
    SetClockSource(this, MPU6050_CLOCK_INTERNAL);
    SetDlpf(this, MPU6050_DLPF_BW_20);
    SetFullScaleGyroRange(this, MPU6050_GYRO_FS_500);
    SetFullScaleAccelRange(this, MPU6050_ACCEL_FS_4);
    SetSleepEnabled(this, 0);
    this->rx_cal = 0;
    this->ry_cal = 0;
    this->rz_cal = 0;
    HAL_Delay(500);
}

static float get_acc_angle(MPU_t *public)
{
    private_MPU_t *this = (private_MPU_t *)public;
    MPU_reading_t scaled_data = GetAllScaled(this);
    if (scaled_data.z == 0)
        return 90;
    float acc_angle = atan(scaled_data.x / scaled_data.z) * 57.3 - this->mount_error;
    return acc_angle;
}

static float get_comp_angle(MPU_t *public)
{
    private_MPU_t *this = (private_MPU_t *)public;
    MPU_reading_t scaled_data = GetAllScaled(this);
    if (scaled_data.z == 0)
        return 90;
    float acc_angle = atan(scaled_data.x / scaled_data.z) * 57.3 - this->mount_error;
    float gyro_angle = this->last_angle - (scaled_data.ry - scaled_data.rz * sin(GYRO_Z_AXIS_ERROR)) * 1 / this->frequency;
    accelerometer_angle = acc_angle;
    gyroscope_angle -= (scaled_data.ry - scaled_data.rz * sin(GYRO_Z_AXIS_ERROR)) * 1 / this->frequency;
    float new_angle = (1.0 - ACC_PART) * gyro_angle + ACC_PART * acc_angle;
    this->last_angle = new_angle;
    return new_angle;
}

static void reset_last_angle(MPU_t *public)
{
    private_MPU_t *this = (private_MPU_t *)public;
    this->last_angle = 0;
}

static void set_last_angle(MPU_t *public, float new_last_angle)
{
    private_MPU_t *this = (private_MPU_t *)public;
    this->last_angle = new_last_angle;
}

static void reset_mount_error(MPU_t *public)
{
    private_MPU_t *this = (private_MPU_t *)public;
    this->mount_error = MOUNT_ERROR;
}

static HAL_StatusTypeDef calibrate_gyro(MPU_t *public)
{
    private_MPU_t *this = (private_MPU_t *)public;
    MPU_reading_t reading;
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    int32_t z_sum = 0;
    HAL_StatusTypeDef status = HAL_OK;
    uint16_t cal_num = CALIBRATION_ROUNDS;

    for (int i = 0; i < cal_num; ++i)
    {
        reading = GetAllRAW(this);
        x_sum += (int32_t)reading.rx;
        y_sum += (int32_t)reading.ry;
        z_sum += (int32_t)reading.rz;
        if (!reading.rx && !reading.ry && !reading.rz)
        {
            status = HAL_ERROR;
        }
        HAL_Delay(1);
    }

    this->rx_cal = x_sum / cal_num;
    this->ry_cal = y_sum / cal_num;
    this->rz_cal = z_sum / cal_num;
    char buff[30];
    sprintf(buff, "C:x%dy%dz%d", this->rx_cal, this->ry_cal, this->rz_cal);
    HAL_Delay(2);
    bt_send_message(&huart1, buff);
    return status;
}

static void set_mount_error(MPU_t *public, float (mount_error))
{
    private_MPU_t *this = (private_MPU_t *)public;
    this->mount_error = mount_error;
}

MPU_t *MPU_create(I2C_HandleTypeDef *I2C, uint16_t frequency, pin_t power_pin, uint8_t address)
{
    private_MPU_t *this = malloc(sizeof(private_MPU_t));

    *this = (private_MPU_t){
        .public = {
            .reset = reset,
            .get_acc_angle = get_acc_angle,
            .get_comp_angle = get_comp_angle,
            .calibrate_gyro = calibrate_gyro,
            .reset_last_angle = reset_last_angle,
            .reset_mount_error = reset_mount_error,
            .set_last_angle = set_last_angle,
            .set_mount_error = set_mount_error,
        },
        .I2C = I2C,
        .frequency = frequency,
        .power_pin = power_pin,
        .address = address,
        .rx_cal = 0,
        .ry_cal = 0,
        .rz_cal = 0,
        .last_angle = 0,
        .mount_error = MOUNT_ERROR,
    };
    DeviceReset(this, 1);
    SetSleepEnabled(this, 0);
    SetClockSource(this, MPU6050_CLOCK_INTERNAL);
    SetDlpf(this, MPU6050_DLPF_BW_20);
    SetFullScaleGyroRange(this, MPU6050_GYRO_FS_500);
    SetFullScaleAccelRange(this, MPU6050_ACCEL_FS_4);
    SetSleepEnabled(this, 0);
    return &(this->public);
}
