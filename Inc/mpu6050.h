/*
 * mpu6050.h
 *
 *  Created on: Oct 4, 2019
 *      Author: kamil
 */

#ifndef MPU6050_H_
#define MPU6050_H_


#include "stm32f1xx_hal.h"

// Defined in main.h. array that holds raw imu readings
extern int16_t IMU[6];



#define MPU6050_I2C_ADDR			0xD0
#define MPU6050_I2C_ADDW			0xD1

#define MPU6050_I_AM				0x70 // 0x70 // 0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19 // Register 25 –Sample Rate Divider
#define MPU6050_CONFIG				0x1A // Register 26 –Configuration
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37 //Register 55 –INT Pin / Bypass Enable Configuration
#define MPU6050_INT_ENABLE			0x38 //Register 56 –Interrupt Enable
#define MPU6050_INT_STATUS			0x3A

#define MPU6050_FIFO_EN 			0x23 // Register 35 –FIFO Enable

#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48


#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75




/*
#define SD_MPU6050_DataRate_8KHz       0   /!< Sample rate set to 8 kHz /
#define SD_MPU6050_DataRate_4KHz       1   /!< Sample rate set to 4 kHz /
#define SD_MPU6050_DataRate_2KHz       3   /!< Sample rate set to 2 kHz /
#define SD_MPU6050_DataRate_1KHz       7   /!< Sample rate set to 1 kHz /
#define SD_MPU6050_DataRate_500Hz      15  /!< Sample rate set to 500 Hz /
#define SD_MPU6050_DataRate_250Hz      31  /!< Sample rate set to 250 Hz /
#define SD_MPU6050_DataRate_125Hz      63  /!< Sample rate set to 125 Hz /
#define SD_MPU6050_DataRate_100Hz      79  /!< Sample rate set to 100 Hz /
*/
#define MPU6050_DATA_RATE 0x0F // 500Hz sample rate

// Digital low pass filter settings
#define MPU6050_DLPF_CFG 0x00

// Available accelerometer ranges
// 0x00 	+/- 2G
// 0x01 	+/- 4G
// 0x02 	+/- 8G
// 0x03 	+/- 16G
#define MPU6050_ACC_RANGE 0x03 << 3


// Available gyroscope ranges
// 0x00 	+/- 250 deg/s
// 0x01 	+/- 500 deg/s
// 0x02 	+/- 1000 deg/s
// 0x03 	+/- 2000 deg/s
#define MPU6050_GYRO_RANGE 0x03 << 3


int MPU6050_ReadGyroscope(I2C_HandleTypeDef* I2Cx);
int MPU6050_ReadAccelerometer(I2C_HandleTypeDef* I2Cx);
int MPU6050_Init(I2C_HandleTypeDef* I2Cx);

#endif /* MPU6050_H_ */
