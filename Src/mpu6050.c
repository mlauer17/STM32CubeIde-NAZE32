/*
 * mpu6050.c
 *
 *  Created on: Oct 4, 2019
 *      Author: kamil
 */

#include "mpu6050.h"



				//acc X Y Z, gyro X Y Z
int16_t IMU_BIAS[6] = {-17,-25,22,13,12,-26};


int MPU6050_Init(I2C_HandleTypeDef* I2Cx)
{
	uint8_t WHO_AM_I = (uint8_t)MPU6050_WHO_AM_I;
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t d[2];


	// Check if device is connected
	if(HAL_I2C_IsDeviceReady(Handle,MPU6050_I2C_ADDR,5,10)!=HAL_OK){return 2;}

	if(HAL_I2C_Master_Transmit(Handle, MPU6050_I2C_ADDR, &WHO_AM_I, 1, 1000) != HAL_OK){return 3;}

	// Receive who am i response
	if(HAL_I2C_Master_Receive(Handle, MPU6050_I2C_ADDR, &temp, 1, 1000) != HAL_OK){return 4;}

	/* Checking */
	while(temp != MPU6050_I_AM){return 5;} // device invalid


	// Wakeup MPU6050
	d[0] = MPU6050_PWR_MGMT_1;
	d[1] = 0x00;
	if(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDR, (uint8_t *)d, 2, 1000) != HAL_OK){return 6;}

	// Set sample rate
	d[0] = MPU6050_SMPLRT_DIV;
	d[1] = MPU6050_DATA_RATE;
	while(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDW,&d,2,1000)!=HAL_OK);

	// Config accelerometer
	d[0] = MPU6050_ACCEL_CONFIG;
	d[1] = MPU6050_ACC_RANGE;
	while(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDW,&d, 2, 1000) != HAL_OK);

	// Config Gyroscope
	d[0] = MPU6050_GYRO_CONFIG;
	d[1] = MPU6050_GYRO_RANGE;
	while(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDR,&d, 2, 1000) != HAL_OK);

	// Set digital low pass filter
	d[0] = MPU6050_CONFIG;
	d[1] = MPU6050_DLPF_CFG;
	while(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDR,&d, 2, 1000) != HAL_OK);

	// Interrupt pin config
	//d[0] = MPU6050_INT_ENABLE;
	//d[1] = 0x01;
	//while(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDR,&d, 2, 1000) != HAL_OK);

	// Enable interrupt
	//d[0] = MPU6050_INT_PIN_CFG;
	//d[1] = 0x00;
	//while(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDR,&d, 2, 1000) != HAL_OK);



	// FIFO buffer enable
	//regAdd = (uint8_t)MPU6050_FIFO_EN;
	//while(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDR,&regAdd, 1, 1000) != HAL_OK);
	//while(HAL_I2C_Master_Receive(Handle,MPU6050_I2C_ADDR, &temp, 1, 1000) != HAL_OK);
	//temp = 0x78;
	//while(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDR,&temp, 1, 1000) != HAL_OK);

	return 1;
}



int MPU6050_ReadAccelerometer(I2C_HandleTypeDef* I2Cx)
{
	uint8_t data[6];
	uint8_t reg = MPU6050_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;

	/* Read accelerometer data */
	while(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDR, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle,MPU6050_I2C_ADDR, data, 6, 1000) != HAL_OK);

	/* Format */
	IMU[0] = -(((int16_t)(data[0] << 8 | data[1])) + IMU_BIAS[0]); // Flip sign to fit the convention
	IMU[1] = (int16_t)(data[2] << 8 | data[3]) + IMU_BIAS[1];
	IMU[2] = (int16_t)(data[4] << 8 | data[5]) + IMU_BIAS[2];

	/* Return OK */
	return 1;
}
int MPU6050_ReadGyroscope(I2C_HandleTypeDef* I2Cx)
{
	uint8_t data[6];
	uint8_t reg = MPU6050_GYRO_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;

	/* Read gyroscope data */
	while(HAL_I2C_Master_Transmit(Handle,MPU6050_I2C_ADDR, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle,MPU6050_I2C_ADDR, data, 6, 1000) != HAL_OK);

	/* Format */
	IMU[3] = (int16_t)(data[0] << 8 | data[1]) + IMU_BIAS[3];
	IMU[4] = -(((int16_t)(data[2] << 8 | data[3])) + IMU_BIAS[4]); // Flip sign to fit the convention
	IMU[5] = -(((int16_t)(data[4] << 8 | data[5])) + IMU_BIAS[5]); // Flip sign to fit the convention

	/* Return OK */
	return 1;
}

/*
int SD_MPU6050_ReadTemperature(I2C_HandleTypeDef* I2Cx)
{
	uint8_t data[2];
	int16_t temp;
	uint8_t reg = MPU6050_TEMP_OUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	// Read temperature
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 2, 1000) != HAL_OK);

	// Format temperature
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	// Return OK
	return 1;
}

*/
