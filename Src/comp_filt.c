/*
 * comp_filt.c
 *
 *  Created on: Sep 27, 2019
 *      Author: kamil
 */

#include "math.h"
#include "comp_filt.h"



FILT acc_filt;
FILT gyro_filt;


int16_t atan_table[LEN_X*LEN_Y] = { 0 };

//#define RAW_GYRO_TO_RAD 0.00013323124 // 250deg/s
//#define RAW_GYRO_TO_RAD 0.00053263223 // 1000deg/s
#define RAW_GYRO_TO_RAD 0.0010654012 // 2000deg/s
#define Ts 1/500
#define GYRO_GAIN 0.99
#define ACC_GAIN  (1 - GYRO_GAIN)


/* COMP_FILT_update
 * roll and pitch calculation takes approx 150 microseconds to calculate
 * when the clock runs at 72MHz. 140 us of it is due to 2x atan2 calculation.
 * In order to improve performance a lookup table and interpolation technique
 * is used
 */

void COMP_FILT_update(COMP_FILT* DataStruct) {
	// Convert imu readings into new roll pitch and yaw

	//float gyro_X = (float)imu[3] * RAW_GYRO_TO_RAD;
	//float gyro_Y = (float)imu[4] * RAW_GYRO_TO_RAD;

	// Highpass the gyro




	// Lowpass the accelerometer
	/*
	float accX = lowpass_filt((float)imu[0],0, &acc_filt);
	float accY = lowpass_filt((float)imu[1],1, &acc_filt);
	float accZ = lowpass_filt((float)imu[2],2, &acc_filt);
	*/

	float accX = (float)IMU[0];
	float accY = (float)IMU[1];
	float accZ = (float)IMU[2];


	float acc_roll = atan2(accY,accZ);
	float acc_pitch = atan2(-accX,accZ);

	/*
	float gyroX = highpass_filt((float)imu[3]* RAW_GYRO_TO_RAD,0, &gyro_filt);
	float gyroY = highpass_filt((float)imu[4]* RAW_GYRO_TO_RAD,1, &gyro_filt);
	float gyroZ = highpass_filt((float)imu[5]* RAW_GYRO_TO_RAD,2, &gyro_filt);
	*/

	float gyroX = (float)IMU[3]* RAW_GYRO_TO_RAD;
	float gyroY = (float)IMU[4]* RAW_GYRO_TO_RAD;
	//float gyroZ = (float)IMU[5]* RAW_GYRO_TO_RAD;


	float roll = acc_roll * ACC_GAIN + (gyroX * Ts + RPYA[0]) * GYRO_GAIN;
	float pitch = acc_pitch * ACC_GAIN + (gyroY * Ts + RPYA[1]) * GYRO_GAIN;

	RPYA[0] = roll;
	RPYA[1] = pitch;
	RPYA[2] = 0.0;

}

void filt_init(FILT* filt){
	for (int i =0;i<3;i++){
		filt->wz0[i]=0;
		filt->wz1[i]=0;
	}
}

float lowpass_filt(int16_t x, int i, FILT* acc_filt){
	  float y = ACC_b0 * x + acc_filt->wz0[i];
	  acc_filt->wz0[i] = ACC_b1 * x - ACC_a1 * y + acc_filt->wz1[i];
	  acc_filt->wz1[i] = ACC_b2 * x - ACC_a2 * y;
	  return y;
}

float highpass_filt(int16_t x, int i, FILT* gyro_filt){
	  float y = GYRO_b0 * x + gyro_filt->wz0[i];
	  gyro_filt->wz0[i] = GYRO_b1 * x - GYRO_a1 * y + gyro_filt->wz1[i];
	  gyro_filt->wz1[i] = GYRO_b2 * x - GYRO_a2 * y;
	  return y;
}


void atan_lookup_init(int xMin, int xMax, int yMin, int yMax, int delta){
	for (int x=0;x<LEN_X;x++){
		for (int y=0;y<LEN_Y;y++){
			atan_table[x*LEN_X+y] = atan2(xMin + x*delta, yMin + y*delta)*floatPiToInt16;
		}
	}
}

float atan_lookup(int16_t x, int16_t y){
    // calculate the indices. remember that its 1d array so the value is at
    // x_index * LEN_X + y_index

	// if x is negative compute everything as for positive and change the output
	// to negative
	uint8_t neg_sign = 0;
	if (x<0){
		x=-x;
		neg_sign = 1;
	}

    // taking advantage from the fact that resolution is set to 256
    // the division operation can be performed by bit shifting left by 8
    int x_index = x>>RES_SHIFT_DIVIDE;
    int y_index = (y-Y_MIN)>>RES_SHIFT_DIVIDE;

    // Check if the values are covered by the lookup table. if not then
    // calculate the atan the usual way.
    if ((x_index >= LEN_X) || (y_index >= LEN_Y)){
    	if (neg_sign == 1){
    		return atan2(-x,y);
    	}
    	else {
    		return atan2(x,y);
    	}
    }

    // Get the values from lookup table
    int16_t base_val = atan_table[x_index*LEN_X+y_index];
    int16_t next_x_val = atan_table[(x_index+1)*LEN_X+y_index];
    int16_t next_y_val = atan_table[x_index*LEN_X+y_index+1];

    // compute reminder from the division. because the resolution is 256
    // modulo operation can be translated to bitwise AND. val%divisior
    // is val & divisor-1. 256-1 = hex 0xFF
    int16_t rem_x = x & 0xFF;
    int16_t rem_y = y & 0xFF;

    // compute change in values for interval from base to next_x* mod(x,interval)
    //
    int dx = ((next_x_val - base_val)* rem_x ) >> RES_SHIFT_DIVIDE;
    int dy = ((next_y_val - base_val)* rem_y ) >> RES_SHIFT_DIVIDE;

    int output = base_val + dx +dy;

    if (neg_sign == 1) {
    	return -output*int16PiToFloat;
    }
    else {
    	return output*int16PiToFloat;
    }
}

