/*
 * comp_filt.c
 *
 *  Created on: Sep 27, 2019
 *      Author: kamil
 */

#include "math.h"
#include "comp_filt.h"

int16_t atan_table[LEN_X*LEN_Y] = { 0 };

#define RAW_GYRO_TO_RAD 0.00013323124
#define RAW_ACC_TO_MS2 0.0005988024


/* COMP_FILT_update
 * roll and pitch calculation takes approx 150 microseconds to calculate
 * when the clock runs at 72MHz. 140 us of it is due to 2x atan2 calculation.
 * In order to improve performance a lookup table and interpolation technique
 * is used
 */

void COMP_FILT_update(int16_t *imu, COMP_FILT* DataStruct) {
	// Convert imu readings into new roll pitch and yaw
	float GYRO_GAIN = 0.95;
	float ACC_GAIN = 0.05;
	float Ts = 0.1;
	float gyro_X = (float)imu[3] * RAW_GYRO_TO_RAD;
	float gyro_Y = (float)imu[4] * RAW_GYRO_TO_RAD;

	// Acc to roll and pitch
	// roll = atan2(accY, accZ)

	//float acc_X = (float)imu[0] * RAW_ACC_TO_MS2;
	//float acc_Y = (float)imu[1] *  RAW_ACC_TO_MS2;
	//float acc_Z = (float)imu[2] *  RAW_ACC_TO_MS2;
	//float acc_roll = atan2(acc_Y,acc_Z);
	float acc_roll = atan_lookup(imu[1],imu[2]);

	// pitch = atan2(accX, acc magnitude
	// acc_pitch = atan2(-acc_X ,sqrt(acc_X^2 + acc_Y^2 + acc_Z^2 ));
	//float acc_pitch = atan2(-acc_X ,acc_Z);
	float acc_pitch = atan_lookup(-imu[0],imu[2]);

	float roll = acc_roll * ACC_GAIN + (gyro_X * Ts + DataStruct->roll) * GYRO_GAIN;
	float pitch = acc_pitch * ACC_GAIN + (gyro_Y * Ts + DataStruct->pitch) * GYRO_GAIN;

	DataStruct->roll = roll;
	DataStruct->pitch = pitch;
	DataStruct->yaw = 0.0;

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

