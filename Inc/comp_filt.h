/*
 * comp_filt.h
 *
 *  Created on: Sep 27, 2019
 *      Author: kamil
 */

#ifndef COMP_FILT_H_
#define COMP_FILT_H_


#include <main.h>

extern int16_t IMU[6];
extern float RPYA[4];



//#define ACC_SCALE

#define ACC_b0 0.013359200027857
#define ACC_b1 2*0.013359200027857
#define ACC_b2 0.013359200027857
#define ACC_a1 -1.647459981076977
#define ACC_a2 0.700896781188403


#define GYRO_b0 0.991153597831726
#define GYRO_b1 -1.98230719566345
#define GYRO_b2 0.991153597831726
#define GYRO_a1 -1.98222887516022
#define GYRO_a2 0.982385456562042


typedef struct {
	float wz0[3];
	float wz1[3];
} FILT;


typedef struct  {
	/* Private */
	float roll;
	float pitch;
	float yaw;
} COMP_FILT;


#define INT_TO_MS2 float(1/1670)

// lookup for atan function //

// Y vals should be in range 8192 which corresponds to g accel
// when roll and pitch are 45 degrees at the same time.
// upper range is choosen to be 20k. verify those values later!
// values updated to fit a range with 256 res. it will be then
// possible to speed up division by bit shifting
#define Y_MIN 7856
#define Y_MAX 20144

// X min is 0 because negative values just change the sign of the atan2
// max is 12k which corresponds to slightly more than 45deg tilt
#define X_MIN 0
#define X_MAX 12288

// number of array elements is then xMax-xMin + yMax-yMin
// so 24k. with a resolution of 250 (250 difference between precalculated
// values) it results in 48x48 array. so 2304 elements
#define RES_LOOKUP 256
#define RES_SHIFT_DIVIDE 8
#define LEN_X 48
#define LEN_Y 48

// Values in the lookup table are stored as 16 bit integers that
// can be converted to float with the following relationship
// val_rad * floatPiToInt = int_16 (range -2 to 2 pi)
#define floatPiToInt16 10430.378  // 1(PI/2^15)
#define int16PiToFloat 1/floatPiToInt16




void COMP_FILT_update(COMP_FILT* DataStruct);
float lowpass_filt(int16_t x, int index, FILT* acc_filt);
float highpass_filt(int16_t x, int index, FILT* gyro_filt);
void filt_init(FILT* filt);

void atan_lookup_init(int xMin, int xMax, int yMin, int yMax, int delta);
float atan_lookup(int16_t x, int16_t y);


#endif /* COMP_FILT_H_ */
