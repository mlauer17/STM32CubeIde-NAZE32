/*
 * pid.h
 *
 *  Created on: Sep 28, 2019
 *      Author: kamil
 */

#ifndef PID_H_
#define PID_H_

extern float RPYA[4];
extern float RPYA_SP[4];

extern float MOTOR_CMD[4];

typedef struct  {
	/* Private */
	float kp[3];
	float ki[3];
	float kd[3];
	float kn[3];
	float satMax[3];
	float satMin[3];
	float Ts[3];
	float freq[3];
	float intTot[3]; 		// stores total integral error
	float prevErrFilt[3];	//
	float out[3];

} PID_DATA;

PID_DATA pid_data;

void pid_update();

void pid_init();


#endif /* PID_H_ */

