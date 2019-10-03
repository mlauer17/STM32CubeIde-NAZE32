/*
 * pid.h
 *
 *  Created on: Sep 28, 2019
 *      Author: kamil
 */

#ifndef PID_H_
#define PID_H_


typedef struct {
	float sp[3]; 			// Stores setpoints
	float out[3]; 			// stores outputs
	float intTot[3]; 		// stores total integral error
	float prevErrFilt[3];	//
} RPY_PID_DATA;


typedef struct  {
	/* Private */
	float kp[3];
	float ki[3];
	float kd[3];
	float kn[3];
	float satMax[3];
	float satMin[3];
	float Ts;
	float freq;

} RPY_PID_VAL;

void rpy_pid_update(float *meas, RPY_PID_DATA* data, RPY_PID_VAL* pid);

void rpy_pid_init(RPY_PID_DATA* data, RPY_PID_VAL* pid);


#endif /* PID_H_ */

