/*
 * pid.c
 *
 *  Created on: Sep 28, 2019
 *      Author: kamil
 */


#include "pid.h"


void rpy_pid_init(RPY_PID_DATA* data, RPY_PID_VAL* pid){

	// Initialize with default values
	for (int i=0;i<3;i++){
		data->sp[i] = 0;
		data->out[i] = 0;
		data->intTot[i] = 0;
		data->prevErrFilt[i] = 0;
	}

	pid->freq = 450;
	pid->Ts = 1/pid->freq;

	// Roll pid
	pid->kp[0] = 1;
	pid->ki[0] = 1;
	pid->kd[0] = 1;
	pid->kn[0] = 1;
	pid->satMax[0] = 1;
	pid->satMin[0] = 1;

	// Pitch pid
	pid->kp[1] = 1;
	pid->ki[1] = 1;
	pid->kd[1] = 1;
	pid->kn[1] = 1;
	pid->satMax[1] = 1;
	pid->satMin[1] = 1;

	// Yaw pid
	pid->kp[2] = 1;
	pid->ki[2] = 1;
	pid->kd[2] = 1;
	pid->kn[2] = 1;
	pid->satMax[2] = 1;
	pid->satMin[2] = 1;

}


void rpy_pid_update(float *meas, RPY_PID_DATA* data, RPY_PID_VAL* pid) {


	// Loop through all pid's (roll,pitch,yaw)
	for(int i = 0; i<3; i++){

		// Calculate error
		float err = data->sp[i] - meas[i];

		// Derivative filtering
		float errFilt = data->prevErrFilt[i] + pid->Ts*pid->kn[i] * (err- data->prevErrFilt[i]);

		// Derivative error
		float derErr = (errFilt - data->prevErrFilt[i]) * pid->freq;

		// Update prev filtered error
		data->prevErrFilt[i] = errFilt;

		// Integral error = ki * error * sampling time
		float intErr = pid->ki[i] * err * pid->Ts ;

		// output
		float out = pid->kp[i]*err + intErr + data->intTot[i] + pid->kd[i] * derErr;

		// Saturate output
		if (out > pid->satMax[i]) {
			data->out[i] = out;
			if (intErr < 0) data->intTot[i] += intErr;
		}
		else if (out < pid->satMin[i]) {
			data->out[i] = out;
			if (intErr > 0) data->intTot[i] += intErr;
		}
		else {
			data->out[i] = out;
			data->intTot[i] += intErr;
		}
	}
}
