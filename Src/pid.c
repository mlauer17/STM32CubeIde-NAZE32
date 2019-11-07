/*
 * pid.c
 *
 *  Created on: Sep 28, 2019
 *      Author: kamil
 */


#include "pid.h"

float hover_const = 0.41666667;


void pid_init(){

	// Initialize with default values
	for (int i=0;i<3;i++){
		pid_data.intTot[i] = 0;
		pid_data.prevErrFilt[i] = 0;
		pid_data.freq[i] = 500;
		pid_data.Ts[i] = 1/pid_data.freq[i];
		pid_data.out[i] = 0;
	}


	// Roll pid
	pid_data.kp[0] = 0.1;
	pid_data.ki[0] = 0.005;
	pid_data.kd[0] = 0.03;
	pid_data.kn[0] = 100;
	pid_data.satMax[0] = 0.3;
	pid_data.satMin[0] = -0.3;

	// Pitch pid
	pid_data.kp[1] = 0.008;
	pid_data.ki[1] = 0.002;
	pid_data.kd[1] = 0.002;
	pid_data.kn[1] = 100;
	pid_data.satMax[1] = 0.05;
	pid_data.satMin[1] = -0.05;

	// Yaw pid
	pid_data.kp[2] = 0.008;
	pid_data.ki[2] = 0.002;
	pid_data.kd[2] = 0.002;
	pid_data.kn[2] = 100;
	pid_data.satMax[2] = 0.05;
	pid_data.satMin[2] = -0.05;

}


void pid_update() {


	// Loop through all pid's (roll,pitch,yaw)
	for(int i = 0; i<3; i++){

		// Calculate error
		float err = RPYA_SP[i] - RPYA[i];

		// Derivative filtering
		float errFilt = pid_data.prevErrFilt[i] + pid_data.Ts[i] * pid_data.kn[i] * (err - pid_data.prevErrFilt[i]);

		// Derivative error
		float derErr = (errFilt - pid_data.prevErrFilt[i]) * pid_data.freq[i];

		// Update prev filtered error
		pid_data.prevErrFilt[i] = errFilt;

		// Integral error = ki * error * sampling time
		float intErr = pid_data.ki[i] * err * pid_data.Ts[i];

		// output
		float out = pid_data.kp[i]*err + intErr + pid_data.intTot[i] + pid_data.kd[i] * derErr;

		// Saturate output
		if (out > pid_data.satMax[i]) {
			pid_data.out[i] = out;
			if (intErr < 0) pid_data.intTot[i] += intErr;
		}
		else if (out < pid_data.satMin[i]) {
			pid_data.out[i] = out;
			if (intErr > 0) pid_data.intTot[i] += intErr;
		}
		else {
			pid_data.out[i] = out;
			pid_data.intTot[i] += intErr;
		}
	}

	// Convert pid outputs to motor commands
	MOTOR_CMD[0] = (	pid_data.out[0] 	+ pid_data.out[1] 	+ pid_data.out[2] + hover_const);
	MOTOR_CMD[1] = (   -pid_data.out[0] 	+ pid_data.out[1] 	- pid_data.out[2] + hover_const);
	MOTOR_CMD[2] = (   -pid_data.out[0] 	- pid_data.out[1] 	+ pid_data.out[2] + hover_const);
	MOTOR_CMD[3] = (	pid_data.out[0] 	- pid_data.out[1] 	- pid_data.out[2] + hover_const);



}
