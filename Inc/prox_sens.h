/*
    Name: prox_sens.h
    Description: This file provides functions to initialize and read Ultrasonic Sensor
    Date: 2019-10-21
    Author(s): Martin Lauersen
*/

#ifndef PROX_SENS_H_
#define PROX_SENS_H_


/*
    Function: prox_Start
    Description: Sends the initial pulse to the proximity sensor
    Parameters: N/A
    Returns: N/A
 */
void prox_Start(void);

/*
    Function: prox_Interrupt
    Description: This function is called in the interrupt
    Parameters: N/A
    Returns: N/A
 */
void prox_Interrupt(void);

/*
    Function: prox_Distance
    Description: Calculates measured distance, using size of counter
    Parameters: float -> counter_Val -> Value of the counter of timer
    Returns: float -> Measured distance in meters(?)
 */
float prox_DistCalc(float counter_Val);

#endif /* PROX_SENS_H_ */
