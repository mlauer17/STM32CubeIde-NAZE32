/*
 * comm.h
 *
 *  Created on: Oct 25, 2019
 *      Author: kamil
 */

#ifndef COMM_H_
#define COMM_H_

// Buffers used by UART1 in DMA mode
extern uint8_t buffer_tx[16];
extern uint8_t buffer_rx[16];

// raw sensor measurements. acc XYZ, gyro XYZ
extern int16_t IMU[6];

// roll pitch yaw and altitude estimate
extern float RPYA[4];

// Setpoints
extern float RPYA_SP[4];


typedef union
{
 float number;
 uint8_t bytes[4];
} FLOATUINT_t;

typedef union
{
 int16_t number;
 uint8_t bytes[2];
} INT16UINT_t;


void COMM_parse_cmd(void);
void COMM_send_states(void);
void COMM_send_rawSens(void);


#endif /* COMM_H_ */
