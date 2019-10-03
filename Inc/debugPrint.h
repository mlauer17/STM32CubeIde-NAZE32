/*
 * debugPrint.h
 *
 *  Created on: Sep 27, 2019
 *      Author: kamil
 */

#ifndef DEBUGPRINT_H_
#define DEBUGPRINT_H_

#include "main.h"

// UART used for debug prints
UART_HandleTypeDef huart1;


void debugPrintStr(char _out[]);
void debugPrintInt(int num);
void debugPrintFloat(float num);

#endif /* DEBUGPRINT_H_ */
