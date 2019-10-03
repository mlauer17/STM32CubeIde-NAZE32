/*
 * debug_print.cpp
 *
 *  Created on: Sep 27, 2019
 *      Author: kamil
 */

#include <debugPrint.h>
#include "stdio.h"
#include "string.h"



void debugPrintStr(char _out[]){
	HAL_UART_Transmit(&huart1, (uint8_t *) _out, strlen(_out), 10);
}

void debugPrintInt(int num){

	char str_num[10];

	// convert 123 to string [buf]
	itoa(num, str_num, 10);

	debugPrintStr(str_num);


}


void debugPrintFloat(float num) {

	//gcvt (float value, int ndigits, char * buf);

	//float value : It is the float or double value.
	//int ndigits : It is number of digits.
	//char * buf : It is character pointer, in this
	//variable string converted value will be copied.

	char str_num[10];
	int ndigits = 5;
	gcvt(num, ndigits, &str_num);
	debugPrintStr(str_num);


}






