
#include "main.h"
#include "comm.h"



void COMM_parse_cmd(void){
	//char msg[4] = {'t','e','s','t'};
	//HAL_UART_Transmit(&huart1,msg,4, 4);
	//HAL_UART_Transmit_DMA(&huart1,buffer_tx,10);


	FLOATUINT_t cmd[4];
	for (int i=0;i<4;i++){
		for (int j=0;j<4;j++){
			cmd[i].bytes[j] = buffer_rx[i*4+j];
		}
	}

	RPYA_SP[0] = cmd[0].number;
	RPYA_SP[1] = cmd[1].number;
	RPYA_SP[2] = cmd[2].number;
	RPYA_SP[3] = cmd[3].number;
}

void COMM_send_states(void){

	// ADD ALTITUDE!

	// Send roll pitch and yaw
	for (int i=0;i<4;i++){
	  FLOATUINT_t rpy_bytes;
	  rpy_bytes.number = RPYA[i];
	  for (int j=0;j<4;j++){
		  buffer_tx[j+i*4]= rpy_bytes.bytes[j];
	  }
	}
}



void COMM_send_rawSens(void){

	// Send raw imu data
	for (int i =0 ; i<6;i++){
	  INT16UINT_t imu_bytes;
	  imu_bytes.number = IMU[i];
	  buffer_tx[i*2] = imu_bytes.bytes[0];
	  buffer_tx[i*2+1] = imu_bytes.bytes[1];
	}
}


// Timer 1 overflow interrupt callback.
// Used to run pid at fixed intervals
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim1.Instance){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

		float RPY[2];
		RPY[0] = comp_filt.roll;
		RPY[1] = comp_filt.pitch;
		RPY[2] = comp_filt.yaw;

		// update pid. outputs available in rpy_pid_data->out[i]
		rpy_pid_update(RPY, &rpy_pid_data,&rpy_pid_val);

		// Full throttle, 0 throttle, setup value
		// 48000 = 2ms, 24000 = 1ms, 16800 = 0.7ms
		pwm_setvalue(48000, TIM_CHANNEL_1);
		pwm_setvalue(48000, TIM_CHANNEL_2);
		pwm_setvalue(48000, TIM_CHANNEL_3);
		pwm_setvalue(48000, TIM_CHANNEL_4);


	}

}
*/
