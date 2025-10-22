#include "DJI_motor_drive.h"
#include "math.h"
#include "can.h"
#include "main.h"
#include "string.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

encoder_t CAN2_encoder_data[2];
motor_measure_t Can1_motor_data[8];
motor_measure_t Can2_motor_data[8];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2; 



void get_encoder_measuer(encoder_t *encoder,uint8_t *rx_data)
{
	  encoder->lenth=rx_data[0];
	  encoder->add=rx_data[1];
	  encoder->data=(uint32_t)(rx_data[7]<<6|rx_data[6]<<4|rx_data[5]<<2|rx_data[4]);
}	
void get_DJI_motor_measure(motor_measure_t *ptr, uint8_t *rx_data)                                  
  {                                                                
    (ptr)->last_ecd = (ptr)->ecd;                                  
    (ptr)->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);           
    (ptr)->speed_rpm = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]);     
    (ptr)->given_current = (uint16_t)((rx_data)[4] << 8 | (rx_data)[5]); 
    (ptr)->temperate = (rx_data)[6];                                  
  }

void process_motor_data(CAN_HandleTypeDef *hcan, motor_measure_t *motor_data, int motor_ID)
{
  
		UNUSED(motor_ID);
		UNUSED(hcan); 

		if (motor_data->last_ecd > 7000 && motor_data->ecd < 2000)
				motor_data->ecd_cnt += ((ECD_MAX - motor_data->last_ecd) + motor_data->ecd);
		else if (motor_data->last_ecd < 2000 && motor_data->ecd > 7000)
				motor_data->ecd_cnt -= ((ECD_MAX - motor_data->ecd) + motor_data->last_ecd);
		else
				motor_data->ecd_cnt += (motor_data->ecd - motor_data->last_ecd);

  
		motor_data->round_speed = motor_data->speed_rpm / 60.0f / M3508_P;

		motor_data->angle_cnt = motor_data->ecd_cnt / ECD_MAX / M3508_P * 360.00f;
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
		float span = x_max - x_min;
		float offset = x_min;
		return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
		float span = x_max - x_min;
		float offset = x_min;
		return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}



motor_measure_t get_DJI_motor_data(uint8_t CAN_x, int motorID) 
{
		motor_measure_t *target_data;
		motorID -= 0x201;
		if (CAN_x == CAN_1)
				target_data = &Can1_motor_data[motorID];
		else if (CAN_x == CAN_2)
				target_data = &Can2_motor_data[motorID];
		return *target_data;
}


void set_motor_current(int16_t current, uint8_t CAN_x, int motorID)
{
		motorID -= 0x201;
		if (CAN_x == CAN_1)
				Can1_motor_data[motorID].set_current = current;
		else if (CAN_x == CAN_2)
				Can2_motor_data[motorID].set_current = current;
}


void CAN1_send_current()
{
		uint8_t can_send_data[8];
		static CAN_TxHeaderTypeDef tx_message;
		uint32_t send_mail_box;

		tx_message.StdId = CAN_1_4_SIGN_ID;
		tx_message.IDE = CAN_ID_STD;
		tx_message.RTR = CAN_RTR_DATA;
		tx_message.DLC = 0x08;

		can_send_data[0] = (Can1_motor_data[0].set_current >> 8);
		can_send_data[1] = Can1_motor_data[0].set_current;

		can_send_data[2] = (Can1_motor_data[1].set_current >> 8);
		can_send_data[3] = Can1_motor_data[1].set_current;

		can_send_data[4] = (Can1_motor_data[2].set_current >> 8);
		can_send_data[5] = Can1_motor_data[2].set_current;

		can_send_data[6] = (Can1_motor_data[3].set_current >> 8);
		can_send_data[7] = Can1_motor_data[3].set_current;
		HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);

	
		tx_message.StdId = CAN_5_8_SIGN_ID;

		can_send_data[0] = (Can1_motor_data[4].set_current >> 8);
		can_send_data[1] = Can1_motor_data[4].set_current + 4;

		can_send_data[2] = (Can1_motor_data[5].set_current >> 8);
		can_send_data[3] = Can1_motor_data[5].set_current;

		can_send_data[4] = (Can1_motor_data[6].set_current >> 8);
		can_send_data[5] = Can1_motor_data[6].set_current;

		can_send_data[6] = (Can1_motor_data[7].set_current >> 8);
		can_send_data[7] = Can1_motor_data[7].set_current;
		
		HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
}

void CAN2_send_current() 
{
		uint8_t can_send_data[8];
		static CAN_TxHeaderTypeDef tx_message;
		uint32_t send_mail_box;

		
		tx_message.StdId = CAN_1_4_SIGN_ID;
		tx_message.IDE = CAN_ID_STD;
		tx_message.RTR = CAN_RTR_DATA;
		tx_message.DLC = 0x08;

		can_send_data[0] = (Can2_motor_data[0].set_current >> 8);
		can_send_data[1] = Can2_motor_data[0].set_current;

		can_send_data[2] = (Can2_motor_data[1].set_current >> 8);
		can_send_data[3] = Can2_motor_data[1].set_current;

		can_send_data[4] = (Can2_motor_data[2].set_current >> 8);
		can_send_data[5] = Can2_motor_data[2].set_current;

		can_send_data[6] = (Can2_motor_data[3].set_current >> 8);
		can_send_data[7] = Can2_motor_data[3].set_current;
		HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);

		tx_message.StdId = CAN_5_8_SIGN_ID;

		can_send_data[0] = (Can2_motor_data[4].set_current >> 8);
		can_send_data[1] = Can2_motor_data[4].set_current + 4;

		can_send_data[2] = (Can2_motor_data[5].set_current >> 8);
		can_send_data[3] = Can2_motor_data[5].set_current;

		can_send_data[4] = (Can2_motor_data[6].set_current >> 8);
		can_send_data[5] = Can2_motor_data[6].set_current;

		can_send_data[6] = (Can2_motor_data[7].set_current >> 8);
		can_send_data[7] = Can2_motor_data[7].set_current;
		
		HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
}
