#include "DM_motor_drive.h"
#include "math.h"
#include "DJI_motor_drive.h"
#include "pid.h"
#include "stdlib.h"
#include "stdio.h"
#include "Vofa_send.h"
#include "CAN.h"


#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f


DM_motor_data_t DM_pitch_motor_data[4];

DM_motor_data_t get_DM_data(int motorID)
{
	DM_motor_data_t *target_data;
	target_data = &DM_pitch_motor_data[motorID];
	return *target_data;
}
void get_DM_motor_measure(DM_motor_data_t *ptr, uint8_t *rx_data)                                  
  {                                                                
		ptr->id = (rx_data[0])&0x0F;
		ptr->state = (rx_data[0])>>4;
		ptr->p_int=(rx_data[1]<<8)|rx_data[2];
		ptr->v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
		ptr->t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
		ptr->pos = uint_to_float(ptr->p_int, P_MIN, P_MAX, 16); 
		ptr->vel = uint_to_float(ptr->v_int, V_MIN, V_MAX, 12);
		ptr->toq = uint_to_float(ptr->t_int, T_MIN, T_MAX, 12); 
		ptr->Tmos = (float)(rx_data[6]);
		ptr->Tcoil = (float)(rx_data[7]);
                              
  }

void get_DM2_motor_measure(DM_motor2_data_t*ptr, uint8_t *rx_data)                                  
  {                                                                
    (ptr)->last_ecd = (ptr)->ecd;                                  
    (ptr)->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);           
    (ptr)->speed_rpm = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]);     
    (ptr)->given_current = (uint16_t)((rx_data)[4] << 8 | (rx_data)[5]); 
    (ptr)->temperate = (rx_data)[6];                                  
  }

void ctrl_motor1(CAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq)
{
		uint8_t TX_Data[8];
		uint32_t send_mail_box;
		CAN_TxHeaderTypeDef Tx_Msg;
	
		
		Tx_Msg.StdId=id;	 
		Tx_Msg.IDE=CAN_ID_STD;		  
		Tx_Msg.RTR=CAN_RTR_DATA;		  
		Tx_Msg.DLC=8; 

		uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
		pos_tmp = float_to_uint(_pos, -P_MAX, P_MAX, 16);
		vel_tmp = float_to_uint(_vel, -V_MAX, V_MAX, 12);
		kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
		kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
		tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
		TX_Data[0] = (pos_tmp >> 8);
		TX_Data[1] = pos_tmp;
		TX_Data[2] = (vel_tmp >> 4);
		TX_Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
		TX_Data[4] = kp_tmp;
		TX_Data[5] = (kd_tmp >> 4);
		TX_Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
		TX_Data[7] = tor_tmp;

		HAL_CAN_AddTxMessage(hcan, &Tx_Msg, TX_Data, &send_mail_box);  
}


void ctrl_motor2(CAN_HandleTypeDef *hcan,uint16_t id, float _pos, float _vel)

{
		uint8_t TX_Data[8];
		uint32_t send_mail_box;
		CAN_TxHeaderTypeDef Tx_Msg;
	
		uint8_t *pbuf,*vbuf;
		pbuf=(uint8_t*)&_pos;
		vbuf=(uint8_t*)&_vel;
	
		Tx_Msg.StdId = 0x100+id;
		Tx_Msg.IDE = CAN_ID_STD;
		Tx_Msg.RTR = CAN_RTR_DATA;
		Tx_Msg.DLC = 0x08;
	
		TX_Data[0]= *pbuf;
		TX_Data[1]= *(pbuf+1);
		TX_Data[2] = *(pbuf+2);
		TX_Data[3] = *(pbuf+3);
		TX_Data[4] = *vbuf;
		TX_Data[5] = *(vbuf+1);
		TX_Data[6] = *(vbuf+2);
		TX_Data[7] = *(vbuf+3);

		HAL_CAN_AddTxMessage(hcan, &Tx_Msg, TX_Data, &send_mail_box); 
}

void ctrl_motor3(CAN_HandleTypeDef *hcan, uint16_t id,  float _vel)

{
		uint8_t TX_Data[8];
		uint32_t send_mail_box;
		CAN_TxHeaderTypeDef Tx_Msg;
	
		uint8_t *vbuf;
		vbuf=(uint8_t*)&_vel;
	
		Tx_Msg.StdId = 0x200+id;
		Tx_Msg.IDE = CAN_ID_STD;
		Tx_Msg.RTR = CAN_RTR_DATA;
		Tx_Msg.DLC = 0x08;
	
	  TX_Data[0] = *vbuf;
	  TX_Data[1] = *(vbuf+1);
	  TX_Data[2] = *(vbuf+2);
	  TX_Data[3] = *(vbuf+3);
		HAL_CAN_AddTxMessage(hcan, &Tx_Msg, TX_Data, &send_mail_box); 
}
void ctrl_motor4(CAN_HandleTypeDef *hcan, uint16_t id, float pos, float vel, float cur)
{
	uint8_t TX_Data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;

	uint8_t *pbuf, *vbuf, *ibuf;
	
	uint16_t u16_vel = vel*100;
	uint16_t u16_cur  = cur*10000;
	
	Tx_Msg.StdId = 0x300+id;
    Tx_Msg.IDE = CAN_ID_STD;
	Tx_Msg.RTR = CAN_RTR_DATA;
	Tx_Msg.DLC = 0x08;

	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&u16_vel;
	ibuf=(uint8_t*)&u16_cur;
	
	TX_Data[0] = *pbuf;
	TX_Data[1] = *(pbuf+1);
	TX_Data[2] = *(pbuf+2);
	TX_Data[3] = *(pbuf+3);

	TX_Data[4] = *vbuf;
	TX_Data[5] = *(vbuf+1);
	
	TX_Data[6] = *ibuf;
	TX_Data[7] = *(ibuf+1);
	
	HAL_CAN_AddTxMessage(hcan, &Tx_Msg, TX_Data, &send_mail_box); 
}
void damiao_init()
{
	    
		enable_DM(&hcan2,0x01,4);
		osDelay(3);
		enable_DM(&hcan2,0x02,4);
		osDelay(3);
		enable_DM(&hcan2,0x03,4);
		osDelay(3);
		enable_DM(&hcan2,0x04,4);
		osDelay(3);
//         static int j=1;
//     while(j)
//     {
// 	      j=0;
//           for(int i=0;i<4;i++)
//    {   if(DM_pitch_motor_data[i].state==0)
//        {
//         j=1;
//         enable_DM(&hcan2,0x01+i,4);
//         osDelay(3);
//        }
//     }
//     }
 
}
void enable_DM(CAN_HandleTypeDef *hcan,uint8_t id, uint8_t ctrl_mode)
{
		uint8_t TX_Data[8];
		uint32_ t send_mail_box;
		CAN_TxHeaderTypeDef Tx_Msg;
	
		if(ctrl_mode==1)		Tx_Msg.StdId=0x000+id;	 
		else if(ctrl_mode==2)	Tx_Msg.StdId=0x100+id;	 
		else if(ctrl_mode==3)	Tx_Msg.StdId=0x200+id;	 
		else if(ctrl_mode==4)	Tx_Msg.StdId=0x300+id;	
		Tx_Msg.IDE=CAN_ID_STD;		  
		Tx_Msg.RTR=CAN_RTR_DATA;		 
		Tx_Msg.DLC=8; 
	
		TX_Data[0] = 0xff;
		TX_Data[1] = 0xff;
		TX_Data[2] = 0xff;
		TX_Data[3] = 0xff;
		TX_Data[4] = 0xff;
		TX_Data[5] = 0xff;
		TX_Data[6] = 0xff;
		TX_Data[7] = 0xfc;

		HAL_CAN_AddTxMessage(hcan, &Tx_Msg, TX_Data, &send_mail_box);   
}


void disable_DM(CAN_HandleTypeDef *hcan,uint8_t id, uint8_t ctrl_mode)
{
		uint8_t TX_Data[8];
		uint32_t send_mail_box;
		CAN_TxHeaderTypeDef Tx_Msg;
	
		if(ctrl_mode==1)			Tx_Msg.StdId=0x000+id;	
		else if(ctrl_mode==2)	Tx_Msg.StdId=0x100+id;	 
		else if(ctrl_mode==3)	Tx_Msg.StdId=0x200+id;	 

		Tx_Msg.IDE=CAN_ID_STD;		 
		Tx_Msg.RTR=CAN_RTR_DATA;		  
		Tx_Msg.DLC=8; 
	
		TX_Data[0] = 0xff;
		TX_Data[1] = 0xff;
		TX_Data[2] = 0xff;
		TX_Data[3] = 0xff;
		TX_Data[4] = 0xff;
		TX_Data[5] = 0xff;
		TX_Data[6] = 0xff;
		TX_Data[7] = 0xfd;

		HAL_CAN_AddTxMessage(hcan, &Tx_Msg, TX_Data, &send_mail_box);    
}





void dm_current_set(CAN_HandleTypeDef *hcan,uint16_t id, float m1_cur_set, float m2_cur_set, float m3_cur_set, float m4_cur_set)
{
	uint8_t data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;
	int16_t m1_cur_tmp = m1_cur_set*(16384.0f/20.0f);
	int16_t m2_cur_tmp = m2_cur_set*(16384.0f/20.0f);
	int16_t m3_cur_tmp = m3_cur_set*(16384.0f/20.0f);
	int16_t m4_cur_tmp = m4_cur_set*(16384.0f/20.0f);
	Tx_Msg.IDE=CAN_ID_STD;		 
	Tx_Msg.RTR=CAN_RTR_DATA;		  
	Tx_Msg.DLC=8; 
	Tx_Msg.StdId=id;
	data[0] = (m1_cur_tmp >> 8);
	data[1] = m1_cur_tmp;
	
	data[2] = (m2_cur_tmp >> 8);
	data[3] = m2_cur_tmp;
	
	data[4] = (m3_cur_tmp >> 8);
	data[5] = m3_cur_tmp;
	
	data[6] = (m4_cur_tmp >> 8);
	data[7] = m4_cur_tmp;
	
HAL_CAN_AddTxMessage(hcan, &Tx_Msg, data, &send_mail_box); 
}

void save_pos_zero(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;
	Tx_Msg.StdId = id;
	Tx_Msg.IDE = CAN_ID_STD;
	Tx_Msg.RTR = CAN_RTR_DATA;
	Tx_Msg.DLC = 0x08;
	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = 0xff;
	data[3] = 0xff;
	data[4] = 0xff;
	data[5] = 0xff;
	data[6] = 0xff;
	data[7] = 0xfe;
	
	HAL_CAN_AddTxMessage(hcan, &Tx_Msg, data, &send_mail_box);
}