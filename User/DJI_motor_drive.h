#ifndef __DJI_motor_drive_H__
#define __DJI_motor_drive_H__

#include "struct_typedef.h"
#include "string.h"
#include "CAN.h"

#define CAN_1 1
#define CAN_2 2


typedef enum
{
  CAN_1_4_SIGN_ID = 0x200,
  CAN_ID1 = 0x201,
  CAN_ID2 = 0x202,
  CAN_ID3 = 0x203,
  CAN_ID4 = 0x204,

  CAN_5_8_SIGN_ID = 0x1FF,
  CAN_ID5 = 0x205,
  CAN_ID6 = 0x206,
  CAN_ID7 = 0x207,
  CAN_ID8 = 0x208,
} can_msg_id_e;


#define chassis_FL CAN_ID1
#define chassis_FR CAN_ID2
#define chassis_BR CAN_ID3
#define chassis_BL CAN_ID4
#define Gimbal     CAN_ID5
#define M2006      CAN_ID6


#define ECD_MAX 8191  
#define M3508_P 13.733f 




typedef struct
{
  int16_t set_current; 

  uint16_t ecd;          
  int16_t speed_rpm;     
  int16_t given_current; 
  uint8_t temperate;     
  int16_t last_ecd;      

  long long ecd_cnt; 
  float angle_cnt;   
  float round_speed; 

} motor_measure_t;




typedef struct
{
	uint8_t  lenth;
	uint8_t  add;
	uint32_t  data;
} encoder_t;



void CAN1_send_current(void);                                        
void CAN2_send_current(void);                                        
void set_motor_current(int16_t current, uint8_t CAN_x, int motorID); 
void get_DJI_motor_measure(motor_measure_t *ptr, uint8_t *rx_data) ;
motor_measure_t get_DJI_motor_data(uint8_t CAN_x, int motorID);          
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void process_motor_data(CAN_HandleTypeDef *hcan, motor_measure_t *motor_data, int motor_ID);

#endif
