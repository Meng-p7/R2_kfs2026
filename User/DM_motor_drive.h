#ifndef DA_MIAO_H
#define DA_MIAO_H

#include "main.h"
#include "string.h"
#include "math.h"
#include "struct_typedef.h"

typedef struct{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float toq;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
	

	uint16_t target_pos;
	int16_t  target_vel;
	int16_t  target_current;
	fp32 INS_speed;
    fp32 INS_speed_set;
    fp32 INS_angle1;
    fp32 INS_angle_set;
	
}DM_motor_data_t;
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

}DM_motor2_data_t;

DM_motor_data_t get_DM_data(int motorID);
void get_DM_motor_measure(DM_motor_data_t *ptr, uint8_t *rx_data);  
void get_DM2_motor_measure(DM_motor2_data_t*ptr, uint8_t *rx_data);         
void enable_DM(CAN_HandleTypeDef *hcan,uint8_t id, uint8_t ctrl_mode);
void disable_DM(CAN_HandleTypeDef *hcan,uint8_t id, uint8_t ctrl_mode);
void ctrl_motor1(CAN_HandleTypeDef *hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);
void ctrl_motor2(CAN_HandleTypeDef *hcan,uint16_t id, float _pos, float _vel);
void ctrl_motor3(CAN_HandleTypeDef *hcan,uint16_t id,  float _vel);
void ctrl_motor4(CAN_HandleTypeDef *hcan, uint16_t id, float pos, float vel, float cur);
void dm_current_set(CAN_HandleTypeDef *hcan,uint16_t id, float m1_cur_set, float m2_cur_set, float m3_cur_set, float m4_cur_set);
void damiao_init(void);
void save_pos_zero(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);

#endif
