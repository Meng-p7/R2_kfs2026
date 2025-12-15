#include "chassis_move.h"
#include "math.h"
#include "DJI_motor_drive.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "stdlib.h"
#include "stdio.h"
#include "DBUS_remote_control.h"
#include "Vofa_send.h"
#include "user_lib.h"
#include "DM_motor_drive.h"

//#define WHEEL_RADIUS 0.15240f	 
//#define ELEVATOR_WHEEL_RADIUS 0.0145f	
//#define PI 3.1415926f



static double wheel_rpm[4];	
#define WHEELBASE_HALF 0.21f   // 前后轮距的一半
#define TRACK_WIDTH_HALF 0.255f // 左右轮距的一半
#define WHEEL_RADIUS 0.0905f    // 轮子半径
float PID_SPEED[3] = {950.0f, 260.0f, 660.0f};
pid_type_def motor_speed[4];
int16_t wheel_current[4]={0,0,0,0}; 
int16_t move[4]={0,0,0,0}; 
extern  volatile uint8_t remote_power_off ;
// 预先计算常用组合
const float L_plus_W = WHEELBASE_HALF + TRACK_WIDTH_HALF;
const float inv_radius = 1.0f / WHEEL_RADIUS;
extern Chassis_Data_Rx Chassis_Data_Receive;
//Encoder2_t encoder2;
//extern int16_t back;
CurrentFilter filter1;

void chassis_move_init()
{  
    PID_init(&motor_speed[0], PID_POSITION, PID_SPEED, 7000, 5000);
    PID_init(&motor_speed[1], PID_POSITION,PID_SPEED, 7000, 5000);
    PID_init(&motor_speed[2], PID_POSITION, PID_SPEED, 7000, 5000);
    PID_init(&motor_speed[3], PID_POSITION, PID_SPEED, 7000, 5000);
  damiao_init();	
filter1_init(&filter1, 0.6f);
}

void val_limit(float *val, float MAX)
{
    if (fabs(*val) > MAX)
    {
        if (*val > 0)
            *val = MAX;
        else
            *val = -MAX;
    }
}

void change_limit(float *now, float limit)
{
    static float last = 0;
    float change = *now - last;
    
    *now = last + fmaxf(-limit, fminf(change, limit));
    
    last = *now; 
}

void chassis_moto_world_calc(Chassis_Data_Rx Chassis_Data_Receive,fp32 INS_angle[3])
{

float cos_yaw = cosf(-INS_angle[0]);
float sin_yaw = sinf(-INS_angle[0]);


float vx_body = Chassis_Data_Receive.vx * cos_yaw - Chassis_Data_Receive.vy * sin_yaw;
float vy_body = Chassis_Data_Receive.vx * sin_yaw + Chassis_Data_Receive.vy * cos_yaw;

    val_limit(&vx_body, MAX_VX_SPEED);
    val_limit(&vy_body, MAX_VY_SPEED);
    val_limit(&Chassis_Data_Receive.vw, MAX_VR_SPEED);

    change_limit(&vx_body, ACC_VX);
    change_limit(& vy_body, ACC_VY);
    change_limit(&Chassis_Data_Receive.vw, ACC_VW);



    float common_term = L_plus_W * Chassis_Data_Receive.vw;
    
 
    wheel_rpm[FL] = (vx_body - vy_body - common_term) * inv_radius;
    wheel_rpm[FR] = (vx_body + vy_body + common_term) * inv_radius;
    wheel_rpm[BR] = (vx_body + vy_body - common_term) * inv_radius;
    wheel_rpm[BL] = (vx_body - vy_body + common_term) * inv_radius;
	
    ctrl_motor3(&hcan1,0x01, wheel_rpm[FL]); 
    osDelay(1);
    ctrl_motor3(&hcan1,0x02, wheel_rpm[FR]); 
    osDelay(1);
    ctrl_motor3(&hcan1,0x03, wheel_rpm[BR]); 
    osDelay(1);
    ctrl_motor3(&hcan1,0x04, wheel_rpm[BL]);



}
void chassis_moto_body_calc(Chassis_Data_Rx Chassis_Data_Receive)
{
    // val_limit(&Chassis_Data_Receive.vx, MAX_VX_SPEED);
    // val_limit(&Chassis_Data_Receive.vy, MAX_VY_SPEED);
    // val_limit(&Chassis_Data_Receive.vw, MAX_VR_SPEED);

    // change_limit(&Chassis_Data_Receive.vx, ACC_VX);
    // change_limit(&Chassis_Data_Receive.vy, ACC_VY);
    // change_limit(&Chassis_Data_Receive.vw, ACC_VW);



    float common_term = L_plus_W * Chassis_Data_Receive.vw;
    
 
    // wheel_rpm[FL] = (Chassis_Data_Receive.vx - Chassis_Data_Receive.vy - common_term) * inv_radius;
    // wheel_rpm[FR] = (Chassis_Data_Receive.vx + Chassis_Data_Receive.vy + common_term) * inv_radius;
    // wheel_rpm[BR] = (Chassis_Data_Receive.vx + Chassis_Data_Receive.vy - common_term) * inv_radius;
    // wheel_rpm[BL] = (Chassis_Data_Receive.vx - Chassis_Data_Receive.vy + common_term) * inv_radius;
	 wheel_rpm[FL] = (Chassis_Data_Receive.vy + Chassis_Data_Receive.vx + Chassis_Data_Receive.vw) ;
    wheel_rpm[FR] = (Chassis_Data_Receive.vy - Chassis_Data_Receive.vx + Chassis_Data_Receive.vw) ;
    wheel_rpm[BR] = -(Chassis_Data_Receive.vy - Chassis_Data_Receive.vx - Chassis_Data_Receive.vw) ;
    wheel_rpm[BL] = -(Chassis_Data_Receive.vy + Chassis_Data_Receive.vx - Chassis_Data_Receive.vw) ;

     wheel_current[FL] = apply_current_filter(&filter1,PID_calc(&motor_speed[0], get_DJI_motor_data(CAN_1, chassis_FL).round_speed * 10, -wheel_rpm[FL] / 10));
     wheel_current[FR] = apply_current_filter(&filter1,PID_calc(&motor_speed[1], get_DJI_motor_data(CAN_1, chassis_FR).round_speed * 10,-wheel_rpm[FR] / 10));
    wheel_current[BR] = apply_current_filter(&filter1,PID_calc(&motor_speed[2], get_DJI_motor_data(CAN_1, chassis_BR).round_speed * 10,- wheel_rpm[BR] / 10));
     wheel_current[BL] = apply_current_filter(&filter1,PID_calc(&motor_speed[3], get_DJI_motor_data(CAN_1, chassis_BL).round_speed * 10, -wheel_rpm[BL] / 10));
     
    set_motor_current(wheel_current[FL], CAN_1, 0x201);
    set_motor_current(wheel_current[FR], CAN_1, 0x202);
    set_motor_current(wheel_current[BR], CAN_1, 0x203);
    set_motor_current(wheel_current[BL], CAN_1, 0x204);

    if (remote_power_off == 1)
    {
        set_motor_current(0, CAN_1, 0x201);
        set_motor_current(0, CAN_1, 0x202);
        set_motor_current(0, CAN_1, 0x203);
        set_motor_current(0, CAN_1, 0x204);
    }  
  
}

void Encoder_Update(Encoder2_t *enc, int i, float new_angle) {
    float delta = new_angle - enc->last_angle[i];
    
    if (delta < -3.125f) {
        delta += 6.25f;
        enc->round_count[i]++;
    } else if (delta >= 3.125f) {
        delta -= 6.25f;
        enc->round_count[i]--;
    }
    
    enc->total_angle[i] = enc->round_count[i] * 6.25f + new_angle;
    enc->last_angle[i] = new_angle;
}
void climb_test(RC_ctrl_t *rc_ptr)
{

float result = signed_to_float(rc_ptr->rc.ch[4], -800.0f, 800.0f, 0.0f, 4.0f);

ctrl_motor4(&hcan1, 0x01,  result, 0.5, 0.3);
 osDelay(1);
ctrl_motor4(&hcan1, 0x02,  -result, 0.5, 0.3);             
 osDelay(1);
ctrl_motor4(&hcan1, 0x03,  result, 0.5, 0.3);
 osDelay(1);                
ctrl_motor4(&hcan1, 0x04,  -result, 0.5, 0.3);            
}

void move_text(int x)
{
    move[FL] = apply_current_filter(&filter1,PID_calc(&motor_speed[0], get_DJI_motor_data(CAN_1, chassis_FL).round_speed * 10, -x / 10));
    move[FR] = apply_current_filter(&filter1,PID_calc(&motor_speed[1], get_DJI_motor_data(CAN_1, chassis_FR).round_speed * 10,-x / 10));
    move[BR] = apply_current_filter(&filter1,PID_calc(&motor_speed[2], get_DJI_motor_data(CAN_1, chassis_BR).round_speed * 10, x / 10));   
    move[BL] = apply_current_filter(&filter1,PID_calc(&motor_speed[3], get_DJI_motor_data(CAN_1, chassis_BL).round_speed * 10, x / 10));
     
    set_motor_current(move[FL], CAN_1, 0x201);
    set_motor_current(move[FR], CAN_1, 0x202);
    set_motor_current(move[BR], CAN_1, 0x203);
    set_motor_current(move[BL], CAN_1, 0x204);
           
}
void filter1_init(CurrentFilter* filter, float alpha) {
    filter->prev_output = 0.0f;
    filter->alpha = alpha;
}

float apply_current_filter(CurrentFilter* filter, float current_input) {
    float filtered = filter->alpha * current_input + 
                   (1.0f - filter->alpha) * filter->prev_output;
    filter->prev_output = filtered;
    return filtered;
}
float signed_to_float(int x_signed, float input_min, float input_max, float output_min, float output_max)
{
    // 计算输入范围和输出范围
    float input_span = input_max - input_min;
    float output_span = output_max - output_min;
    
    // 将输入值归一化到0-1范围
    float normalized = (float)(x_signed - input_min) / input_span;
    
    // 映射到输出范围
    return normalized * output_span + output_min;
}



