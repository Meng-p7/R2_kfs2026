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


float result=1;
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
CurrentFilter filter2;
CurrentFilter filter3;
CurrentFilter filter4;

void chassis_move_init()
{  
    PID_init(&motor_speed[0], PID_POSITION, PID_SPEED, 15000, 5000);
    PID_init(&motor_speed[1], PID_POSITION,PID_SPEED, 15000, 5000);
    PID_init(&motor_speed[2], PID_POSITION, PID_SPEED, 15000, 5000);
    PID_init(&motor_speed[3], PID_POSITION, PID_SPEED, 15000, 5000);
    damiao_init();	
	filter1_init(&filter1, 0.6f);
	filter1_init(&filter2, 0.6f);
	filter1_init(&filter3, 0.6f);
	filter1_init(&filter4, 0.6f);
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

 result = signed_to_float(rc_ptr->rc.ch[4], -800.0f, 800.0f, 0.0f, 6.0f);

ctrl_motor4(&hcan2, 0x01,  result, 0.3, 0.3);
 osDelay(1);
ctrl_motor4(&hcan2, 0x02,  -result, 0.3, 0.3);             
 osDelay(1);
ctrl_motor4(&hcan2, 0x03,  result, 0.3, 0.3);
 osDelay(1);                
ctrl_motor4(&hcan2, 0x04,  -result, 0.3, 0.3);            
}

void move_text(int x)
{
    move[FL] = apply_current_filter(&filter1,PID_calc(&motor_speed[0], get_DJI_motor_data(CAN_1, chassis_FL).round_speed * 10, -x / 10));
    move[FR] = apply_current_filter(&filter2,PID_calc(&motor_speed[1], get_DJI_motor_data(CAN_1, chassis_FR).round_speed * 10,-x / 10));
    move[BR] = apply_current_filter(&filter3,PID_calc(&motor_speed[2], get_DJI_motor_data(CAN_1, chassis_BR).round_speed * 10, x / 10));   
    move[BL] = apply_current_filter(&filter4,PID_calc(&motor_speed[3], get_DJI_motor_data(CAN_1, chassis_BL).round_speed * 10, x / 10));
     
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



