#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "DBUS_remote_control.h"
#include "chassis_move.h"
#include "DM_motor_drive.h"
#include "DJI_motor_drive.h"




Chassis_Data_Rx Chassis_Data_Receive;
RC_ctrl_t *rc_ptr;
DM_motor_data_t DM_Receive[4];
Coordinates_t body_coordinates,arm_start_coordinates[4],arm_end_coordinates[4],d[4];
extern DM_motor_data_t DM_pitch_motor_data[4];
uint8_t motor_step_climb = 0;
uint8_t motor_step_stay = 0;
uint8_t motor_step_move = 0;
extern fp32 INS_angle[3];
float cos_angle[4];
typedef enum {
    STATE_STAY = 0,
    STATE_CLIMB,
    STATE_MOVE,
} system_state_t ;
// 全局变量
 extern RC_ctrl_t RC_data;
static system_state_t current_state = STATE_STAY;
static uint32_t state_entry_time = 0;
int flag=0 ; 
TickType_t xLastWakeTime2 = 0;
TickType_t xCurrentTime2 = 0;    
const TickType_t xPeriod1 = pdMS_TO_TICKS(3); 


void test_task(void const * argument)
{  
 chassis_move_init();

     for(;;)
{ const RC_ctrl_t *rc_ptr = get_remote_control_point();
        
rc_deadband_limit(process_throttle(rc_ptr->rc.ch[1]*0.5), Chassis_Data_Receive.vx, CHASSIS_RC_DEADLINE);
rc_deadband_limit(process_throttle(rc_ptr->rc.ch[2]*0.5), Chassis_Data_Receive.vy, CHASSIS_RC_DEADLINE);
rc_deadband_limit(process_throttle(-rc_ptr->rc.ch[3]*0.5), Chassis_Data_Receive.vw, CHASSIS_RC_DEADLINE2);
//           for(int i=0;i<4;i++)
//    {   if(DM_pitch_motor_data[i].state==0)
//        { enable_DM(&hcan2,0x01+i,4);
//         osDelay(3);
//        }
//     }


//  Euler_angles(&body_coordinates,INS_angle);
//   for(int i = 0;i<4;i++) {
//   Euler_angles( &arm_start_coordinates[i],INS_angle);
//   Euler_angles( &arm_end_coordinates[i],INS_angle);
//   }
//  Stress_cone_anologize(&body_coordinates,
//                        arm_start_coordinates,
//                        arm_end_coordinates);
switch(current_state) {
        case STATE_STAY:
            state_stay();
            break;
            
        case STATE_CLIMB:
            state_climb();
            break;
            
        case STATE_MOVE:
            state_move();
            break;
    }


 }
}


static void state_stay(void)
{ 
 TickType_t xCurrentTime2 = xTaskGetTickCount();
  if((xCurrentTime2 - xLastWakeTime2) >= xPeriod1)
        {
          chassis_moto_body_calc(Chassis_Data_Receive);
          CAN1_send_current();
            xLastWakeTime2 = xCurrentTime2;
         }


            ctrl_motor4(&hcan2, 0x01, 4, 0.5, 0.3);
            osDelay(1);     
            ctrl_motor4(&hcan2, 0x02, -2, 0.5, 0.3);
            osDelay(1);     
            ctrl_motor4(&hcan2, 0x03, 2, 0.5, 0.3);
            osDelay(1);     
            ctrl_motor4(&hcan2, 0x04, -4, 0.5, 0.3);
            osDelay(1);  
              
    if( motor_step(DM_pitch_motor_data,1,1.0f) && motor_step2(DM_pitch_motor_data,0,2,4)) {
        current_state = STATE_CLIMB;
        state_entry_time = osKernelSysTick();
    }
}

// 攀爬状态 - 修正版本
static void state_climb(void)
{
    // 先检查超时
    // if(osKernelSysTick() - state_entry_time > 5000) {
    //     current_state = STATE_STAY;
    //     motor_step_stay = 0;
    //     return;
    // }
         TickType_t xCurrentTime2 = xTaskGetTickCount();
  if((xCurrentTime2 - xLastWakeTime2) >= xPeriod1)
        {
        move_text(10);
        CAN1_send_current();
            xLastWakeTime2 = xCurrentTime2;
        }
            ctrl_motor4(&hcan2, 0x01, 6.7, 0.5, 0.3);
            osDelay(1);     
            ctrl_motor4(&hcan2, 0x02, -4, 0.5, 0.3);
            osDelay(1);     
            ctrl_motor4(&hcan2, 0x03, 4, 0.5, 0.3);
            osDelay(1);     
            ctrl_motor4(&hcan2, 0x04, -6.7, 0.5, 0.3);
            osDelay(1);     
            if(INS_angle_reset(&INS_angle[1])&&motor_step2(DM_pitch_motor_data,0,4,6.7)) {
                current_state = STATE_MOVE;
                state_entry_time = osKernelSysTick();
            }
           
    
}

// 移动状态 - 修正版本
static void state_move(void)
{

    // if(osKernelSysTick() - state_entry_time > 5000) {
    //     current_state = STATE_STAY;
    //     motor_step_stay = 0;
    //     return;
    // }
     TickType_t xCurrentTime2 = xTaskGetTickCount();
  if((xCurrentTime2 - xLastWakeTime2) >= xPeriod1)
        {
        move_text(100);
        CAN1_send_current();
            xLastWakeTime2 = xCurrentTime2;
        }

         
            ctrl_motor4(&hcan2, 0x01, 6.7, 0.5, 0.3);
            osDelay(1);     
            ctrl_motor4(&hcan2, 0x02, -4, 0.5, 0.3);  //前
            osDelay(1);     
            ctrl_motor4(&hcan2, 0x03, 4, 0.5, 0.3); //前
            osDelay(1);     
            ctrl_motor4(&hcan2, 0x04, -6.7, 0.5, 0.3);
            osDelay(1); 
            if(motor_step3(DM_pitch_motor_data,0,3.8f) ) {
                current_state = STATE_STAY; 
                state_entry_time = osKernelSysTick();
              
            }
          
    
}


int INS_angle_reset(fp32* INS_angle)  
{
    if (fabs(INS_angle[1]) < 0.2f)  
        return 1;
    else
        return 0;
}

int motor_step(DM_motor_data_t *DM_Receive,int i,float set)  
{
    if (fabs(DM_Receive[i].toq) > set &&
        fabs(DM_Receive[i+1].toq) > set)  
        return 1;
 
    else return 0;
}
int motor_step2(DM_motor_data_t *DM_Receive,int i,float set1,float set2) 
{
    if((fabs(DM_Receive[i].pos) - set2&&
    fabs(fabs(DM_Receive[i+1].pos) - set1) < 0.01f &&
    fabs(fabs(DM_Receive[i+2].pos) - set1) < 0.01f &&
    fabs(fabs(DM_Receive[i+3].pos) - set2) < 0.01f))  
        return 1;
    else
        return 0;
}

int motor_step3(DM_motor_data_t *DM_Receive,int i,float set)  
{
    if (
        fabs(DM_Receive[i+3].toq) > set)  
        return 1;
 
    else return 0;
}
// 定义向量运算结构体和函数
typedef struct {
    float x, y, z;
} Vector3D;

// 向量减法
static inline Vector3D vector_sub(const Coordinates_t* a, const Coordinates_t* b) {
    return (Vector3D){a->x - b->x, a->y - b->y, a->z - b->z};
}

// 向量点积
static inline float vector_dot(const Vector3D* a, const Vector3D* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

// 向量模的平方
static inline float vector_sqr_norm(const Vector3D* v) {
    return v->x * v->x + v->y * v->y + v->z * v->z;
}

// 向量模
static inline float vector_norm(const Vector3D* v) {
    return sqrtf(vector_sqr_norm(v));
}

// 计算垂足和垂线向量
static void calculate_perpendicular(const Coordinates_t* line_start, 
                                   const Coordinates_t* line_end,
                                   const Coordinates_t* point,
                                   Vector3D* perpendicular_vec,
                                   float* cos_angle) {
    Vector3D line_vec = vector_sub(line_end, line_start);
    Vector3D point_vec = vector_sub(point, line_start);
    
    float t = vector_dot(&line_vec, &point_vec);
    float norm_line_sqr = vector_sqr_norm(&line_vec);
    
    // 避免除零
    if (norm_line_sqr < 1e-6f) {
        perpendicular_vec->x = perpendicular_vec->y = perpendicular_vec->z = 0.0f;
        *cos_angle = 0.0f;
        return;
    }
    
    float theta = t / norm_line_sqr;
    
    // 计算垂足坐标
    Coordinates_t foot = {
        .x = line_start->x + line_vec.x * theta,
        .y = line_start->y + line_vec.y * theta,
        .z = line_start->z + line_vec.z * theta
    };
    
    // 计算垂线向量
    *perpendicular_vec = vector_sub(&foot, point);
    
    // 计算与z轴夹角的余弦值
    float norm_perp = vector_norm(perpendicular_vec);
    *cos_angle = (norm_perp > 1e-6f) ? perpendicular_vec->z / norm_perp : 0.0f;
}

void Under_force_anologize(Coordinates_t* body_coordinates,
                          Coordinates_t* arm_start_coordinates,
                          Coordinates_t* arm_end_coordinates) {
    for (int i = 0; i < 4; i++) {
        if (DM_pitch_motor_data[i].pos < 2) {
            arm_start_coordinates[i].z -= 0.01f;
            flag = 1;
        } else {
            arm_end_coordinates[i].z -= 0.01f;
            flag = 0;
        }
    }
}

void Stress_cone_anologize(Coordinates_t* body_coordinates,
                          Coordinates_t* arm_start_coordinates,
                          Coordinates_t* arm_end_coordinates) {
    const Coordinates_t* arm_coords = (flag == 1) ? arm_start_coordinates : arm_end_coordinates;
    
    for (int i = 0; i < 4; i++) {
        Vector3D perpendicular_vec;
        
        
        calculate_perpendicular(&arm_coords[i], &arm_coords[i + 1], 
                              &body_coordinates[i], &perpendicular_vec, &cos_angle[i]);
        
        // 存储结果（根据原代码逻辑，可能需要保存到全局变量）
        d[i].x = perpendicular_vec.x;
        d[i].y = perpendicular_vec.y;
        d[i].z = perpendicular_vec.z;
        
    }
}
void Euler_angles(Coordinates_t* a,fp32 *INS_angle) {
    
    // 计算三角函数值（提前计算提高效率）
    float cr = cos(INS_angle[2]); float sr = sin(INS_angle[2]);
    float cp = cos(INS_angle[1]); float sp = sin(INS_angle[1]);
    float cy = cos(INS_angle[0]); float sy = sin(INS_angle[0]);
    
    // 组合旋转矩阵
    a->x = a->x * (cy*cp) + a->y * (cy*sp*sr - sy*cr) + a->z * (cy*sp*cr + sy*sr);
    a->y = a->x * (sy*cp) + a->y * (sy*sp*sr + cy*cr) + a->z * (sy*sp*cr - cy*sr);
    a->z = a->x * (-sp)   + a->y * (cp*sr)            + a->z * (cp*cr);
}