/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "DBUS_remote_control.h"
#include "chassis_move.h"
#include "DM_motor_drive.h"
/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
Chassis_Data_Rx Chassis_Data_Receive;
DM_motor_data_t DM_Receive[4];
int flag=0;
extern INS_Data_t INS_angle[3];
typedef enum {
    STATE_IDLE = 0,
    STATE_MOVE_LEGS,
    STATE_ADJUST_BODY,
    STATE_STOP_MOTORS
} system_state_t;

// 全局变量
static system_state_t current_state = STATE_IDLE;
static uint32_t state_entry_time = 0;
void test_task(void const * argument)
{  
 chassis_move_init();

     for(;;)
{ const RC_ctrl_t *rc_ptr = get_remote_control_point();
  for(int i=0;i<4;i++) {DM_Receive[i]=get_DM_data(i);}
switch(current_state) {
        case STATE_IDLE:
            state_idle();
            break;
            
        case STATE_MOVE_LEGS:
            state_move_legs();
            break;
            
        case STATE_ADJUST_BODY:
            state_adjust_body();
            break;
            
        case STATE_STOP_MOTORS:
            state_stop_motors();
            break;
            
        default:
            current_state = STATE_IDLE;
            break;
    }
  osDelay(10);

    }
}

static void state_idle(void)
{
    if(rc_ptr->rc.ch[6] == 0 && 
       DM_Receive[0].toq >= 2 && 
       DM_Receive[1].toq >= 2) {
        
        // 执行腿部电机控制
        ctrl_motor1(&hcan1, 0x01, -1.5, 0.5, 3, 0.3, 2);
        osDelay(1);
        ctrl_motor1(&hcan1, 0x02, 1.5, 0.5, 3, 0.3, 2);
        osDelay(1);
        ctrl_motor1(&hcan1, 0x03, -3, 0.5, 3, 0.3, 2);
        osDelay(1);
        ctrl_motor1(&hcan1, 0x04, 3, 0.5, 3, 0.3, 2);
        
        current_state = STATE_MOVE_LEGS;
        state_entry_time = osKernelGetTickCount();
    }
}

// 移动腿部状态
static void state_move_legs(void)
{
    if(INS_angle(1) == 0) {
        // 执行身体调整电机控制
        ctrl_motor3(&hcan2, 0x01, 3);
        osDelay(1);
        ctrl_motor3(&hcan2, 0x02, -3);
        osDelay(1);
        ctrl_motor3(&hcan2, 0x03, -3);
        osDelay(1);
        ctrl_motor3(&hcan2, 0x04, 3);
        
        current_state = STATE_ADJUST_BODY;
        state_entry_time = osKernelGetTickCount();
    }
    
    // 可选：添加超时保护
    // if(osKernelGetTickCount() - state_entry_time > 5000) {
    //     current_state = STATE_IDLE; // 超时返回空闲状态
    // }
}

// 调整身体状态
static void state_adjust_body(void)
{
    if(DM_Receive[3].toq >= 2 && DM_Receive[2].toq >= 2) {
        // 停止所有电机
        ctrl_motor3(&hcan2, 0x01, 0);
        osDelay(1);
        ctrl_motor3(&hcan2, 0x02, 0);
        osDelay(1);
        ctrl_motor3(&hcan2, 0x03, 0);
        osDelay(1);
        ctrl_motor3(&hcan2, 0x04, 0);
        osDelay(10);
        
        // 执行最终电机控制
        ctrl_motor2(&hcan1, 0x01, -1, 0.2);
        osDelay(10);
        ctrl_motor2(&hcan1, 0x02, 1, 0.2);
        osDelay(10);
        ctrl_motor2(&hcan1, 0x03, -1, 0.2);
        osDelay(10);
        ctrl_motor2(&hcan1, 0x04, 1, 0.2);
        osDelay(10);
        
        current_state = STATE_IDLE;
    }
    
    // 可选：添加超时保护
    // if(osKernelGetTickCount() - state_entry_time > 5000) {
    //     current_state = STATE_STOP_MOTORS; // 超时进入停止状态
    // }
}

// 停止电机状态（安全状态）
static void state_stop_motors(void)
{
    // 停止所有电机
    for(int i = 1; i <= 4; i++) {
        ctrl_motor3(&hcan2, i, 0);
        osDelay(1);
    }
    
    current_state = STATE_IDLE;
}

// 外部状态重置函数
void system_reset(void)
{
    current_state = STATE_IDLE;
}

// 获取当前状态（用于调试）
system_state_t get_current_state(void)
{
    return current_state;
}

