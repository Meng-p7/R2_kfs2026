#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "DM_motor_drive.h"

//气泵 = PF0
#define PUMP_ON()     HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET)
#define PUMP_OFF()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET)

//电磁阀1 = PF1
#define VALVE1_ON()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET)
#define VALVE1_OFF()  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET)

//电磁阀2 = PB12
#define VALVE2_ON()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define VALVE2_OFF()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

//状态枚举
typedef enum {
    STATE_INIT = 0,
    STATE_TASK1,
    STATE_TASK2,
    STATE_TASK3,
    STATE_TASK4,
    STATE_SLEEP,
} system_state_t;

//共享变量（在arm_task.c中定义，test_task.c中通过extern访问）
extern system_state_t current_state;
extern system_state_t prev_state;
extern uint8_t task1_phase;
extern uint8_t task2_phase;
extern TickType_t entertime;
extern TickType_t phase_entertime;
extern float pickup_height;
extern uint8_t store_count;
extern float store_pos;

//动作函数声明
void state_task0(void);
void state_task1(void);
void state_task2(void);
void state_task3(void);
void state_task4(void);
void state_task_sleep(void);
void save_zero(void);

#endif
