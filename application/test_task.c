#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "DBUS_remote_control.h"
#include "DM_motor_drive.h"
#include "DJI_motor_drive.h"

extern volatile DM_motor_data_t DM_pitch_motor_data[5];
//气泵 = PF0
#define PUMP_ON()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET)
#define PUMP_OFF()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET)

// 电磁阀1 = PF1
#define VALVE1_ON()  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET)
#define VALVE1_OFF()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET)

// 电磁阀2 = PB12	
#define VALVE2_ON()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define VALVE2_OFF()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

/*
REDME!
//task1用于取方块，task2用于放方块，task3，4用于夹武器
对于方块任务，会接收到算法的三种信号
目前写好的task1、2为取较低的方块，而且存方块并没有存到最里面
后续工作是完善task1,取较高的两个方块以及把存、放的两种情况写完
对于取方块可以用三个数值的拨杆代表不同高度的方块，只需要修改取方块的高度即可
对于存方块，可以定义一个i用来记录已经存放的方块数量，i=0时放里面，i=1时放外面
对于放方块同理，判断应该取外面还是里面

*/

typedef enum {
    STATE_INIT = 0,
    STATE_TASK1,
    STATE_TASK2,
	STATE_TASK3, 
    STATE_TASK4,

} system_state_t ;

static system_state_t prev_state = STATE_INIT;
static system_state_t current_state = STATE_INIT;	
static uint8_t task1_phase = 0;	//用于局部判断task1所处的阶段
static uint8_t task2_phase = 0;
static TickType_t entertime = 0;
static TickType_t phase_entertime = 0;
void test_task(void const * argument)
{  
	save_zero();
//	save_pos_zero(&hcan2, 0x05, 0);  
//    osDelay(300); 
	damiao_init();	
	
    for(;;)
    { 	
		const RC_ctrl_t *rc_ptr = get_remote_control_point();
        
		 // 每次进入新的状态都重新记录时间
		if (current_state != prev_state) {
            entertime = xTaskGetTickCount();
            prev_state = current_state;          
         }
      
		 if (rc_ptr->rc.ch[10] != 0) {
            current_state = STATE_TASK3;
		  } 
		 
        switch(current_state) {
			case STATE_INIT:
				state_task0();
			if (rc_ptr->rc.ch[9] == 1 ){ 
			current_state = STATE_TASK1;
			}			
                break;
			
            case STATE_TASK1:
                state_task1();
			if (rc_ptr->rc.ch[9] == 0){
            current_state = STATE_TASK2;
          
			}			
                break;
                
            case STATE_TASK2:
                state_task2();
			if ( rc_ptr->rc.ch[9] == 1){
            current_state = STATE_TASK1;
			}
                break;
					                
            case STATE_TASK3:
                state_task3();
			if (rc_ptr->rc.ch[10] == 0)
            {
           current_state = STATE_TASK4;
            }
                break;
		
		 case STATE_TASK4:
                state_task4();		
            if (rc_ptr->rc.ch[9] == 1 )	
            {  
            current_state = STATE_TASK1;
            }
                break;
				
         } 		  
    }
}


//前1+，2-
//升3+，4—
//往前转5+

static void state_task1(void)//phase0:吸方块;phase1:升高;phase2:存放;phase3:存方块;phase4:完成
{ 		
    static uint8_t prev_phase = 0xFF;
    TickType_t xCurrentTime2 = xTaskGetTickCount();
    float vel1 = DM_pitch_motor_data[0].vel;
    float vel3 = DM_pitch_motor_data[2].vel;
    if (task1_phase != prev_phase) {
        phase_entertime = xCurrentTime2;
        prev_phase = task1_phase;
    }//每次进入新phase都重新记录时间

    if (task1_phase == 0) {
    ctrl_motor2(&hcan2, 0x01, 600, 3);  
    osDelay(1);     
    ctrl_motor2(&hcan2, 0x02, -600, 3);
    osDelay(1);          
    ctrl_motor2(&hcan2, 0x05, 0, 1.5);
	osDelay(1);
	
	if ((xCurrentTime2-entertime)>pdMS_TO_TICKS(2000)&&fabsf(vel1) < 0.05f )
	{  
	ctrl_motor2(&hcan2, 0x03, 75, 2);  
    osDelay(1); 
    ctrl_motor2(&hcan2, 0x04, -75, 2);
	osDelay(1);  	 
	PUMP_ON();
 
	if ((xCurrentTime2 - phase_entertime) > pdMS_TO_TICKS(12000) && fabsf(vel3) < 0.05f ) {//确保吸住方块
        task1_phase = 1;               
        }
        }
	else{
	ctrl_motor2(&hcan2, 0x03, 115, 2);  
    osDelay(1);     
    ctrl_motor2(&hcan2, 0x04, -115, 2);
    osDelay(1); 
	}
}
   
   else if (task1_phase == 1)  {  
       PUMP_ON(); 
       ctrl_motor2(&hcan2, 0x03, 115, 2);
       osDelay(1);
       ctrl_motor2(&hcan2, 0x04, -115, 2);
       osDelay(1);   
    if ((xCurrentTime2 - phase_entertime) > pdMS_TO_TICKS(2000) && fabsf(vel3) < 0.05f) {
        task1_phase = 2;
        }
   } 
    else if (task1_phase == 2 ) {      
            ctrl_motor2(&hcan2, 0x01, 300, 0.5);
            osDelay(1);
            ctrl_motor2(&hcan2, 0x02, -300, 0.5);
            osDelay(1);
            ctrl_motor2(&hcan2, 0x05, 4.52, 1);
            osDelay(1);   
            if ((xCurrentTime2 - phase_entertime) > pdMS_TO_TICKS(2000) && fabsf(vel1) < 0.05f) {
                PUMP_OFF();
                task1_phase = 3;
            }                           
     }
    else if (task1_phase == 3) {      
        if ((xCurrentTime2 - phase_entertime) > pdMS_TO_TICKS(2000) && fabsf(vel1) < 0.05f) {
		ctrl_motor2(&hcan2, 0x03, 0, 2);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x04, 0, 2);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x05, 0, 1);
        osDelay(1);
		if ((xCurrentTime2 - phase_entertime) > pdMS_TO_TICKS(5000) && fabsf(vel3) < 0.05f) {
              task1_phase = 4;
            } 	
        }	
		else{
		ctrl_motor2(&hcan2, 0x01, 500, 2);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x02, -500, 2);
        osDelay(1);	
			}
        }
	else if (task1_phase == 4) { 
		ctrl_motor2(&hcan2, 0x01, 0, 2);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x02, 0, 2);
        osDelay(1);
	 }
		
    }


static void state_task2(void) //取方块，拆分动作：机械臂先伸到位置，往里顶吸住，往外伸放方块，归位
{      
    static uint8_t prev_phase = 0xFF;
    TickType_t xCurrentTime2 = xTaskGetTickCount();
    float vel1 = DM_pitch_motor_data[0].vel;
    float vel3 = DM_pitch_motor_data[2].vel;

    if (task2_phase != prev_phase) {
        phase_entertime = xCurrentTime2;
        prev_phase = task2_phase;
    }//每次进入新phase都重新记录时间

        if (task2_phase == 0) {
        ctrl_motor2(&hcan2, 0x01, 400, 3);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x02, -400, 3);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x03, 115, 2);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x04, -115, 2);
        osDelay(1);
         if ((xCurrentTime2 - phase_entertime) > pdMS_TO_TICKS(2000) && fabsf(vel1) < 0.05f && fabsf(vel3) < 0.05f){ 
        task2_phase = 1;
         }
    }
     else if (task2_phase == 1) {      
        ctrl_motor2(&hcan2, 0x01, 290, 1);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x02, -290, 1);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x05, -1.56, 2);
        osDelay(1);
        PUMP_ON();
        if ((xCurrentTime2 - phase_entertime) > pdMS_TO_TICKS(6000) &&fabsf(vel1) < 0.05f )
        {
            task2_phase = 2;
        }   
    }
    else if (task2_phase == 2) {           
        		
		ctrl_motor2(&hcan2, 0x01, 600, 3);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x02, -600, 3);
        osDelay(1);
        if ((xCurrentTime2 - phase_entertime) > pdMS_TO_TICKS(2000) && fabsf(vel1) < 0.05f){           
          ctrl_motor2(&hcan2, 0x05, -4.5, 1);
          osDelay(1);   				
		  task2_phase = 3;
             }
    }

     else if (task2_phase == 3) {
        if ((xCurrentTime2 - phase_entertime) > pdMS_TO_TICKS(8000)) {
        PUMP_OFF();
		ctrl_motor2(&hcan2, 0x01, 0, 3);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x02, 0, 3);
        osDelay(1);
		ctrl_motor2(&hcan2, 0x03, 0, 2);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x04, 0, 2);
        osDelay(1);
		ctrl_motor2(&hcan2, 0x05, 0, 2);
        osDelay(1);
        
        }
    }

    
}


/*气缸逻辑：
VALVE1_ON()+VALVE2_OFF() =气缸闭合
VALVE1_OFF()+VALVE2_ON() =气缸张开

*/
static void state_task3(void) 
{        
    VALVE1_OFF(); 		 
    osDelay(10);
	VALVE2_ON();//张开	
	
	ctrl_motor2(&hcan2, 0x03, 125, 2);
    osDelay(1);     
    ctrl_motor2(&hcan2, 0x04, -125, 2);
    osDelay(1); 
	ctrl_motor2(&hcan2, 0x05, -1.56, 1);
    osDelay(1);
	
}

static void state_task4(void) //气缸闭合
{        
    VALVE2_OFF();
	osDelay(10);
	VALVE1_ON(); 		    
	
	TickType_t xCurrentTime2 = xTaskGetTickCount();
    if((xCurrentTime2-entertime)>pdMS_TO_TICKS(3000))
	{
	ctrl_motor2(&hcan2, 0x03, 220, 2);
    osDelay(1);     
    ctrl_motor2(&hcan2, 0x04, -220, 2);
    osDelay(1); 		
	}
	
}

static void state_task0(void) //初始化：将所有电机设置为0位置
{
		ctrl_motor2(&hcan2, 0x01, 0, 1);
        osDelay(1);
        ctrl_motor2(&hcan2, 0x02, 0, 1);
        osDelay(1);
		ctrl_motor2(&hcan2, 0x05, 0, 1);
        osDelay(1);
		static float last_toq3 = 0.0f; 
	    static float last_toq4 = 0.0f;
		static uint8_t stop = 0;        // 停止标志（1个，最少）
	
        float toq3 = DM_pitch_motor_data[2].toq;
		float toq4 = DM_pitch_motor_data[3].toq;
        float diff3 = fabs(toq3 - last_toq3);
		float diff4 = fabs(toq4 - last_toq4);
        if(stop == 0)
        {
            ctrl_motor2(&hcan2, 0x03, -80, 1);
            ctrl_motor2(&hcan2, 0x04, 80, 1);
            osDelay(1);
            if(last_toq3 == 0 && last_toq4 == 0.0f)
            {
                last_toq3 = toq3;
				 last_toq4 = toq4;
               
            }
            else if(diff3 > 10.0f ||diff4 > 12.0f)
            {
                save_pos_zero(&hcan2, 0x03, 0x100);
                osDelay(10);
                save_pos_zero(&hcan2, 0x04, 0x100); 
                osDelay(10);
                ctrl_motor2(&hcan2, 0x03, 0, 1);
                ctrl_motor2(&hcan2, 0x04, 0, 1);
                stop = 1; // 锁死停止
            }
        }
        else
        {
            ctrl_motor2(&hcan2, 0x03, 0, 1);
            osDelay(1);
            ctrl_motor2(&hcan2, 0x04, 0, 1);
            osDelay(1);
        }
	
}

static void save_zero(void)
{  		
	save_pos_zero(&hcan2, 0x01, 0x100);  
    osDelay(100);     
    save_pos_zero(&hcan2, 0x02, 0x100);  
    osDelay(100);  
	save_pos_zero(&hcan2, 0x03, 0x100);  
    osDelay(100);  
	save_pos_zero(&hcan2, 0x04, 0x100);  
    osDelay(100); 	
}

