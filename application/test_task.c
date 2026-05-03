#include "arm_task.h"
#include "test_task.h"
#include "DBUS_remote_control.h"
#include "DJI_motor_drive.h"

extern volatile DM_motor_data_t DM_pitch_motor_data[5];
/*
遥控器的ch6,7,8,9是我随便写的，测试时需修改，确保这几个通道只有0/1的值
当ch6=0,为吸取状态，ch7/8/9分别代表三种高度的方块，通道值=1吸取，=0进入休眠状态，可以重复按下，最多可吸取三个方块
当ch6=1,为放置状态，ch7/8/9分别代表三种高度的九宫格，通道值=1存储，=0进入休眠状态，可以重复按下，最多可放置三个方块
当ch10=1,强制进入夹武器，ch10=0,松开
操作逻辑：先检查ch6，吸取是为0，放置是为1，然后根据需要拨下ch7/8/9
首要测试任务为测试TASK1：1.测试三种方块高度，修改 pickup_height；2.修改arm_task.c中109、117行的高度，确保高于方块高度；
3.修改arm_task.c中130行的store_pos,表示存放的位置；4.测试电机转速，确保不会撞到

*/
void test_task(void const * argument)
{
	save_zero();
	damiao_init();

    for(;;)
    {
		const RC_ctrl_t *rc_ptr = get_remote_control_point();

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
			if (rc_ptr->rc.ch[9] != 0 || rc_ptr->rc.ch[8] != 0 || rc_ptr->rc.ch[7] != 0){
				if (rc_ptr->rc.ch[7] != 0) pickup_height = 115;
				else if (rc_ptr->rc.ch[8] != 0) pickup_height = 95;
				else pickup_height = 75;
				task1_phase = 0;
				current_state = STATE_TASK1;
			}
                break;

            case STATE_TASK1:
                state_task1();
			if (task1_phase == 4) {
				if (rc_ptr->rc.ch[9] == 0 && rc_ptr->rc.ch[8] == 0 && rc_ptr->rc.ch[7] == 0) {
					current_state = STATE_SLEEP;
				}
			}
			else if (task1_phase == 1 && store_count >= 3) {
				if (rc_ptr->rc.ch[9] == 0 && rc_ptr->rc.ch[8] == 0 && rc_ptr->rc.ch[7] == 0) {
					current_state = STATE_SLEEP;
				}
			}
                break;

            case STATE_TASK2:
                state_task2();
			if (task2_phase == 4) {
				if (rc_ptr->rc.ch[9] == 0 && rc_ptr->rc.ch[8] == 0 && rc_ptr->rc.ch[7] == 0) {
					store_count--;
					current_state = STATE_SLEEP;
				}
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
            if (rc_ptr->rc.ch[9] != 0 || rc_ptr->rc.ch[8] != 0 || rc_ptr->rc.ch[7] != 0)
            {
				if (rc_ptr->rc.ch[7] != 0) pickup_height = 115;
				else if (rc_ptr->rc.ch[8] != 0) pickup_height = 95;
				else pickup_height = 75;
				task1_phase = 0;
				current_state = STATE_TASK1;
            }
                break;

         case STATE_SLEEP:
                state_task_sleep();
			if (rc_ptr->rc.ch[6] != 0 && (rc_ptr->rc.ch[9] != 0 || rc_ptr->rc.ch[8] != 0 || rc_ptr->rc.ch[7] != 0)) {
				if (rc_ptr->rc.ch[7] != 0) pickup_height = 115;
				else if (rc_ptr->rc.ch[8] != 0) pickup_height = 145;
				else pickup_height = 175;
				if (store_count == 3) task2_phase = 2;
				else task2_phase = 0;
				current_state = STATE_TASK2;
			}
                break;

         }
    }
}
