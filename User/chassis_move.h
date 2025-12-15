#include "struct_typedef.h"
#include "user_lib.h"
#include "DBUS_remote_control.h"
#define __chassis_move_H__
#ifdef  __chassis_move_H__

struct chassis_status
{
	struct
	{
		float vx,vy,vw;
		float last_vx,last_vy,last_vw;
	}speed;
	struct
	{
		float ax,ay,aw;
	}acc;
};
/*******************************一节低通滤波参数************************/
#define CHASSIS_CONTROL_TIME 0.012f   //x和y本次信任参数
#define CHASSIS_CONTROL_TIME_Z 0.005f  //z本次信任参数
#define CHASSIS_CONTROL_TIME_3508 0.1f   //3508本次信任参数

//信任上一次参数占比
#define CHASSIS_ACCEL_X_NUM 0.98f
#define CHASSIS_ACCEL_Y_NUM 0.98f
#define CHASSIS_ACCEL_Z_NUM 0.98f
#define CHASSIS_ACCEL_3508_NUM 0.90f


typedef struct {
    float total_angle[8];
    int32_t round_count[8];
    float last_angle[8];
} Encoder2_t;

extern Encoder2_t encoder2 ;


typedef struct {
    float prev_output;
    float alpha;
} CurrentFilter;

extern struct chassis_status chassis;
/*******************************????*******************************/
#define R       MOTOR_DISTANCE_TO_CENTER
#define PI       3.1415926f

//???????????
#define HALF_ECD_RANGE  3.125
#define ECD_RANGE       6.25

//????? m 
#define MOTOR_DISTANCE_TO_CENTER 0.31823f

//?????
#define SMALL_TOP_CIRCUMFERENCE	 	MOTOR_DISTANCE_TO_CENTER*2*3.1415926f			//1.480440609656214f

//????  m
#define WHEEL_HALF_SIZE 	0.045f

//????	m
#define WHEEL_CIRCUMFERENCE				WHEEL_HALF_SIZE*2*3.1415926f           //WHEEL_HALF_SIZE*2*3.1415926f  	

//??????  1.84175866175m/s   --8911
//减速比13.733，rpm: 圈/min
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例	
#define CHASSIS_VX_RC_SEN            19.0f/60.0f*WHEEL_CIRCUMFERENCE

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 					 19.0f/60.0f*WHEEL_CIRCUMFERENCE

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 					 19.0f/60.0f*WHEEL_CIRCUMFERENCE / SMALL_TOP_CIRCUMFERENCE/2.0f/PI			

//电机 ID
#define FL 0
#define FR 1
#define BR 2
#define BL 3


#define MAX_VX_SPEED 1000 			 
#define MAX_VY_SPEED 1000
#define MAX_VR_SPEED 1000



#define ACC_VX       10       
#define ACC_VY       10   
#define ACC_VW       10
#define HALF_ECD_RANGE  3.125
#define ECD_RANGE       6.25
//摇杆死区

#define GIM_Y_ECD_1 2.532f
#define GIM_Y_ECD_2 1.993f
#define GIM_Y_ECD_3 0.34f
#define GIM_Y_ECD_4 0.43f
void chassis_move_init(void);//底盘移动初始化
 void chassis_moto_world_calc(Chassis_Data_Rx  Chassis_Data_Receive,fp32 INS_angle[3]);//计算马达速度
 void chassis_moto_body_calc(Chassis_Data_Rx Chassis_Data_Receive);
void chassis_moto_angle_calc(Chassis_Data_Rx  Chassis_Data_Receive);//计算舵轮角度
void chassis_rc_to_control_vector(Chassis_Data_Rx Chassis_Data_Rx_vector);
void track_moto_speed_calc(float vy);//计算轨道马达速度
float Angle_Limit (float angle ,float max);
float target_limit(float ecd, float max);
void Speed_Toggle(int wheel_id, float total_angle[8],int32_t round_count[8]);
int angle_judge(float ecd);
float Find_min_Angle(float angle1, float angle2);
float total_Angle(float last, float current);
void Encoder_Update(Encoder2_t *enc, int i, float new_angle);
float exponential_map(float input, float exp_factor);
float adaptive_exp_factor(float current_speed);
float process_throttle(float raw_input);
void filter1_init(CurrentFilter* filter, float alpha);
float apply_current_filter(CurrentFilter* filter, float current_input);
float signed_to_float(int x_signed, float input_min, float input_max, float output_min, float output_max);
void move_text(int x);
#endif
//end of file
