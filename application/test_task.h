#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "struct_typedef.h"
#include "DM_motor_drive.h"
typedef struct
{ float x;
  float y;
  float z;
} Coordinates_t;


static void state_stay(void);
static void state_climb(void);
static void state_move(void);
int INS_angle_reset(fp32 INS_angle[3]);
int motor_step(DM_motor_data_t *DM_Receive,int i,float set);
int motor_step2(DM_motor_data_t *DM_Receive,int i,float set1,float set2) ;
 void Stress_cone_anologize(Coordinates_t *body_coordinates,Coordinates_t *arm_start_coordinates,Coordinates_t *arm_end_coordinates)  ;
  void Under_force_anologize(Coordinates_t *body_coordinates,Coordinates_t *arm_start_coordinates,Coordinates_t *arm_end_coordinates)  ;
  void Euler_angles(Coordinates_t* a,fp32 *INS_angle);
  int motor_step3(DM_motor_data_t *DM_Receive,int i,float set);
#endif
