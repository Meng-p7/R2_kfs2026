#include "CAN_receive_send.h"
#include "math.h"
#include "DM_motor_drive.h"
#include "DJI_motor_drive.h"

#define P_MIN -3.125f
#define P_MAX 3.125f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f



extern DM_motor_data_t DM_pitch_motor_data[4];	

extern motor_measure_t Can1_motor_data[8];
extern motor_measure_t Can2_motor_data[8];

extern encoder_t CAN2_encoder_data[2];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2; 



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header; 
  motor_measure_t *DJI_motor_data;   
  DM_motor_data_t *DM_motor_data;
  uint8_t rx_data[8];            
           
  encoder_t *encoder;            
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); 
 
  if (hcan == &hcan1) 
   
{	DJI_motor_data = Can1_motor_data;

  switch (rx_header.StdId)
  {
  case CAN_ID1:
  {
    get_DJI_motor_measure(DJI_motor_data, rx_data);
    process_motor_data(hcan, DJI_motor_data, 0);
    break;
  }
  case CAN_ID2:
  {
    get_DJI_motor_measure(DJI_motor_data + 1, rx_data);
    process_motor_data(hcan, DJI_motor_data + 1, 1);
    break;
  }
  case CAN_ID3:
  {
    get_DJI_motor_measure(DJI_motor_data + 2, rx_data);
    process_motor_data(&hcan1, DJI_motor_data + 2, 2);
    break;
  }
  case CAN_ID4:
  {
    get_DJI_motor_measure(DJI_motor_data + 3, rx_data);
    process_motor_data(&hcan1,DJI_motor_data + 3, 2);
    break;
  }
  case CAN_ID5:
  {
    get_DJI_motor_measure(DJI_motor_data + 4, rx_data);
    process_motor_data(&hcan1,DJI_motor_data + 4, 2);
    break;
  }
  case CAN_ID6:
  {
    get_DJI_motor_measure(DJI_motor_data + 5, rx_data);
    process_motor_data(&hcan1,DJI_motor_data + 5, 2);
    break;
  }
  case CAN_ID7:
  {
    get_DJI_motor_measure(DJI_motor_data + 6, rx_data);
    process_motor_data(&hcan1, DJI_motor_data + 6, 2);
    break;
  }
  case CAN_ID8:
  {
    get_DJI_motor_measure(DJI_motor_data + 7, rx_data);
    process_motor_data(&hcan1, DJI_motor_data + 7, 2);
    break;
  }
  default:
  {
    break;
  }
}
	}
	if(hcan==&hcan2)
	{

			DM_motor_data = DM_pitch_motor_data;	
		switch (rx_header.StdId)
		{
			case 0x01:
			{
			get_DM_motor_measure(DM_motor_data,rx_data) ;
				break;
			}
			
			case 0x02:
			{
			get_DM_motor_measure(DM_motor_data+1,rx_data) ;
				break;
			}
			  case 0x03:
			{
				get_DM_motor_measure(DM_motor_data+2,rx_data) ;
				break;
			}
			case 0x04:
			{
				get_DM_motor_measure(DM_motor_data+3,rx_data) ;
				break;
			}
		}
		}
	}







