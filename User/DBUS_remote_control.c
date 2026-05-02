#include "DBUS_remote_control.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "user_lib.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
 volatile uint8_t damiao_task_suspended;  
 volatile uint8_t rc_signal_lost;         
 volatile uint32_t last_rc_receive_time;  
 volatile uint8_t remote_power_off = 0; 
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
volatile uint8_t damiao_task_suspended = 0;
 RC_ctrl_t RC_data;
 
 uint8_t test[2];

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

  SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

  
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

  
  __HAL_DMA_DISABLE(&hdma_usart3_rx);
  while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

  
  hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);

  
  hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);

  
  hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);


  hdma_usart3_rx.Instance->NDTR = dma_buf_num;


  SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);


  __HAL_DMA_ENABLE(&hdma_usart3_rx);
}


static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


void remote_control_init(void)
{
  RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

const RC_ctrl_t *get_remote_control_point(void)
{
  return &RC_data;
}

void Dbus_USARTx_IRQHandler() 
{
    if (huart3.Instance->SR & UART_FLAG_RXNE) 
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 25;

        __HAL_UART_CLEAR_PEFLAG(&huart3);
        
     
        last_rc_receive_time = xTaskGetTickCount();
   
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            
            if (this_time_rx_len == 25)
            {
                sbus_to_rc(sbus_rx_buf[0], &RC_data);	
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            
            if (this_time_rx_len == 25)
            {
                sbus_to_rc(sbus_rx_buf[1], &RC_data);
            }
        }
    }
}


static void sbus_to_rc(volatile const uint8_t *buf, RC_ctrl_t *rc_ctrl)
{
    if (buf == NULL || rc_ctrl == NULL) {
        return;
    }


rc_ctrl->rc.ch[1] =   ((int16_t)buf[ 5] >> 1 | ((int16_t)buf[ 6] << 7 )) & 0x07FF;     
rc_ctrl->rc.ch[2] = ((int16_t)buf[ 2] >> 3 | ((int16_t)buf[ 3] << 5 )) & 0x07FF; 
rc_ctrl->rc.ch[3] = ((int16_t)buf[ 1] >> 0 | ((int16_t)buf[ 2] << 8 )) & 0x07FF;
rc_ctrl->rc.ch[4] = ((int16_t)buf[ 3] >> 6 | ((int16_t)buf[ 4] << 2 ) | (int16_t)buf[ 5] << 10 ) & 0x07FF;
rc_ctrl->rc.ch[5]  = ((int16_t)buf[ 6] >> 4 | ((int16_t)buf[ 7] << 4 )) & 0x07FF;
rc_ctrl->rc.ch[6] = ((int16_t)buf[ 7] >> 7 | ((int16_t)buf[ 8] << 1 ) | (int16_t)buf[9] << 9 ) & 0x07FF;
rc_ctrl->rc.ch[7] = ((int16_t)buf[ 9] >> 2 | ((int16_t)buf[10] << 6 )) & 0x07FF;
rc_ctrl->rc.ch[8] = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3 )) & 0x07FF;
rc_ctrl->rc.ch[9] = ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8 )) & 0x07FF;
rc_ctrl->rc.ch[10] = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5 )) & 0x07FF;
rc_ctrl->rc.ch[0] =((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2 ) | (int16_t)buf[16] << 10 ) & 0x07FF;
rc_ctrl->rc.ch[11]= ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7 )) & 0x07FF;
rc_ctrl->rc.ch[12] =((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4 )) & 0x07FF;
rc_ctrl->rc.ch[13] =((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1 ) | (int16_t)buf[20] << 9 ) & 0x07FF;
rc_ctrl->rc.ch[14] =((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6 )) & 0x07FF;
rc_ctrl->rc.ch[15] =((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3 )) & 0x07FF;

               
    uint8_t status = buf[23];
    rc_signal_lost  = (status & (1 << 2)) ? 1 : 0;
    remote_power_off = rc_signal_lost;
	//测试
	test[0]=buf[12];
	test[1]=buf[13];
}

void calibrate_rc_channels_batch(RC_ctrl_t *rc_ctrl, const uint8_t channels[], const int16_t offsets[], const bool inverts[], uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        uint8_t channel = channels[i];
        if (channel >= 1 && channel <= 4) {
            rc_ctrl->rc.ch[channel] -= offsets[i];
            if (inverts[i]) {
                rc_ctrl->rc.ch[channel] = -rc_ctrl->rc.ch[channel];
            }
        }
    }
}

void process_channel(int16_t *channel) {
    int16_t value = *channel;
    
    if (value < 300) {
        *channel = 0;
    } else if (value >= 400 && value < 600) {
        *channel = 1;
    } else if (value >= 700 && value < 1000) {
      *channel = 2;
    } else if (value >= 1100 && value < 1200) {
       *channel = 3;
    } else if (value >= 1300 && value < 1600) {
       *channel = 4;
    } else if (value >= 1700) {
      *channel = 5;
    }
}

void process_binary_channel(int16_t *channel, int16_t low, int16_t high) {
    if (*channel < low) {
        *channel = 0;
    } else if (*channel > high) {
        *channel = 1;
    }
}

void process_ternary_channel(int16_t *channel, int16_t low, int16_t high) {
    if (*channel < low) {
        *channel = 0;
    } else if (*channel > high) {
        *channel = 2;
    } else {
        *channel = 1;
    }
}
float exponential_map(float input, float exp_factor) {
    float abs_input = fabsf(input);
    float output = powf(abs_input, exp_factor);
    return (input >= 0) ? output : -output;
}

float adaptive_exp_factor(float current_speed) {
    float normalized_speed = fminf(fabsf(current_speed), 1.0f); // 归一化速度
    return 0.3f + 1.7f * (1.0f - normalized_speed); // 速度越小，指数越大
}
float process_throttle(float raw_input) {
    static float filtered_speed = 0.0f;
    float norm_input = raw_input / RC_INPUT_MAX;
    filtered_speed = 0.9f * filtered_speed + 0.1f * norm_input;
    float factor = adaptive_exp_factor(filtered_speed);
    float norm_output = exponential_map(norm_input, factor);
    return norm_output * RC_INPUT_MAX;
}

