/*
 * AFSK.c
 *
 *  Created on: Nov 3, 2020
 *      Author: Javad
 */
#include "AFSK.h"

typedef struct _AFSK_CONF_
{
  TIM_HandleTypeDef 	*PWM_Timer;
  uint32_t		delay;
}ASK_CONF;
static ASK_CONF pConf;

int afsk_init(TIM_HandleTypeDef* i_timer, uint32_t i_delay)
{
  pConf.PWM_Timer = i_timer;
  pConf.delay = i_delay;

  return 0;
}


uint16_t sel_freq(uint8_t i_bit, uint8_t i_pos)
{
  uint16_t iFreq = 0;

  switch (i_pos)
  {
    case(0):
            iFreq = (i_bit == 0) ? 4999 : 3599;// 0---> 720Hz, 1--->1000
    break;
    case(1):
            iFreq = (i_bit == 0) ? 2999 : 2571;// 0--->1200, 2499--1440
    break;
    case(2):
            iFreq = (i_bit == 0) ? 2399 : 1999;// 0--->1500, 1--->1800
    break;
    case(3):
            iFreq = (i_bit == 0) ? 1799 : 1636;// 0--->2000, 1--->2400
    break;

  }
  return iFreq;
}
int gen_fsk(TIM_TypeDef * i_TIM,const uint16_t* i_codeByte, const size_t i_len)
{

  // GEN STX
  i_TIM->PSC = 7199;// STX---> 500Hz//399;
  HAL_Delay(pConf.delay);
  // GEN MSG
  for (int i = 0; i < i_len; i++)
    {
      i_TIM->PSC = i_codeByte[i];
      HAL_Delay(pConf.delay);


    }
  //GEN ETX
  i_TIM->PSC = 1439;//359;ETX---> 2500Hz
  HAL_Delay(pConf.delay);
  return 0;
}
int send_msg(const uint8_t* i_msg, const uint8_t i_len)
{
  uint16_t codeByte[8] = { 0 };

  HAL_TIM_PWM_Start_IT(pConf.PWM_Timer, TIM_CHANNEL_1);

  for (int ii = 0; ii < i_len; ii++)
    {
      uint8_t iByte = i_msg[ii];

      for (int i = 0; i < 8; i++)
	{
	  uint8_t bits = (iByte >> i) & 0x01;// (1 << i);
	  codeByte[i] = sel_freq(bits, (i % 4));
	}
      gen_fsk(TIM4, codeByte, 4);
      gen_fsk(TIM4, codeByte+4, 4);

      memset(codeByte,0,sizeof(codeByte));
    }
  TIM4->PSC = 1286;//319;
  HAL_Delay(5);
  HAL_TIM_PWM_Stop_IT(pConf.PWM_Timer, TIM_CHANNEL_1);

  return 0;
}
