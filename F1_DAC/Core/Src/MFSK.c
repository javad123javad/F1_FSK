/*
 * MFSK.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Javad Rahimi
 *      A simple implementation of MFSK modulation scheme for long range telemetry
 *      systems
 */
#include "MFSK.h"
#include <string.h>

uint16_t gSinWave[NS] =
  { 2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837, 2929, 3020, 3108,
      3193, 3275, 3355, 3431, 3504, 3574, 3639, 3701, 3759, 3812, 3861, 3906,
      3946, 3982, 4013, 4039, 4060, 4076, 4087, 4094, 4095, 4091, 4082, 4069,
      4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730, 3671, 3607, 3539,
      3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790, 2695, 2598, 2500,
      2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595, 1497, 1400, 1305,
      1212, 1120, 1031, 944, 860, 779, 701, 627, 556, 488, 424, 365, 309, 258,
      211, 168, 130, 97, 69, 45, 26, 13, 4, 0, 1, 8, 19, 35, 56, 82, 113, 149,
      189, 234, 283, 336, 394, 456, 521, 591, 664, 740, 820, 902, 987, 1075,
      1166, 1258, 1353, 1449, 1546, 1645, 1745, 1845, 1946, 2047 };


uint16_t sel_freq(uint8_t i_bit, uint8_t i_pos)
{
  uint16_t iFreq = 0;

  switch (i_pos)
  {
    case(0):
            iFreq = (i_bit == 0) ? 561 : 449;
    break;
    case(1):
            iFreq = (i_bit == 0) ? 374 : 320;
    break;
    case(2):
            iFreq = (i_bit == 0) ? 280 : 249;
    break;
    case(3):
            iFreq = (i_bit == 0) ? 224 : 203;
    break;

  }
  return iFreq;
}
int gen_fsk(TIM_TypeDef * i_TIM,const uint16_t* i_codeByte, const size_t i_len)
{

  // GEN STX
  i_TIM->ARR = 124;
  HAL_Delay(20);
  // GEN MSG
  for (int i = 0; i < i_len; i++)
    {
      i_TIM->ARR = i_codeByte[i];
      HAL_Delay(20);


    }
  //GEN ETX
  i_TIM->ARR = 111;
  HAL_Delay(20);
  return 0;
}
int send_msg(const uint8_t* i_msg, const uint8_t i_len)
{
  uint16_t codeByte[8] = { 0 };

  for (int ii = 0; ii < i_len; ii++)
    {
      uint8_t iByte = i_msg[ii];

      for (int i = 0; i < 8; i++)
	{
	  uint8_t bits = (iByte >> i) & 0x01;// (1 << i);
	  codeByte[i] = sel_freq(bits, (i % 4));
	}
      gen_fsk(TIM2, codeByte, 4);
      gen_fsk(TIM2, codeByte+4, 4);

      memset(codeByte,0,sizeof(codeByte));
    }
  TIM2->ARR = 99;
  HAL_Delay(20);
  return 0;
}
