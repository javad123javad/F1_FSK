/*
 * MFSK.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Javad
 */

#ifndef INC_MFSK_H_
#define INC_MFSK_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define NS 128

extern uint16_t gSinWave[NS];

uint16_t sel_freq(uint8_t i_bit, uint8_t i_pos);
int gen_fsk(TIM_TypeDef * i_TIM,const uint16_t* i_codeByte, const size_t i_len);
int send_msg(const uint8_t* i_msg, const uint8_t i_len);



#endif /* INC_MFSK_H_ */
