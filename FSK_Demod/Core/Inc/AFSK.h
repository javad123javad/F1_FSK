/*
 * AFSK.h
 *
 *  Created on: Nov 3, 2020
 *      Author: Javad
 */

#ifndef INC_AFSK_H_
#define INC_AFSK_H_
#include "stm32f1xx_hal.h"
#include <stdint.h>


int afsk_init(TIM_HandleTypeDef* i_timer, uint32_t i_delay);
uint16_t sel_freq(uint8_t i_bit, uint8_t i_pos);
int gen_fsk(TIM_TypeDef * i_TIM,const uint16_t* i_codeByte, const size_t i_len);
int send_msg(const uint8_t* i_msg, const uint8_t i_len);

#endif /* INC_AFSK_H_ */
