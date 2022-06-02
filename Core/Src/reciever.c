#include "reciever.h"
#include "macro.h"

#include "main.h"

Receive_TypeDef r = { .IC_callback_cnt = 0, .bit_cnt = 0 };

static uint8_t binToChar(uint32_t *bits) {
	uint8_t ch = 0;
	for (int i = 0, rank = 128; i < 8 && rank > 0; i++, rank /= 2) {
		if (bits[i] > TIM3->ARR * 0.5) {
			ch += rank;
		}
	}
	return ch;
}

void lastBitCheck(void) {
	if (r.bit_cnt == 7 && r.IC_callback_cnt > 1) {
		r.bits[r.bit_cnt] = TIM3_CH1_ReadCapValue();

		uint8_t ch = binToChar(r.bits);
		HAL_UART_Transmit(&huart1, &ch, 1, -1);

		r.bit_cnt = r.IC_callback_cnt = 0;
	}
}

void ICCheck(void) {
	r.IC_callback_cnt++;

	uint32_t ra = TIM3_CH1_ReadCapValue();
	uint32_t fa = TIM3_CH2_ReadCapValue();

	if (r.IC_callback_cnt % 2 != 0 && r.IC_callback_cnt != 1) {
		uint32_t rf = ra + fa;

		if (rf >= 0.9 * TIM3->ARR && rf <= 1.1 * TIM3->ARR) {
			r.bits[r.bit_cnt] = ra;
		}
		r.bit_cnt++;
	}

	TIM3->CNT = 0;
}
