#include "reciever.h"
#include "macro.h"

#include "main.h"

Receive_TypeDef r = { .IC_callback_cnt = 0, .bit_cnt = 0 };
Reliable_TypeDef rel = { .msg_pos = 0 };

static uint8_t binToChar(uint32_t *bits) {
	uint8_t ch = 0;
	for (int32_t i = 0, mv = 7; i < 8; i++, mv--) {
		uint32_t bit = (bits[i] > TIM3->ARR / 2u) ? 1 : 0;
		ch |= bit << mv;
	}
	return ch;
}

static void checkReceivedCharSYNACK(uint8_t ch) {
	if (rel.msg_pos == 0) {
		if (ch == SYN) {
			rel.msg_pos = 1;
		}
	} else {
		if (ch == ACK) {
			preparedForReceive();
		}
		rel.msg_pos = 0;
	}
}

void lastBitCheck(void) {
	if (r.bit_cnt == 7 && r.IC_callback_cnt > 1) {
		r.bits[r.bit_cnt] = TIM3_CH1_ReadCapValue();

		uint8_t ch = binToChar(r.bits);
		checkReceivedCharSYNACK(ch);

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
