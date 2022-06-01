#include "transmitter.h"
#include "macro.h"

#include "main.h"

static void processString(const uint8_t *str, size_t len) {
	const uint8_t *s = str;
	while (len > 0) {
		sendChar(*s);
		len--, s++;
	}
}

void sendHelloMsg(void) {
	sendChar(ACK);
}

void sendString(const uint8_t *str, size_t len) {
	processString(str, len);
	while (!(USART1->ISR & USART_ISR_TC))
		;
}

void sendChar(uint8_t ch) {
	sending_flag = 1;
	for (int i = 0, mv = 7; i <= 7; ++i, --mv) {
		uint8_t bit = (ch & (1 << mv)) ? 1 : 0;
		uint16_t duty_cycle = bit == 0 ? PWM_LOW_PERCENT : PWM_HIGH_PERCENT;

		TIM2_CH2_SetCompare(GET_PERCENT_VALUE(TIM2->ARR, duty_cycle));

		TIM2_CH2_StartPWM_IT();
		while (sending_flag)
			;
		sending_flag = 1;
	}
}
