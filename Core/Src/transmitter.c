#include "transmitter.h"
#include "macro.h"

#include "main.h"

static void processString(const uint8_t *str, size_t len) {
	TIM3_Stop_IT();

	const uint8_t *s = str;
	while (len > 0) {
		sendChar(*s);
		HAL_UART_Transmit(&huart1, s, 1, -1);
		len--, s++;
	}

	TIM3_Start_IT();
}

void sendACK(void) {
	sendChar(ACK);
}

void sendSYN(void) {
	sendChar(SYN);
}

void sendString(const uint8_t *str, size_t len) {
	processString(str, len);
	while (!(USART1->ISR & USART_ISR_TC))
		;
}

void sendChar(uint8_t ch) {
	sending_flag = 1;
	for (int32_t i = 0, mv = 7; i <= 8; ++i, --mv) {
		uint16_t duty_cycle;

		if (i == 8) {
			duty_cycle = PWM_IDLE_PERCENT;
		} else {
			uint8_t bit = (ch & (1 << mv)) ? 1 : 0;
			duty_cycle = bit == 0 ? PWM_LOW_PERCENT : PWM_HIGH_PERCENT;
		}

		TIM2_CH2_SetCompare(GET_PERCENT_VALUE(TIM2->ARR, duty_cycle));

		TIM2_CH2_StartPWM_IT();
		while (sending_flag)
			;
		sending_flag = 1;
	}
}
