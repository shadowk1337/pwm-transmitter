#ifndef INC_RECIEVER_H_
#define INC_RECIEVER_H_

#include <stdint.h>
#include <stdlib.h>

typedef struct {
	uint32_t IC_callback_cnt;
	uint8_t bit_cnt;
	uint32_t bits[8];
} Receive_TypeDef;

typedef struct {
	uint16_t msg_pos;
} Reliable_TypeDef;

void lastBitCheck(void);
void ICCheck(void);

#endif /* INC_RECIEVER_H_ */
