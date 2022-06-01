#ifndef INC_RINGBUF_H_
#define INC_RINGBUF_H_

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
	uint8_t buff[1024];
	size_t pos;
} RingBuf_TypeDef;

RingBuf_TypeDef *rb;

void ringBuf_Init(void);

uint8_t* ringBuf_Begin();
uint8_t* ringBuf_End();
uint8_t* ringBuf_Pos();
size_t ringBuf_Size();

#endif /* INC_RINGBUF_H_ */
