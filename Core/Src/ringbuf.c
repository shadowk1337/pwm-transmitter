#include "macro.h"
#include "ringbuf.h"

static uint8_t checkInstance(RingBuf_TypeDef *instance) {
	if (IS_NULL(instance->buff) || !ARRAY_LEN(instance->buff)) {
		return 0;
	}
	return 1;
}

void ringBuf_Init(void) {
	rb = (RingBuf_TypeDef*) malloc(sizeof(RingBuf_TypeDef));
	rb->pos = 0;
}

uint8_t* ringBuf_Begin(void) {
	if (!checkInstance(rb)) {
		return NULL;
	}
	return &rb->buff[0];
}

uint8_t* ringBuf_End(void) {
	if (!checkInstance(rb)) {
		return NULL;
	}
	return &rb->buff[ringBuf_Size() - 1];
}

uint8_t* ringBuf_Pos(void) {
	if (!checkInstance(rb)) {
		return NULL;
	}
	return &rb->buff[rb->pos];
}

size_t ringBuf_Size(void) {
	if (!checkInstance(rb)) {
		return 0;
	}
	return ARRAY_LEN(rb->buff);
}
