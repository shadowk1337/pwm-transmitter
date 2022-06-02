#ifndef INC_TRANSMITTER_H_
#define INC_TRANSMITTER_H_

#include <stddef.h>
#include <stdint.h>

uint8_t sending_flag;

void sendSYN(void);
void sendACK(void);
void sendString(const uint8_t*, size_t);
void sendChar(uint8_t);

#endif /* INC_TRANSMITTER_H_ */
