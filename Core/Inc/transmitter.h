#ifndef INC_TRANSMITTER_H_
#define INC_TRANSMITTER_H_

#define ACK     0x06U
#define NAK     0x25U
#define SYN     0x26U

#define PWM_LOW_PERCENT 25
#define PWM_HIGH_PERCENT 75

#include <stddef.h>
#include <stdint.h>

uint8_t sending_flag;

void sendHelloMsg(void);
void sendString(const uint8_t*, size_t);
void sendChar(uint8_t);

#endif /* INC_TRANSMITTER_H_ */
