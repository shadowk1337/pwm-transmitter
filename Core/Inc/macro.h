#ifndef INC_MACRO_H_
#define INC_MACRO_H_

#define NEXT_LINE "\r\n"

#define ACK     0x06U
#define NAK     0x25U
#define SYN     0x26U

#define PWM_LOW_PERCENT 25
#define PWM_HIGH_PERCENT 75
#define PWM_IDLE_PERCENT 101

#define ARRAY_LEN(x) 						(sizeof(x) / sizeof((x)[0]))
#define IS_NULL(x)							((x) == NULL)
#define GET_PERCENT_VALUE(value, percent) 	((value) * ((percent) / 100.0))

#endif /* INC_MACRO_H_ */
