#ifndef INC_MACRO_H_
#define INC_MACRO_H_

#define NEXT_LINE "\r\n"

#define ARRAY_LEN(x) 						(sizeof(x) / sizeof((x)[0]))
#define IS_NULL(x)							((x) == NULL)
#define GET_PERCENT_VALUE(value, percent) 	((value) * ((percent) / 100.0))

#endif /* INC_MACRO_H_ */
