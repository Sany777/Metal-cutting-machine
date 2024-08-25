#ifndef _TOOLS_H
#define _TOOLS_H

#include "stdarg.h"
#include "stdint-gcc.h"
#include "string.h"


#define MIN(a, b) 							((a)>(b)?(b):(a))
#define MAX(a, b) 							((a)>(b)?(a):(b))


#define setFlag(_flags, _bit) 			((_flags) |= (1<< (_bit)))

#define clearFlag(_flags, _bit) 		((_flags) &= ~(1<< (_bit)))

#define getFlag(_flags, _bit) 			((_flags)&(1<< (_bit)))


#define toggleBit(_flags, _bit )	 		\
	do{ 													\
			if( getFlag(_flags, _bit) ){			\
				clearFlag(_flags, _bit);		 	\
			} 	else { 									\
				setFlag(_flags, _bit); 				\
			} 												\
	}while(0)

#define setBit(_flags, _bit, _val)	 		\
	do{ 													\
		if(_val){ 										\
			setFlag(_flags, _bit); 					\
		}else{ 											\
			clearFlag(_flags, _bit);		 		\
		} 													\
}while(0)


uint8_t getCharValue( uint8_t c, uint8_t hex );
uint8_t* numToString(uint64_t value, uint8_t* buf, const  uint8_t digits, const uint8_t base);
uint64_t getNumberFromString( const uint8_t* data, const size_t numLen, const uint8_t base );

#endif
