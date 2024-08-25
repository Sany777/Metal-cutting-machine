#include "servo_tools.h"

#include "main.h"
#include "math.h"
#include "servo_amplifier.h"

#define END_NUM 0xff

uint8_t* numToString(uint64_t value, uint8_t* buf, const  uint8_t digits, const uint8_t base)
{
    const char hex_chars[] = "0123456789ABCDEF";
    uint8_t size_num = digits;
    uint8_t tmp,
        * end = buf,
        * ptr = buf;
    do{
        *end = hex_chars[value % base];
        if( size_num < 2 ) break;
        size_num -= 1;
        value /= base;
        end += 1;
    }while(1);
    while (ptr < end) {
        tmp = *ptr;
        *ptr = *end;
        *end = tmp;
        ptr += 1;
        end -= 1;
    }
    return buf + digits;
}


uint8_t getCharValue( uint8_t c, uint8_t base_hex )
{
	if( c >= '0' && c <='9' ) return c - '0';
	if( base_hex ){
		if( c >= 'A' && c <= 'F' ) return c - 'A'+10;
		if( c >= 'a' && c <= 'f' ) return c - 'a'+10;
	}
	return END_NUM;
}

uint64_t getNumberFromString( const uint8_t* data, const size_t numLen, const uint8_t base )
{
	uint64_t result = 0;
	const uint8_t* end = data+numLen;
	uint8_t c;
	while( c =  getCharValue( *data, base == BASE_HEX ),
			c != END_NUM && data < end ){
		result = result*base + c;
		data +=1;
	}
	return result;
}
