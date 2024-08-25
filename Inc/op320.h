#ifndef OP320_H
#define OP320_H

#include "stdbool.h"
#include "stdint-gcc.h"
#include "servo_tools.h"

#define MODBUS_DATA_SIZE 	32
#define DELAY_SEND_CMD 		200
#define DELAY_STOP_HELP  		2000


enum{
	STATUS_NUM					= 13,
	ONLY_SERVO_INDX			= 85,
	ACC_TIME_INDX 				= 86,
	CUT_ACT_TIME_INDX		= 87,
	CUT_DEACT_TIME_INDX	= 88,
	FIX_ACT_TIME					= 89,
	SPEED_INDX 					= 90,
	KOEF_INDX						= 91,
	PARAM_NUM					= 91
};

enum IndicatorsRegister{
	REVERSE_IND,
	DIRECT_IND,
	HELP_IND,
	HELP_REVERSE_IND,
	CUT_IND,
	FIX_IND,
	HYDR_IND,
	POMP_IND,
	PRESS_STOP_IND,
	PRESS_BUT_IND,
	HELP_SWITCH_IND,
	CUTTING_IND,
	HELP_REV_ACT_IND,
	HELP_DIRECT_ACT_IND,
};

enum ModbusRegistersMap{
	ALARMS_REG 				= 0,
	FLAGS_REG 					= 2,
	TOTAL_REG 					= 5,
	TOTAL_COUNT_REG 	= 6,
	LENGTH_REG 				= 7,
	LENGTH_COUNT_REG	= 8,
	MAIN_MES_REG 			= 9,
	BUTTON_REG 				= 10,
	PARAMETER_REG 			= 12,
	PARAM_VALUE_REG 	= 13,
};

typedef enum {
	MODE_STOP,
	MODE_ALARM,
	MODE_PAUSA,
	MODE_WORK,
	MODE_TEST,
	MODE_JOG_DIRECT,
	MODE_JOG_REVERSE,
	MODE_CUTTING,
	MODE_HELP_DIRECT,
	MODE_HELP_REVERSE,
	MODE_OPERATION,
	MODE_TEST_FIX,
	MODE_TEST_CUT,
	MODE_TEST_HYDR,
	MODE_TEST_REVERSE,
	MODE_TEST_HELP

}device_mode_t;


enum Buttons{
	NO_CMD_BUT							= 0,
	START_SERVO_BUT 					= 1,
	TEST_SERVO_BUT 	 			    	= 2,
	PAUSE_TOGGLE_BUT				= 3,
	HELP_ACT_BUT							= 5,
	REVERSE_BUT 							= 7,
	CUT_TOGGLE_BUT 	 				= 8,
	FIX_TOGGLE_BUT 	 					= 9,
	CUTTING_BUT							= 10,
	PARAM_INC_BUT 	 					= 12,
	PARAM_DEC_BUT 	 				= 13,
	ENT_PARAM_BUT 	 				= 14,
	HYDRAVLICS_TOGGLE_BUT		= 15,
	JOG_DIRECT_BUT						= 16,
	JOG_REVERSE_BUT					= 17,
	START_HELP_DIRECT_BUT		= 18,
	START_HELP_REV_BUT				= 19,
	STOP_SERVO_BUT					= 33,
};

#define reg_alarms   					*( (uint32_t*)( &ModbusDATA[ALARMS_REG] ) )
#define reg_device_flags			ModbusDATA[FLAGS_REG]
#define reg_but							ModbusDATA[BUTTON_REG]
#define reg_total						ModbusDATA[TOTAL_REG]
#define reg_total_count				ModbusDATA[TOTAL_COUNT_REG]
#define reg_len							ModbusDATA[LENGTH_REG]
#define reg_position_len_mm	ModbusDATA[LENGTH_COUNT_REG]
#define reg_mode						ModbusDATA[MAIN_MES_REG]
#define reg_param_indx			ModbusDATA[PARAMETER_REG]
#define reg_param_val				ModbusDATA[PARAM_VALUE_REG]

#define setAlarm( indx )	\
	setFlag( reg_alarms, (indx) )

#define setFlagReg( indx, value )\
	setBit( reg_device_flags, (indx), (value) )

#define getFlagReg( indx ) \
	getFlag( reg_device_flags, (indx) )

#define getAlarmReg( indx )	\
	getFlag( reg_alarms, (indx) )

#define setStatusVal( val ) 	\
	setRegAsFloat( &reg_info_val, (val) )

#define toggleFlagReg( indx )	\
	toggleBit( reg_device_flags, (indx) )

void initOP320();


void updateData();
void readButton();


extern uint16_t ModbusDATA[];

#endif
