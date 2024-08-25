#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "main.h"
#include "stdbool.h"
#include "op320.h"



enum ParamMachine {
	MACHINE_LEN_MM				= 2100,
	MAX_IMP_VAl 						= 9999999,
	IMP_IN_SPIN							= 131072,
	MAX_IMP_NUM					=  MAX_IMP_VAl- IMP_IN_SPIN*2,
	MAX_FREQ							= 1000,
	MAX_KOEF							= 999,
	DELAY_CON_CONTACTOR	= 1000,
	START_POMP_DELAY			= 5000,
	DELAY_STOP_SERVO			= 500,
	IMP_IN_MM_CONST				= 19000,
	MISMATCH_IMP					= 2000,
	NO_EXPECTED_LEN				= MAX_IMP_VAl+1,
	MAX_TIMEOUT						= 4,
	MAX_EXPECTED_LEN			=  ( IMP_IN_SPIN*MAX_FREQ*MAX_TIMEOUT ) / 60
};

void updateAlarmStatus( const int alarm );
void operation();
void setHelpReverse( const bool state );
void MRJ2S_stop();
const bool setMode( const device_mode_t mode );
void MRJ2S_pause();
void MRJ2S_continue();
void continueServo( const int32_t imp );
void activateHydravlics( const bool state );
void activateFix( const bool state );
void activateCut( const bool state );
void setHelp( bool state );

#define isPressBut()				( HAL_GPIO_ReadPin( in_start_Port, in_start_Pin ) == GPIO_PIN_SET )
#define isOilPomp()				( HAL_GPIO_ReadPin( in_pomp_Port, in_pomp_Pin) == GPIO_PIN_SET )
#define isEmergOk()				( HAL_GPIO_ReadPin( in_emerg_Port, in_emerg_Pin ) == GPIO_PIN_SET )
#define isHelpSwitch()			( HAL_GPIO_ReadPin( in_help_sw_Port, in_help_sw_Pin ) == GPIO_PIN_SET )

#define isHydravlics()			( HAL_GPIO_ReadPin( out_pneumatic_act_Port, out_pneumatic_act_Pin) == GPIO_PIN_SET )
#define isCut()						( HAL_GPIO_ReadPin( out_cut_act_Port, out_cut_act_Pin) == GPIO_PIN_SET )
#define isHelpWork()				( HAL_GPIO_ReadPin( out_help_ON_Port, out_help_ON_Pin ) == GPIO_PIN_SET )
#define isHelpReverse()			( HAL_GPIO_ReadPin( out_help_reverse_Port, out_help_reverse_Pin ) == GPIO_PIN_SET )
#define isFix()							( HAL_GPIO_ReadPin( out_fix_act_Port, out_fix_act_Pin ) == GPIO_PIN_SET )

#define toggleHydravlics()	activateHydravlics( !isHydravlics() )
#define toggleCut()				activateCut( !isCut() )
#define toggleFix()					activateFix( !isFix() )
#define toggleHelp()				setHelp( !isHelpWork() )

#define isLed()						( HAL_GPIO_ReadPin( out_led_Port, out_led_Pin ) == GPIO_PIN_SET )

#define setLed( state )	 \
	HAL_GPIO_WritePin( out_led_Port, out_led_Pin,	\
	state	\
	? GPIO_PIN_SET	\
	: GPIO_PIN_RESET );

#define isModeServo() 			( reg_mode == MODE_WORK )
#define isModeJog() 				( reg_mode == MODE_JOG_DIRECT || reg_mode == MODE_JOG_REVERSE )
#define isHelpMode()			( reg_mode == MODE_HELP_DIRECT || reg_mode = MODE_HELP_REVERSE )
#define isRun()						( isModeServo() || isModeJog() )

#define isMove()	\
		( isRun() || isHelpMode() )

extern device_mode_t cur_state;
#endif
