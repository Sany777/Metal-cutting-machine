#include "op320.h"

#include "servo_tools.h"
#include "servo_flash.h"
#include "servo_control.h"

#include "../MODBUS_LIB/Modbus.h"
#include "../MODBUS_LIB/ModbusConfig.h"
#include "../SERVO_AMPLIFIER/servo_amplifier.h"

static modbusHandler_t ModbusH;
uint16_t ModbusDATA[MODBUS_DATA_SIZE];

#define allowCmdForRun() \
	( !(isModeServo() &&  reg_but != STOP_SERVO_BUT && reg_but != PAUSE_TOGGLE_BUT) )
#define allowCmdForReverse()	\
	( reg_mode == JOG_REVERSE && ( reg_but == JOG_REVERSE_BUT || reg_but == START_HELP_REV_BUT ) )
#define allowCmdForDirect()	\
	( reg_mode == MODE_JOG_DIRECT &&  ( reg_but == JOG_DIRECT_BUT || reg_but == START_HELP_DIRECT_BUT ) )
#define save() \
	writeToFlash( (uint8_t*)&servoParams, sizeof(servoParams) )

static void showParameter();
static void initTestMode();



void readButton()
{

	if( reg_but != NO_CMD_BUT ){
		if( allowCmdForRun()
					|| allowCmdForReverse()
					|| allowCmdForDirect() ){
	switch( reg_but ){
		case START_HELP_DIRECT_BUT:
		{
			setMode( MODE_HELP_DIRECT );
			break;
		}
		case START_HELP_REV_BUT:
		{
			setMode( MODE_HELP_REVERSE );
			break;
		}
		case STOP_SERVO_BUT:
		{
			MRJ2S_stop();
			break;
		}
		case PAUSE_TOGGLE_BUT :
		{
			if( reg_mode == MODE_PAUSA ){
				MRJ2S_continue();
			} else if( isRun() ){
				MRJ2S_pause();
			} else if (reg_mode == MODE_STOP ){
				reg_total_count = 1;
				setMode( MODE_WORK );
			}
			break;
		}
		case TEST_SERVO_BUT :
		{
			reg_total_count = 1;
			setMode( MODE_WORK );
			break;
		}
		case CUTTING_BUT:
		{
			setMode( MODE_CUTTING );
			break;
		}
		case START_SERVO_BUT :
		{
			if( servoParams.total ){
				reg_total_count = servoParams.total;
				setMode( MODE_WORK );
			}
			break;
		}
		case HELP_ACT_BUT:
		{
			initTestMode();
			toggleHelp();
			break;
		}
		case REVERSE_BUT:
		{
			initTestMode();
			setHelpReverse( !isHelpReverse() );
			break;
		}
		case JOG_DIRECT_BUT :
		{
			setMode( MODE_JOG_DIRECT );
			break;
		}
		case JOG_REVERSE_BUT:
		{
			setMode( MODE_JOG_REVERSE );
			break;
		}
		case CUT_TOGGLE_BUT :
		{
			if( !isOilPomp() )
				setAlarm( OIL_POMP_STOPPED_IND );
			initTestMode();
			toggleCut();
			break;
		}
		case HYDRAVLICS_TOGGLE_BUT:
		{
			if( !isOilPomp() )
				setAlarm( OIL_POMP_STOPPED_IND );
			initTestMode();
			toggleHydravlics();
			break;
		}
		case FIX_TOGGLE_BUT :
		{
			if( !isOilPomp() )
				setAlarm( OIL_POMP_STOPPED_IND );
			initTestMode();
			toggleFix();
			break;
		}
		case PARAM_DEC_BUT :
		{
			if( reg_param_indx == 0 ){
				reg_param_indx = PARAM_NUM;
			} else {
				reg_param_indx -= 1;
			}
			showParameter();
			break;
		}
		case PARAM_INC_BUT :
		{
			if( reg_param_indx == PARAM_NUM ){
				reg_param_indx = 0;
			} else {
				reg_param_indx += 1;
			}
			showParameter();
			break;
		}
		case ENT_PARAM_BUT :
		{
			if( reg_param_indx == KOEF_INDX ){
				if( reg_param_val > MAX_KOEF )
					reg_param_val = MAX_KOEF;
				if( servoParams.koef != reg_param_val ){
					servoParams.koef = reg_param_val;
					save();
				}
			} else if( reg_param_indx == ONLY_SERVO_INDX ){
				const bool activate = reg_param_val != 0;
				if( activate != servoParams.only_servo_f ){
					servoParams.only_servo_f = activate;
					save();
				}
			} else if( reg_param_indx == ACC_TIME_INDX ){
				if( servoParams.accTime != reg_param_val ){
					servoParams.accTime = reg_param_val;
					save();
				}
			} else if( reg_param_indx == SPEED_INDX ){
				if( reg_param_val > MAX_FREQ )
					reg_param_val = MAX_FREQ;
				if( servoParams.freq != reg_param_val ){
					servoParams.freq = reg_param_val;
					save();
				}
			} else if( reg_param_indx == CUT_ACT_TIME_INDX ){
				if( servoParams.cutActTime != reg_param_val ){
					servoParams.cutActTime = reg_param_val;
					save();
				}
			} else if( reg_param_indx == CUT_DEACT_TIME_INDX ){
				if( servoParams.cutDeactTime != reg_param_val ){
					servoParams.cutDeactTime = reg_param_val;
					save();
				}
			} else if( reg_param_indx == FIX_ACT_TIME ){
				if( servoParams.fixActTime != reg_param_val ){
					servoParams.fixActTime = reg_param_val;
					save();
				}
			} else {
				setParam( reg_param_indx, reg_param_val );
			}
			break;
		}
		default : break;
	}
	}

	reg_but = NO_CMD_BUT;
	}
}

static void showParameter()
{
	switch( reg_param_indx ){
		case ACC_TIME_INDX:
			reg_param_val = servoParams.accTime;
			break;
		case SPEED_INDX:
			reg_param_val = servoParams.freq;
			break;
		case  KOEF_INDX:
			reg_param_val = servoParams.koef;
			break;
		case  ONLY_SERVO_INDX:
			reg_param_val = servoParams.only_servo_f>0 ? 1 : 0 ;
			break;
		case  CUT_ACT_TIME_INDX:
			reg_param_val = servoParams.cutActTime;
			break;
		case  CUT_DEACT_TIME_INDX:
			reg_param_val = servoParams.cutDeactTime ;
			break;
		case  FIX_ACT_TIME:
			reg_param_val = servoParams.fixActTime;
			break;
		default:
			reg_param_val = 0;
			getParam( reg_param_indx );
			break;
	}
}

void updateData()
{
	if( reg_len != servoParams.length
			|| reg_total != servoParams.total ){
		if( reg_mode == MODE_STOP
				|| reg_mode == MODE_ALARM ){
			 servoParams.length = reg_len;
			servoParams.total = reg_total;
			save();
		} else {
			reg_len = servoParams.length;
			reg_total = servoParams.total;
		}
	}

	setFlagReg( POMP_IND, isOilPomp() );
	setFlagReg( PRESS_BUT_IND, isPressBut() );
	setFlagReg( PRESS_STOP_IND, isEmergOk() );
	setFlagReg( HELP_SWITCH_IND, isHelpSwitch() );

}

void initOP320()
{
	reg_mode = MODE_STOP;
	reg_len = servoParams.length;
	reg_total = servoParams.total;
	reg_position_len_mm = reg_total_count = 0;
	ModbusH.uModbusType = MB_SLAVE;
	ModbusH.port =  huartOP320;
	ModbusH.u8id = 1;
	ModbusH.u16timeOut = 3000;
	ModbusH.EN_Port = NULL;
	ModbusH.EN_Pin = 0;
	ModbusH.u16regs = ModbusDATA;
	ModbusH.u16regsize=  MODBUS_DATA_SIZE;
	ModbusH.xTypeHW = USART_HW_DMA;
	ModbusInit(&ModbusH);
	ModbusStart(&ModbusH);
}

static void initTestMode()
{
	if( reg_mode != MODE_TEST ){
		MRJ2S_stop();
		reg_mode = MODE_TEST;
		osDelay( 1000 );
	}
}

