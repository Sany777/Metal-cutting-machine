#include "servo_control.h"

#include "servo_amplifier.h"
#include "servo_flash.h"
#include "servo_tools.h"

device_mode_t cur_state;
volatile static int32_t basic_point, expected_len, TOTAL_LEN, total_len_dec,  total_len_incr, imp_in_mm, len_count;
static bool positioning_run, positioning_mode, jog_run, init_work_values;

static void hidravlicsOff();
static void initCicle();
static void continueSetLength();
static void initCounter();
static void initPositioning();
static void initJog( direction_t  direct );
static void deinitJog();
static void deinitPositioning();
static void setWorkValues();


void continueServo( const int32_t imp )
{
	volatile int32_t pass_len;

	if( imp < 0 ){
		pass_len = basic_point >= imp
								? imp - basic_point
								: -MAX_IMP_VAl - basic_point + imp;
	} else {
		pass_len = imp >= basic_point
								? imp - basic_point
								: MAX_IMP_VAl - basic_point + imp;
	}

	total_len_incr += pass_len;
	if( isModeServo() ) {

		len_count += pass_len;

		if( len_count >= expected_len ) {
			if( total_len_dec == 0 ) {
				reg_mode = MODE_OPERATION;
				if( isHelpWork() )
					setHelp( false );
				reg_position_len_mm = servoParams.length;
				operation();
				initCicle();
				reg_mode = cur_state;
			} else if( positioning_run ) {
				continueSetLength();
			} else {
				// count in jog mode
				total_len_dec  -= pass_len;
				// change mode for precision
				if( total_len_dec < MAX_IMP_NUM ){
					reg_mode = MODE_OPERATION;
					if( ! isTxQueueEmpty() ){
						osMessageQueueReset( txMesQueue );
					}
					initPositioning();
					if( ! isRxQueueEmpty() ){
						osMessageQueueReset( rxMesQueue );
					}
					osDelay( 100 );
					getFeedbackImp();
					reg_mode = cur_state;
				}
			}
			len_count = 0;
		}

	} else if( reg_mode == MODE_JOG_REVERSE
				&& total_len_incr < TOTAL_LEN ){
		deinitJog();
		reg_mode = MODE_HELP_REVERSE;
		setFlagReg( HELP_REV_ACT_IND, true );
	}
	basic_point = imp;
	reg_position_len_mm = total_len_incr / imp_in_mm;
}

void  MRJ2S_pause()
{
	setHelp( false );
	if( cur_state != MODE_STOP ){
		reg_mode = MODE_PAUSA;
	}
	if( positioning_run ){
		servoPause();
	} else if( jog_run ){
		jogStop();
	}
	setLed( false );
}

void MRJ2S_continue()
{
	if( reg_mode == MODE_PAUSA ){
		if( positioning_run ){
			servoStart();
			continueSetLength();
		} else if ( cur_state == MODE_JOG_REVERSE ) {
			jogStartReverse();
		} else {
			jogStartDirect();
		}
		setLed( true );
	} else {
		setLed( false );
	}
	reg_mode = cur_state;
}

void MRJ2S_stop()
{
	cur_state = reg_mode = MODE_STOP;
	reg_device_flags = 0;
	setHelp( false );
	if( positioning_run ){
		deinitPositioning();
	} else if( jog_run ){
		deinitJog();
	}
	initCounter();
	if( alarm ){
		resetAlarm();
	}
	hidravlicsOff();
	setLed( false );
}


const bool setMode( const device_mode_t mode )
{
	bool result = false;
	if( isEmergOk() && reg_mode == MODE_STOP ) {
		if( mode == MODE_WORK
				|| mode == MODE_JOG_DIRECT
				|| mode == MODE_JOG_REVERSE ){
			if( alarm ){
				resetAlarm();
				osDelay( 500 );
			}
			resetFeedback();
			osDelay( 100 );
			setParam( 0, 1 );
			osDelay( 100 );
			blockIO( BLOCK_DI_AI );
			basic_point = 0;
			init_work_values = false;
			imp_in_mm = IMP_IN_MM_CONST+servoParams.koef;
		}
		switch( mode ){
			case MODE_HELP_DIRECT:
			{
				setHelpReverse( false );
				setFlagReg( HELP_DIRECT_ACT_IND, true );
				reg_mode = mode;
				break;
			}
			case MODE_HELP_REVERSE:
			{
				setHelpReverse( true );
				setFlagReg( HELP_REV_ACT_IND, true );
				reg_mode = mode;
				break;
			}
			case MODE_CUTTING:
			{
				if( !isOilPomp() ){
					setAlarm( OIL_POMP_STOPPED_IND );
				}
				setFlagReg( CUTTING_IND, true );
				reg_mode = mode;
				break;
			}
			case MODE_WORK :
			{
				if( !isOilPomp() ){
					setAlarm( OIL_POMP_STOPPED_IND );
				} else if(  servoParams.length > 20 ){
					setHelpReverse( false );
					TOTAL_LEN = servoParams.length*imp_in_mm;
					positioning_mode =
							servoParams.only_servo_f
							|| TOTAL_LEN <= MAX_IMP_NUM;
					initCicle();
					result = true;
				}
				break;
			}
			case MODE_JOG_DIRECT:
			{
				setHelpReverse( false );
				initCounter();
				initJog( JOG_DIRECT );
				result = true;
				break;
			}
			case MODE_JOG_REVERSE:
			{
				setHelpReverse( true );
				TOTAL_LEN = - ( servoParams.length+MACHINE_LEN_MM )*imp_in_mm;
				initCounter();
				initJog( JOG_REVERSE );
				result = true;
				break;
			}
			default:break;
			}
			if( result ){
				setLed( true );
				cur_state = reg_mode = mode;
			}
	} else {
		MRJ2S_stop();
	}
	return result;
}

void setHelpReverse( const bool state )
{
	if( isHelpReverse() != state ){
		if( isHelpWork() ){
			setHelp( false );
			osDelay( DELAY_CON_CONTACTOR );
		}
		if( !isHelpWork() ){
			HAL_GPIO_WritePin( out_help_reverse_Port, out_help_reverse_Pin,
								state
								? GPIO_PIN_SET
								: GPIO_PIN_RESET );
			setFlagReg( HELP_REVERSE_IND, state );
		}
	}
}

void activateHydravlics( const bool state )
{
	HAL_GPIO_WritePin( out_pneumatic_act_Port, out_pneumatic_act_Pin,
			state
			? GPIO_PIN_SET
			: GPIO_PIN_RESET );
	setFlagReg( HYDR_IND, state );
}

void activateFix( const bool state )
{
	HAL_GPIO_WritePin( out_fix_act_Port, out_fix_act_Pin,
			state
			? GPIO_PIN_SET
			: GPIO_PIN_RESET );
	setFlagReg( FIX_IND, state );
}

void activateCut( const bool state )
{
	HAL_GPIO_WritePin( out_cut_act_Port, out_cut_act_Pin,
		state
		? GPIO_PIN_SET
		: GPIO_PIN_RESET );
		setFlagReg(CUT_IND, state);
}

void setHelp( bool state )
{
	if( !isEmergOk() && state ){
		state = false;
	}
	HAL_GPIO_WritePin( out_help_ON_Port, out_help_ON_Pin,
			state
			? GPIO_PIN_SET
			: GPIO_PIN_RESET );
	setFlagReg( HELP_IND, state );
}

static void continueSetLength()
{
	if( total_len_dec <= MAX_IMP_NUM ){
		total_len_dec = TOTAL_LEN - total_len_incr;
	}
	expected_len = MIN( total_len_dec, MAX_IMP_NUM );
	servoSetLength( expected_len );
	total_len_dec -= expected_len;
	expected_len -= MISMATCH_IMP;
}

static void initCicle()
{
	initCounter();
	if( reg_total_count > 0 ){
		reg_total_count -= 1;
		total_len_dec = TOTAL_LEN;
		if( positioning_mode ){
			if( ! positioning_run ){
				initPositioning();
			}
			continueSetLength();
		} else {
			initJog( JOG_DIRECT );
		}
	} else {
		MRJ2S_stop();
	}
}

static void hidravlicsOff()
{
	if( isCut() || isFix()	){
		bool is_pomp =  isOilPomp();
		if( is_pomp ){
			activateHydravlics( true );
		}
		activateCut( false );
		activateFix( false );
		if( is_pomp ){
			osDelay( servoParams.cutDeactTime );
		}
	}
	activateHydravlics( false );
}

static void initCounter()
{
	len_count = 0;
	total_len_incr  		 = 0;
	total_len_dec  		 = 0;
	reg_position_len_mm  	= 0;
}

static void setWorkValues()
{
	if( !init_work_values ){
		setAccTimeParam( servoParams.accTime );
		setFreqParam( servoParams.freq );
		init_work_values = true;
	}
}

static void initPositioning()
{
	if( jog_run ){
		deinitJog();
	}
	setWorkMode( SET_SERVO );
	setWorkValues();
	servoStart();
	positioning_run = true;
	expected_len = 0;
}

static void deinitPositioning()
{
	servoPause();
	servoOFF();
	positioning_run = false;
}

static void initJog( direction_t  direction )
{
	if( positioning_run ){
		deinitPositioning();
	}
	setWorkMode( SET_JOG );
	setWorkValues();
	 setFlagReg(
			 direction == JOG_DIRECT
			 ? DIRECT_IND
			 : REVERSE_IND,
			   true );
	 direction == JOG_DIRECT
	 			? jogStartDirect()
	 			: jogStartReverse();
	expected_len = imp_in_mm;
	jog_run = true;
}

static void deinitJog()
{
	jogStop();
	osDelay( 100 );
	jogOFF();
	setFlagReg( DIRECT_IND, false );
	setFlagReg( REVERSE_IND, false );
	jog_run = false;
}

void operation()
{
	if( !isOilPomp() ){
		setAlarm( OIL_POMP_STOPPED_IND );
		do{
			if( reg_mode == MODE_STOP ){
				reg_total_count = 0;
				setLed( false );
				return;
			}
			setLed( !isLed() );
			osDelay( 500 );
		} while( !isOilPomp() );
		setLed( true );
	}

	activateHydravlics( true );
	osDelay( 200 );
	activateFix( true );
	osDelay( servoParams.fixActTime );
	activateCut( true );
	osDelay( servoParams.cutActTime );
	activateCut( false );
	activateFix( false );
	osDelay( servoParams.cutDeactTime );
	activateHydravlics( false );
}

void updateAlarmStatus( const int alarm )
{
	if( alarm == AL_OK ){
		reg_alarms = 0;
		reg_mode = MODE_STOP;
	} else {
		if( reg_mode != MODE_ALARM ){
			MRJ2S_stop();
			reg_mode = MODE_ALARM;
		}
		setAlarm( getAlarmIndx( alarm ));
	}
}
