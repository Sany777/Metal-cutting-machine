#ifndef SERVO_H__
#define SERVO_H__


#include "stdint.h"
#include "stdbool.h"

#include "main.h"

#define SERVO 	'0'
#define SOH 	1
#define STX   	2
#define ETX   	3
#define EOT  	"\4"



enum ConstServo{

	BASE_HEX 						= 16,
	BASE_DEC 						= 10,

	GET_DATA						= 0,
	SET_DATA							= 1,

	SAVE_EIPROM 					= 0,
	DEFAULT_VAL    				= 0x1EA5,

	ATEMPTS							= 3,
	TIMEOUT_SEND_MES		= 500,
	TIMEOUT_WAIT_MES		= 310,
	DELAY_SEND_EOT			= 100,
	WAIT_RESPONSE_MS		= (TIMEOUT_SEND_MES+DELAY_SEND_EOT)*ATEMPTS,

	START_CRC_AREA				= 1,
	REQ_HEADER_SIZE			= 7,
	RESP_HEADER_SIZE 			= 6,

	SERVO_BUF_SIZE 				= 50,
	SIZE_CMD 						= 2,
	SIZE_CRC  						= 2,
	SIZE_DATA_NUM 				= 2,
	SIZE_SYMB 						= 1,
	SIZE_PARAM_DATA 			= 8,
	SIZE_RX_QUEUE 				= 5,
	SIZE_CMD_QUEUE 			= 20,

	// [SOH] [SERVO] [CMD_H ,CMD_L] [STX] [DATA_NUM_H, DATA_NUM_L] [DATA] [ETX] [CRC_H,CRC_L]
	TX_POS_CMD 					= SIZE_SYMB+SIZE_SYMB,
	TX_POS_DATA_NUM 		= TX_POS_CMD+SIZE_CMD+SIZE_SYMB,
	TX_POS_DATA 					= TX_POS_DATA_NUM+SIZE_DATA_NUM,

	//  [STX][ADDR][RESP][DATA][ETX][CRC_H,CRC_L]
	RX_POS_RESP 					= 2,
	RX_POS_DATA					= 3,

	RX_POS_OUTPUT_DATA 	= 7

};

enum RxParamIndicators{
	TO_HEX_IND					= 0,
	SAVING_IN_MEM_IND		= 1,
	BLOCK_READ 					= 	3,
};

enum AnalyzedDataType{
	NO_DATA,
	PERFORM_DATA,
	PARAM_DATA,
	ALARM_DATA,
	TIME_DATA,
	IMPULSE_DATA,
	DRIVE_DATA,
	INFO_DATA,
	FREQ_DATA,
	MODE_DATA,
	RESPONSE_DATA,
	IO_DATA,
};

typedef struct {
	uint32_t freq;
	int32_t length;
	uint32_t total;
	uint32_t accTime;
	uint32_t koef;
	uint32_t only_servo_f;
	uint32_t cutActTime;
	uint32_t cutDeactTime;
	uint32_t fixActTime;
} ServoParams;

typedef struct {
	uint8_t dataBaseNum;
	uint8_t dataBase;
	uint8_t requestType;
	uint8_t dataType;
	uint16_t cmd;
	uint16_t dataNum;
	int32_t data;
} ServoTXMessage;

typedef struct {
	uint8_t resp;
	uint8_t posPoint;
	uint8_t dataType;
	uint8_t requestType;
	int32_t data;
} ServoRXMessage;


enum ErrCodeSymb{

	// err conection
	RESP_ODD_ERR 			= 2,	 // 'B',
	RESP_CRC_ERR 			= 3,	 // 'C',
	RESP_SIGT_ERR 			= 4,	// 'D',
	RESP_COMMAND_ERR = 5, 	// 'E',
	RESP_DATA_ERR 			= 6,	 // 'F',
	RESP_TIMEOUT_ERR 		= 7,	// 'G'

	// err servo
	AL_OK 							= 0xFF,
	AL_8A_SERIAL 				= 0x8A,
	AL_8E_SERIAL 				= 0x8E,
	AL_10_MIN_VOLT   		= 0x10,
	AL_12_MEM					= 0x12,
	AL_13_TEMP_ERR			= 0x13,
	AL_15_MEM					= 0x15,
	AL_16_ENC					= 0x16,
	AL_17_BOARD				= 0x17,
	AL_19_MEM					= 0x19,
	AL_1A_CHOICE_ENG	 	= 0x1A,
	AL_20_ENCODER			= 0x20,
	AL_24_GROUND			= 0x24,
	AL_25_ABS_POS			= 0x25,
	AL_30_OVERLOAD 		= 0x30,
	AL_31_FREEQ				= 0x31,
	AL_32_CURRENT			= 0x32,
	AL_33_OVER_VOLT		= 0x33,
	AL_37_PARAM				= 0x37,
	AL_45_OVERTEMP 		= 0x45,
	AL_46_TEMP_SERVO		= 0x45,
	AL_50_OVERLOAD 		= 0x50,
	AL_51_OVERLOAD 		= 0x51,
	AL_52_POS_ERR  	 		= 0x52,
	AL_E6_EMERG				= 0xE6,
};

enum ErrCodeIndx{
	RESP_CMD_ERR_INDX,
	AL_SERIAL_INDX,
	AL_10_MIN_VOLT_INDX,
	AL_12_MEM_INDX,
	AL_13_TEMP_ERR_INDX,
	AL_15_MEM_INDX	,
	AL_16_ENC_INDX,
	AL_17_BOARD_INDX,
	AL_19_MEM_INDX,
	AL_20_ENCODER_INDX,
	AL_24_GROUND_INDX,
	AL_25_ABS_POS_INDX,
	AL_30_OVERLOAD_INDX,
	AL_32_CURRENT_INDX,
	AL_33_OVER_VOLT_INDX,
	AL_37_PARAM_INDX,
	AL_45_OVERTEMP_INDX,
	AL_50_OVERLOAD_INDX,
	AL_52_DESYNCHR_INDX,
	LOST_CONNECTION_INDX,
	RESP_DATA_ERR_INDX,
	OTHER_ERR_INDX,
	STOP_EMG_INDX,
	OIL_POMP_STOPPED_IND
};

#define INTERNAL_ERR_MASK 			\
		((1<<RESP_DATA_ERR_INDX)			\
		|(1<<LOST_CONNECTION_INDX)		\
		|(1<<RESP_CMD_ERR_INDX))

enum ResponseCode {
	RESP_OK,			 				// 'A',

	RESP_ODD_ERR_CODE = 'B',
	RESP_CRC_ERR_CODE = 'C',
	RESP_SIGT_ERR_CODE,
};

#define isFailureSend(_c) ( (_c) == RESP_CRC_ERR || (_c) == RESP_ODD_ERR )

#define isResponseOk(r) \
	( (r)== RESP_OK )

#define isAlarm(a)	\
	( (a)>='a' && (a)<='f' )

#define getResponseStatus( _r )	\
	( isAlarm(_r)	? (_r) -'a' : (_r) - 'A')

//-------------------------------------------- Read command ---------------------------------------------

enum GetIndicationCMD{
	CMD_GET_STATE										= 0x01,

	FEEDBACK_IMPULSES         						= 0x80, // імпульси зворотного зв'язку
	ENGINE_SPEED          								= 0x81, // частота обертання двигуна
	MISMATCH          										= 0x82, // неузгодження (в iмпульсах)
	PULSE_SETPOINT          							= 0x83, // задане значення імпульсу
	SETPOINT_FREQ          								= 0x84, // частота заданого значення
	FREQ_VAL          										= 0x85, // значення аналогового задання частоти
	TORQUE_VAL          									= 0x86, // значення аналогового задання крут моменту
	BRAKE_CHAIN_LOAD           					= 0x87, // завантаженість гальмівного ланцюга
	EFFICIENT_LOAD_VALUE          				= 0x88, // ефективне значення навантаження
	MAXIMUM_LOAD          							= 0x89, // максимальне значення навантаження
	ACTUAL_TORQUE          							= 0x8A, // фактичний крутний момент
	ABS_POS_PER_ROTATION          				= 0x8B, // абсолютна позиція за один оборот
	ABSOLUTE_COUNTER          					= 0x8C, // лічильник абсолютного положення
	MASS_INERTIA_MOMENT_RATIO          	= 0x8D, // співвідношення моментів інерції маси
	INTERMEDIATE_CIRCUIT_VOLTAGE      	= 0x8E, // напруга проміжного контуру
};



enum GetParamCMD{
	CMD_GET_PARAMS		 							= 5,
	/*
	 * data numb: 0x0 ... 0x54
	 */
};

enum IO_CMD{
	CMD_GET_IO_SIGNALS 							= 0x12,

	GET_IN_IO         											= 0x40,
	GET_OUT_IO         										= 0xC0,
};

enum GetAlarmCMD{
	CMD_GET_AL 											= 0x33,

	GET_CUR_AL         										= 0x10,
	GET_AL_1           											= 0x11,
	GET_AL_2            										= 0x12,
	GET_AL_3           											= 0x13,

	GET_CUR_AL_TIME           							= 0x20,
	GET_AL_1_TIME              							= 0x21,
	GET_AL_2_TIME           								= 0x22,
	GET_AL_3_TIME               							= 0x23,
};

enum GetIOtherCMD{
	CMD_GET_OTHER										= 0x02,

	GET_ALARM												= 0x00,
	GET_ENC_ABS_POS					  				= 0x90,
	GET_ENC_SETED						 				= 0x91,
};


//------------------------------------------- Write command ------------------------------------------------
enum ResetIndCMD{
	CMD_RST_IMP_FEEDBEAK 						= 0x81,

	RST_IMP_FEEDBEAK          						= 0x00,
};

enum SetParamCMD{
	CMD_SET_PARAM 									= 0x84,
	/*
	 * data numb: 0x0 ... 0x54 = param
	 */
};

enum ClearAlarmCMD{
	CMD_CLEAR_AL 									= 0x82,

	CLEAR_CUR_AL 										= 0x00,
	CLEAR_LIST_AL           							= 0x20,
};

enum SelectTestModeCMD{
	/*
	 * 1) block input signal
	 * 2) set test mode
	 * 3) stop
	 */
	CMD_SET_WORKING_MODE 			 	= 0x8B,

	SET_MODE           									= 0x00,

	SET_STOP_TEST 	    		 					= 0x0000,
	SET_JOG 												= 0x0001,
	SET_SERVO 					 						= 0x0002,
	DATA_O_ENGINE 		     						= 0x0003,
	SET_OUT_SIGNAL 		 							= 0x0004,
};


enum BlockInputSIgnalCMD{
	/*
	 * block digital in, analog in,
	 * without EMG, LSP, LSN
	*/
	CMD_BLOCK_IO       							= 0x90,

	BLOCK_DI_AI          							= 0x00,
	DEBLOCK_DI_AI    								= 0x10,
	DEBLOCK_DO 									= 0x13
};

enum SetWorkParam{
	 CMD_SET_WORK_PARAM 				= 0xA0,

	 SET_FREQUENCY          					= 0x10,		// data: hex 4byte
	 SET_BREAK          							= 0x15,
	 SET_STOP         								= 0x12,
	 SET_ACCELERATION_TIME          		= 0x11,		// data: hex 8 byte
	 SET_PATH_LENGTH         					= 0x13,
};

typedef enum DriveCMD{
	CMD_DRIVE                						= 0x92,

	DRIVE         											= 0x0,

	JOG_DIRECT					 					= 0x00000807,
	JOG_REVERSE				 					= 0x00001007,
	JOG_STOP					 						= 0x00000007,
	SERVO_SON_ON 				   				= 0x00000001,
	SERVO_STOP	 									= 0x00000006,
	SERVO_SON_LSN_LSP_ON				= 0x00000007,
}direction_t;

extern bool alarm;
extern ServoParams servoParams;
extern osMessageQueueId_t rxMesQueue, txMesQueue;

uint8_t getAlarmIndx( const uint8_t code );
bool getState( const uint8_t indx );
void initServo();
uint16_t getIndicationCode( const uint8_t indx );
uint32_t calculateWorkingTime();
uint8_t sendServoData( uint8_t _cmd,
		uint8_t	_dataNum,
		uint8_t _dataBaseNum,
		uint32_t	_data,
		uint8_t	 _dataBase,
		uint8_t	 _requestType,
		uint8_t	 _dataType );

#define resetFeedback()	\
sendServoData(																\
			CMD_RST_IMP_FEEDBEAK,									\
			RST_IMP_FEEDBEAK,												\
			BASE_HEX,															\
			DEFAULT_VAL,														\
			BASE_HEX,															\
			SET_DATA,															\
			PERFORM_DATA )

#define setWorkMode( operation )	\
sendServoData(																\
			CMD_SET_WORKING_MODE,								\
			SET_MODE,															\
			BASE_HEX,															\
			(operation),															\
			BASE_HEX,															\
			SET_DATA,															\
			MODE_DATA )

#define setDriveCMD( operation )		\
 sendServoData(																\
			CMD_DRIVE,															\
			DRIVE,																	\
			BASE_HEX,															\
			(operation),															\
			BASE_HEX,															\
			SET_DATA,															\
			DRIVE_DATA )

#define getParam( param )											\
 sendServoData(																\
			CMD_GET_PARAMS,												\
			(param),																	\
			BASE_HEX,															\
			DEFAULT_VAL,														\
			BASE_HEX,															\
			GET_DATA,															\
			PARAM_DATA )

#define resetAlarm()													\
 sendServoData(																\
			CMD_CLEAR_AL,													\
			CLEAR_CUR_AL,													\
			BASE_HEX,															\
			DEFAULT_VAL,														\
			BASE_HEX,															\
			SET_DATA,															\
			PERFORM_DATA )

#define setParam( param, val )										\
 sendServoData(																\
			CMD_SET_PARAM,												\
			(param),																\
			BASE_HEX,															\
			(val)+30000000,													\
			BASE_DEC,															\
			SET_DATA,															\
			PARAM_DATA )

#define setWorkParam( cmd, data, data_type )	\
	 sendServoData(																\
				CMD_SET_WORK_PARAM,									\
				(cmd),																	\
				BASE_HEX,															\
				(data),																	\
				BASE_HEX,															\
				SET_DATA,															\
				(data_type) )

#define servoSetLength(imp)	\
		setWorkParam( SET_PATH_LENGTH, (imp), IMPULSE_DATA )


#define setAccTimeParam(time)											\
		setWorkParam( SET_ACCELERATION_TIME, (time), TIME_DATA )


#define setFreqParam(freq)													\
		setWorkParam( SET_FREQUENCY, (freq), FREQ_DATA )


#define getEncoderInfo( dataNum )									\
sendServoData(																	\
		CMD_GET_OTHER,														\
			(dataNum),																\
			BASE_HEX,																\
			DEFAULT_VAL,															\
			BASE_HEX,																\
			GET_DATA,																\
			IMPULSE_DATA )

#define getStateInfo( data_id )												\
sendServoData(																	\
			CMD_GET_STATE,													\
			(data_id),																	\
			BASE_HEX,																\
			DEFAULT_VAL,														\
			BASE_HEX,																\
			GET_DATA,																\
			INFO_DATA )

#define getFeedbackImp()										\
		getStateInfo( FEEDBACK_IMPULSES );

#define getAlarm()															\
sendServoData(																	\
			CMD_GET_OTHER,													\
			GET_ALARM,															\
			BASE_HEX,																\
			DEFAULT_VAL,														\
			BASE_HEX,																\
			GET_DATA,																\
			ALARM_DATA )

#define getReadyStatus()												\
sendServoData(																	\
		CMD_GET_IO_SIGNALS,											\
			GET_OUT_IO,															\
			BASE_HEX,																\
			DEFAULT_VAL,														\
			BASE_HEX,																\
			GET_DATA,																\
			IO_DATA )

#define blockIO( operation )											\
sendServoData(																	\
			CMD_BLOCK_IO,													\
			(operation),															\
			BASE_HEX,																\
			DEFAULT_VAL,														\
			BASE_HEX,																\
			SET_DATA,																\
			PERFORM_DATA )

#define isTxQueueEmpty()	\
	(osMessageQueueGetCount(txMesQueue) == 0)

#define isRxQueueEmpty()	\
	(osMessageQueueGetCount(rxMesQueue) == 0)

#define servoStart()				setDriveCMD( SERVO_SON_LSN_LSP_ON )

#define servoPause()	setWorkParam( SET_BREAK, DEFAULT_VAL, PERFORM_DATA )
//setDriveCMD( SERVO_STOP )

//setWorkParam( SET_BREAK, DEFAULT_VAL, PERFORM_DATA )
//

#define servoOFF()					setWorkMode( SET_STOP_TEST )

#define jogStartDirect()			setDriveCMD( JOG_DIRECT )
#define jogStartReverse()		setDriveCMD( JOG_REVERSE )

#define jogStop()					setDriveCMD( JOG_STOP )

#define jogOFF()						setWorkMode( SET_STOP_TEST )

#endif


