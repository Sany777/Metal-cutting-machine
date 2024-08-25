#include "servo_amplifier.h"

#include "main.h"
#include "servo_tools.h"
#include "servo_control.h"

bool alarm;
ServoParams servoParams;
osMessageQueueId_t rxMesQueue, txMesQueue;
const osThreadAttr_t ServoMessenger_attributes = {
  .name =  "",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

static bool parceResponseData( uint8_t *data, ServoRXMessage*rxMes, uint8_t dataType );
static void ServoMessenger( void *argument );
static uint8_t getDataSize( uint16_t dataType );


static bool parceResponseData( uint8_t *servo_rxbuffer, ServoRXMessage*rxMes, uint8_t dataType )
{
	bool send = false;
	switch(dataType){
		case PARAM_DATA:
		{
			/*  [flag|pos|data][ETX:1][CRC:2] */
			uint8_t flags = getCharValue( servo_rxbuffer[0], BASE_HEX );
			if( ! ( flags &(1<<BLOCK_READ) ) ){
				rxMes->posPoint = servo_rxbuffer[1];
				rxMes->data = getNumberFromString(
							&servo_rxbuffer[2],
							 6,
							 flags &(1<<TO_HEX_IND)
							 ? BASE_HEX
							 : BASE_DEC );
				send = true;
			}
			break;
		}
		case TIME_DATA:
		case IMPULSE_DATA:
		{
			rxMes->data = getNumberFromString( servo_rxbuffer, 8, BASE_HEX );
			send = true;
			break;
		}
		case INFO_DATA:
		{
			/*  [data: 0|0|point|hexflag|data][ETX:1][CRC:2] */
			uint8_t dec_flag 	= getCharValue( servo_rxbuffer[3], BASE_DEC );
			rxMes->posPoint = getCharValue( servo_rxbuffer[2], BASE_DEC );
			rxMes->data 		= getNumberFromString( &servo_rxbuffer[4], 8, dec_flag ? BASE_DEC : BASE_HEX );
			send = true;
			break;
		}
		case ALARM_DATA:
		{
			rxMes->data = getNumberFromString( servo_rxbuffer, 4, BASE_HEX );
			send = rxMes->data != AL_OK;
			break;
		}
		default: break;
	}
	return send;
}

void initServo()
{
	 txMesQueue = osMessageQueueNew( SIZE_CMD_QUEUE, sizeof( ServoTXMessage ), NULL );
	 rxMesQueue = osMessageQueueNew(SIZE_RX_QUEUE, sizeof( ServoRXMessage ), NULL );
	 if( txMesQueue == NULL
			 || rxMesQueue == NULL
			 || osThreadNew( ServoMessenger, NULL, &ServoMessenger_attributes) == NULL ){
		 Error_Handler();
	 }
	alarm =  false;
}

static void ServoMessenger( void *argument )
{
	volatile uint16_t req_size, resp_size, data_size;
	ServoTXMessage txMes = {0};
	ServoRXMessage rxMes = {0};
	uint8_t servo_txbuffer[ SERVO_BUF_SIZE ] = { SOH , SERVO, 0,0, STX };
	uint8_t servo_rxbuffer[ SERVO_BUF_SIZE ];
	uint8_t response;
	uint16_t crc;
	int atempt;
	osDelay( DELAY_START_TASK );
	for(;;)
	{
		osDelay( 150 );
		osMessageQueueGet( txMesQueue, &txMes, NULL, osWaitForever );

		// set cmd data
		numToString( txMes.cmd,
										&servo_txbuffer[TX_POS_CMD],
										SIZE_CMD,
										BASE_HEX );

		// set data num
		numToString( txMes.dataNum,
										&servo_txbuffer[TX_POS_DATA_NUM],
										SIZE_DATA_NUM,
										txMes.dataBaseNum );

		// set size request and response
		if( txMes.requestType == GET_DATA ){
			req_size = REQ_HEADER_SIZE;
			resp_size = RESP_HEADER_SIZE + getDataSize( txMes.dataType );
		} else {
			resp_size = RESP_HEADER_SIZE;
			data_size = getDataSize( txMes.dataType );
			req_size 	= REQ_HEADER_SIZE + data_size;
			numToString( txMes.data,
										&servo_txbuffer[TX_POS_DATA],
										data_size,
										txMes.dataBase );
		}

		// set end data symb
		servo_txbuffer[req_size++] = ETX;
		uint8_t *crc_area = &servo_txbuffer[START_CRC_AREA];

		// calculation crc data
		crc = ETX;
			while( *crc_area != ETX ) crc += *( crc_area++ );
		numToString( crc,
								&servo_txbuffer[req_size],
								SIZE_CRC,
								BASE_HEX );
		req_size += SIZE_CRC;
		atempt = ATEMPTS;
		do{

			HAL_UART_Transmit( huartServo,
												servo_txbuffer,
												req_size,
												TIMEOUT_SEND_MES );

			if( HAL_UART_Receive( huartServo,
													servo_rxbuffer,
													resp_size,
													TIMEOUT_WAIT_MES ) == HAL_OK ) {
				response = getResponseStatus( servo_rxbuffer[RX_POS_RESP] );
				if( !isFailureSend( response ) ){
					break;
				}
			} else {
				response = RESP_TIMEOUT_ERR;
			}
			if( atempt ){
				HAL_UART_Transmit( huartServo, (uint8_t*)EOT, 1, TIMEOUT_SEND_MES );
				HAL_UART_AbortReceive( huartServo );
				osDelay( DELAY_SEND_EOT );
			}
		} while( atempt-- );

		if( !alarm &&  isAlarm( servo_rxbuffer[RX_POS_RESP] ) ){
			getAlarm();
		}

		alarm = isAlarm( servo_rxbuffer[RX_POS_RESP] );

		rxMes.resp = response;

		if( response != RESP_OK
				|| ( txMes.requestType == GET_DATA
				&& parceResponseData(
						&servo_rxbuffer[RX_POS_DATA],
						&rxMes, txMes.dataType )) ){
			rxMes.requestType = txMes.requestType;
			rxMes.dataType = txMes.dataType;
			osMessageQueuePut( rxMesQueue, &rxMes, 0, 1000 );
		}
	}
}


static uint8_t getDataSize( uint16_t dataType )
{
	switch(dataType){
		case DRIVE_DATA:
		case PARAM_DATA:
		case TIME_DATA:
		case IMPULSE_DATA:
		case IO_DATA:
		{
			return 8;
		}
		case INFO_DATA:
		{
			return 12;
		}
		case FREQ_DATA:
		case ALARM_DATA:
		case PERFORM_DATA:
		case MODE_DATA:
		{
			return 4 ;
		}
		default: break;
	}
	return  0;
}

uint8_t getAlarmIndx( const uint8_t code )
{
	switch(code){
	// connection
		case RESP_TIMEOUT_ERR: return LOST_CONNECTION_INDX;
		case RESP_COMMAND_ERR: return RESP_CMD_ERR_INDX;
		case RESP_SIGT_ERR:
		case RESP_DATA_ERR: return RESP_DATA_ERR_INDX;
		case RESP_ODD_ERR:
		case RESP_DATA_ERR_INDX:
	// device
		case AL_8A_SERIAL:
		case AL_8E_SERIAL: return  AL_SERIAL_INDX;
		case AL_10_MIN_VOLT: return  AL_10_MIN_VOLT_INDX;
		case AL_12_MEM: return  AL_12_MEM_INDX;
		case AL_13_TEMP_ERR: return  AL_13_TEMP_ERR_INDX;
		case AL_15_MEM: return  AL_15_MEM_INDX;
		case AL_16_ENC: return  AL_16_ENC_INDX;
		case AL_17_BOARD: return  AL_17_BOARD_INDX;
		case AL_19_MEM: return  AL_19_MEM_INDX;
		case AL_20_ENCODER: return  AL_20_ENCODER_INDX;
		case AL_24_GROUND: return AL_24_GROUND_INDX;
		case AL_25_ABS_POS: return AL_25_ABS_POS_INDX;
		case AL_30_OVERLOAD: return  AL_30_OVERLOAD_INDX;
		case AL_32_CURRENT: return  AL_32_CURRENT_INDX;
		case AL_33_OVER_VOLT: return  AL_33_OVER_VOLT_INDX;
		case AL_37_PARAM: return  AL_37_PARAM_INDX;
		case AL_45_OVERTEMP: return  AL_45_OVERTEMP_INDX;
		case AL_51_OVERLOAD:
		case AL_50_OVERLOAD: return  AL_50_OVERLOAD_INDX;
		case AL_52_POS_ERR: return AL_52_DESYNCHR_INDX;
		case AL_E6_EMERG: return STOP_EMG_INDX;
		default: break;
	}
	return OTHER_ERR_INDX;
}

bool getState( const uint8_t indx )
{
	switch( indx ){
		case 0: return getStateInfo( FEEDBACK_IMPULSES );
		case 1: return getStateInfo( ENGINE_SPEED );
		case 2: return getStateInfo( MISMATCH );
		case 3: return getStateInfo( PULSE_SETPOINT );
		case 4: return getStateInfo( SETPOINT_FREQ );
		case 5: return getStateInfo( FREQ_VAL );
		case 6: return getStateInfo( TORQUE_VAL );
		case 7: return getStateInfo( BRAKE_CHAIN_LOAD );
		case 8: return getStateInfo( EFFICIENT_LOAD_VALUE );
		case 9: return getStateInfo( MAXIMUM_LOAD);
		case 10: return getStateInfo( ACTUAL_TORQUE);
		case 11: return getStateInfo( ABS_POS_PER_ROTATION );
		case 12: return getStateInfo( ABSOLUTE_COUNTER );
		case 13: return getStateInfo( MASS_INERTIA_MOMENT_RATIO);
		case 14: return getStateInfo( INTERMEDIATE_CIRCUIT_VOLTAGE);
		default: break;
	}
	return false;
}

uint8_t sendServoData( uint8_t _cmd,
		uint8_t	_dataNum,
		uint8_t _dataBaseNum,
		uint32_t	_data,
		uint8_t	 _dataBase,
		uint8_t	 _requestType,
		uint8_t	 _dataType )
{
	ServoTXMessage txMes = {
			.cmd = _cmd,
			.dataNum = _dataNum,
			.dataBaseNum = _dataBaseNum,
			.data = _data,
			.dataBase = _dataBase,
			.dataType = _dataType,
			.requestType = _requestType,
	};
	return osMessageQueuePut( txMesQueue, &txMes, 0, 1000 ) == osOK;
}
