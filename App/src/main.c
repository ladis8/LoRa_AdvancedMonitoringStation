#include "hw.h"
#include "app.h"

#include "radio.h"
#include "timeServer.h"
#include "low_power_manager.h"
#include "vcom.h"

#include "radionet_cmd_api.h"


//TODO: Questions
//What floating point format to use for FFT? q15


//TODO: Tasks
	//join - timesync
	//join - joinInterval apply from config
	//fft  - try real fft

#define DEBUG 1

#define BUFFER_SIZE                  32 // Define the payload size here
#define LED_PERIOD_MS               200



/*#define LEDS_OFF   do{ \
                   LED_Off( LED_BLUE ) ;   \
                   LED_Off( LED_RED ) ;    \
                   LED_Off( LED_GREEN1 ) ; \
                   LED_Off( LED_GREEN2 ) ; \
                   } while(0) ;
*/





uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];
uint8_t fftBuffer[256];
int8_t RssiValue = 0;
int8_t SnrValue = 0;

int seqnum;
int numChunks;




/* Led Timers objects*/
//static  TimerEvent_t timerLed;

static TimerEvent_t timerStatus;
static RadioEvents_t RadioEvents;




void OnTxDone( void );
void FFT_OnTxDone(void);
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxTimeout( void );
void OnRxTimeout( void );
void OnRxError( void );
void OnStatusinfoTimerDone(void *context);


void sendStatusInfo();
void sendJoinRequest();




void PWM_Init(void)
{
	GPIO_InitTypeDef initStruct={0};
	initStruct.Mode =GPIO_MODE_AF_PP;
	initStruct.Pull =GPIO_PULLUP;
	initStruct.Speed = GPIO_SPEED_HIGH;
	initStruct.Alternate= PWM_AF ;
	HW_GPIO_Init( PWM_GPIOPORT, PWM_PIN, &initStruct);


	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	//set TIM2 prescaler to 1MHz clock
	TIM2->PSC = 31;
	//set TIM2 CNT overflow -- reset pin every 0.1 ms	== 10khz
	TIM2->ARR = 100;

	//enable mux pins TIM2_CH2 and TIM2_CH3 in output compare mode
	TIM2->CCER |= TIM_CCER_CC2E;
	//set the compare level
	TIM2-> CCR2 = 0;

	//set OC1M to 011 to toggle
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM2->CCMR1 |= (TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1);
	TIM2->CR1 |= TIM_CR1_CEN;

}



void radioConfigure(void){
    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel(loraNode_cfg.lora.frequency);
    Radio.SetPublicNetwork(true);


    Radio.SetTxConfig(MODEM_LORA,
    		CFG_TX_OUTPUT_POWER, 0,
    		loraNode_cfg.lora.bw,
    		loraNode_cfg.lora.sf,
			loraNode_cfg.lora.cr,
			CFG_LORA_PREAMBLE_LENGTH,
			CFG_LORA_FIX_LENGTH_PAYLOAD_ON,
            true, 0, 0,
			CFG_LORA_IQ_INVERSION_ON,
			loraNode_cfg.lora.txTimeout);


    //TODO: Disable RX continuous for better power consumption
    Radio.SetRxConfig( MODEM_LORA,
    		loraNode_cfg.lora.bw,
    		loraNode_cfg.lora.sf,
    		loraNode_cfg.lora.cr,
			0,
			CFG_LORA_PREAMBLE_LENGTH,
            CFG_LORA_SYMBOL_TIMEOUT,
			CFG_LORA_FIX_LENGTH_PAYLOAD_ON,
            0, true, 0, 0,
			CFG_LORA_IQ_INVERSION_ON,
			true );
}

uint16_t calculateMean(uint16_t *values, uint16_t length){
	uint32_t sum = 0;
	for (int i = 0; i < length; i++){
		sum += values[i];
	}
	return (uint16_t)(sum/length);
}

//TODO: complete N samples in rpi
void calculateRMS(uint16_t *ADCReadings, float32_t *rms, float32_t *rms_meanRemoved){

	uint16_t N = loraNode_cfg.adc_fft.fftSamples;
	N = 1024;

	float32_t *voltageBuffer = (float32_t*) malloc(N * sizeof(float32_t));

	uint16_t mean = calculateMean(ADCReadings,(uint32_t)N);


	for (int i=0; i < N; i++){
		voltageBuffer[i] = ((float32_t)ADCReadings[i])/4095 * 3.3;
	}
	arm_rms_f32(voltageBuffer, N, rms);

	for (int i=0; i < N; i++){
		voltageBuffer[i]  = ((float32_t)ADCReadings[i]-mean)/4095 * 3.3;
	}
	arm_rms_f32(voltageBuffer, N, rms_meanRemoved);
	free(voltageBuffer);
}

void calculateFFT(uint16_t *ADCReadings,uint16_t *fftPeaksIndexes, uint8_t fftPeaksNum){
    /* Process the data through the CFFT/CIFFT module */
   //arm_status status = ARM_MATH_SUCCESS;
   //arm_cfft_f32(&arm_cfft_sR_f32_len1024, testInput_f32_10khz, ifftFlag, doBitReverse);
     /* Process the data through the Complex Magnitude Module for
     calculating the magnitude at each bin */
   //arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);
     /* Calculates maxValue and returns corresponding BIN value */
   //arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);
   //if(testIndex !=  refIndex)
       //status = ARM_MATH_TEST_FAILURE;
   //PRINTF("Test index is %d\n", testIndex);
	uint16_t N = loraNode_cfg.adc_fft.fftSamples;
	N = 1024;

	float32_t *fftBuffer = (float32_t*) malloc(N * 2 * sizeof(float32_t));

	for (int i=0; i < N; i++){
   		fftBuffer[2*i] = ((float32_t)ADCReadings[i])/4095 * 3.3;
   		fftBuffer[2*i +1] = 0;
	}

	float32_t maxValue;
	uint32_t maxIndex;
	arm_cfft_f32(&arm_cfft_sR_f32_len1024, fftBuffer, loraNode_cfg.adc_fft.ifftFlag, loraNode_cfg.adc_fft.doBitReverse);
	//arm_rfft_f32(&arm_cfft_sR_f32_len1024, testInput, loraNode_cfg.adc_fft.ifftFlag, loraNode_cfg.adc_fft.doBitReverse);
	arm_cmplx_mag_f32(fftBuffer, fftBuffer, N);
   	arm_max_f32(fftBuffer, N, &maxValue, &maxIndex);

    //TODO: select max indexes from fft
    for (int i = 0; i < fftPeaksNum; i++)
	   fftPeaksIndexes[i] = maxIndex;

    free(fftBuffer);
    PRINTF("DEBUG: FFT calculation - index with maximal amplitude: %d is and maximum value: %.3f\n\r", maxIndex, maxValue);

}




int radioNetErrors = 0;

int main( void ){

    HAL_Init( );

    //differs
    SystemClock_Config();

    PWM_Init();
    HW_Init( );
    HW_ADC_Init();
    radioConfigure();

    /* Initialize timers */
    TimerInit(&timerStatus, OnStatusinfoTimerDone);




    //DBG_Init( );


    /*Disbale Stand-by mode*/
    //LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );

    /* Led Timers*/
    //TimerInit(&timerLed, OnledEvent);
    //TimerSetValue( &timerLed, LED_PERIOD_MS);
    //TimerStart(&timerLed );


    int counter = 0;
    PRINTF("Starting message loop...\r\n	");

    //send join request at first
    loraNode_cfg.radionet.state = TX;

    while( 1 )
    {
        switch(loraNode_cfg.radionet.state)
        {
        case RX:
        	//PRINTF("Radio set on RX with timeout %u\r\n",loraNode_cfg.lora.rxTimeout );
        	Radio.Rx(loraNode_cfg.lora.rxTimeout);
        	loraNode_cfg.radionet.state = LOWPOWER;
            break;

        case TX:
        	//CASE01: if not joined --> keep sending join
        	if (!loraNode_cfg.radionet.nodeJoined){
        		sendJoinRequest();
        	}

        	//CASE02: if joined and in status mode --> send status info
        	else if (loraNode_cfg.radionet.nodeJoined && APPLICATION_MODE == APP_STATUS_MODE){
        		sendStatusInfo();
        		TimerSetValue(&timerStatus,loraNode_cfg.radionet.statusinfoInterval);
        		TimerStart(&timerStatus);
        	}
        	//CASE03 if joined and in debug mode --> send proper info
        	else if (loraNode_cfg.radionet.nodeJoined && APPLICATION_MODE == APP_DEBUG_MODE){
        		PRINTF("ERROR: Not implemented yet\r\n");
        	}

        	loraNode_cfg.radionet.state = LOWPOWER;
            break;

        case RX_TIMEOUT:
        case RX_ERROR:

        	//CASE01: if not joined --> keep sending join
        	if (!loraNode_cfg.radionet.nodeJoined){
        		loraNode_cfg.radionet.state = TX;
                break;
            }
        	//CASE02: if joined and in status mode --> ??

        	//SendStatusInfo(counter);
        	counter++;
        	loraNode_cfg.radionet.state = RX;
            break;

        case LOWPOWER:
        default:
            // Set low power
            break;
        }

        DISABLE_IRQ( );
        //if an interupt has occured after __disable_irq, it is kept pending  and cortex will not enter low power anyway
        if (loraNode_cfg.radionet.state == LOWPOWER)
        {
#ifndef LOW_POWER_DISABLE
          //LPM_EnterLowPower( );
#endif
        }
        ENABLE_IRQ( );
    }
}


void sendJoinRequest(){
	radioprot_join_req_t joinRequestPacket = {
			.hdr.cmd = RADIOPROT_CMD_JOINREPLY << 4,
			.hdr.id = loraNode_cfg.radionet.sId,
			.time = 0x0f0e0d0c,
			.fwver = 0x01,

	};
	loraNode_cfg.radionet.joinPending = true;
	Radio.Send((uint8_t*)&joinRequestPacket, sizeof(joinRequestPacket));
	PRINTF("INFO: Sending join request... size: %d \r\n", sizeof(joinRequestPacket));
}


void sendStatusInfo(){

	/* Measure temperature, battery level */
	uint16_t ADCReadings[1024];

	HW_ADC_Init();
	uint8_t battery = HW_GetBatteryLevel();
	HW_ADC_DeInit();
	HW_ADC_Init_Con();
	HW_ADC_Read_Continuous(ADCReadings, 1024);

	float32_t temperature = TMP75_GetTemperatureFloat();
	while (!isConversionFinished());

	/* Start low power mode */
	HW_ADC_DeInit();
	//pass

	/* Calculate signal processing */
	uint8_t fftPeaksNum = loraNode_cfg.adc_fft.fftPeaksNum;
	uint16_t fftPeaksIndexes [fftPeaksNum];
	float32_t rmsDC, rmsAC;
	calculateRMS(ADCReadings, &rmsDC, &rmsAC);
	calculateFFT(ADCReadings, fftPeaksIndexes, fftPeaksNum);

	/* Send LoRa packet */
	radioprot_status_info_t statusinfoPacket = {
			.hdr.cmd = RADIOPROT_CMD_STATUS << 4,
			.hdr.id = loraNode_cfg.radionet.sId,
			.battery = battery,
			.rms = rmsAC,
			.temperature = temperature,
			.fftPeaksNum = fftPeaksNum,
	};
	memcpy((uint8_t*)statusinfoPacket.fftPeaksIndexes, (void*)fftPeaksIndexes,fftPeaksNum * sizeof(uint16_t));

	PRINTF("INFO: Sending STATUS INFO packet (s. %u) temperature: %.3f, battery: %u, rms: %.3f\r\n\n", sizeof(statusinfoPacket), temperature, battery, rmsAC);
	Radio.Send((uint8_t*)&statusinfoPacket, sizeof(statusinfoPacket));
}



void OnTxDone( void )
{
    Radio.Sleep();
    PRINTF("OnTxDone\n\r");
    if (!loraNode_cfg.radionet.nodeJoined){
    		loraNode_cfg.radionet.joinPending = true;
    		loraNode_cfg.radionet.state = RX;
    }
}

/*void FFT_OnTxDone(void){


	if (seqnum == numChunks){
		PRINTF("FFT spectrum sent...\r\n");
		RadioEvents.TxDone = OnTxDone;
		loraNode_cfg.radionet.state = RX;
		return;
	}
	radioprot_chunk_data_t chunk = {
		.hdr.cmd =  RADIOPROT_CMD_DATA_CHUNKS << 4,
		.hdr.id = loraNode_cfg.radionet.sId,
		.type = RADIOPROT_CHUNK_TYPE_FFT,
		.seq = seqnum,
		.nchunks = numChunks,
		.timestamp = 0,
	};
	memcpy((uint8_t*)chunk.fft.fftChunk, (void*)testOutput + 128 * seqnum, 128);
	Radio.Send((uint8_t*)&chunk, sizeof(chunk));
	seqnum++;
	PRINTF("FFT Packet with id %d was prepared for sending...\r\n", seqnum);
}*/



void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep();
    PRINTF("OnRxDone: ");

    RssiValue = rssi;
    SnrValue = snr;
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );

    PRINTF("RssiValue=%d dBm, SnrValue=%d, bytes received=%d\n\r", rssi, snr, size);
    PRINTF("%s\r\n", payload);
    TRACE_DEBUG("DEBUG: ");
    for (int i=0; i < 20; i++){
    	TRACE_DEBUG("%u ", *(payload+i));
    }
    TRACE_DEBUG("\r\n");


    gwPacketHandler((radioprot_gw_packet_t*)payload, rssi);

}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    PRINTF("OnTxTimeout\n\r");
    loraNode_cfg.radionet.state = TX_TIMEOUT;

}

void OnRxTimeout( void )
{
    Radio.Sleep();
    PRINTF("OnRxTimeout\n\r");
    if (loraNode_cfg.radionet.joinPending){
    	loraNode_cfg.radionet.joinPending = false;
    }
    loraNode_cfg.radionet.state = RX_TIMEOUT;

}

void OnRxError( void )
{
    Radio.Sleep( );
    PRINTF("OnRxError\n\r");
    loraNode_cfg.radionet.state = RX_ERROR;

    if (loraNode_cfg.radionet.joinPending){
        	loraNode_cfg.radionet.joinPending = false;
        	loraNode_cfg.radionet.state = TX;
        }
}

void OnStatusinfoTimerDone(void *context){
	//cause to send status info
	loraNode_cfg.radionet.state = TX;
}

/*static void OnledEvent( void* context )
{
  LED_Toggle( LED_BLUE ) ; 
  LED_Toggle( LED_RED1 ) ; 
  LED_Toggle( LED_RED2 ) ; 
  LED_Toggle( LED_GREEN ) ;   

  TimerStart(&timerLed );
}*/

