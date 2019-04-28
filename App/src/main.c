
#include "app.h"
#include "hw.h"

#include "radio.h"
#include "timeServer.h"
#include "low_power_manager.h"
#include "vcom.h"


#include "radionet_cmd_api.h"
#include "dsp.h"


//TODO: Questions
//What floating point format to use for FFT? q15


//TODO: Tasks
	//join - timesync
	//join - joinInterval apply from config
	//fft  - try real fft
	//move fft to separate file

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
	//set TIM2 CNT overflow -- reset pin every 1 ms	==> 0.5khz
	TIM2->ARR = 1000;

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


int radioNetErrors = 0;

int main(){


    HAL_Init( );

    //differs
    SystemClock_Config();

    PWM_Init();
    HW_Init( );
    TMP75_Init();
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
    PRINTF("Starting message loop...\r\n");

    //sendStatusInfo();


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
	//TODO: create config init method
	//set uid to cfg structure
	loraNode_cfg.radionet.uId = CFG_UID;

	radioprot_join_req_t joinRequestPacket = {
			.hdr.cmd = RADIOPROT_CMD_JOINREPLY << 4,
			.hdr.sId = loraNode_cfg.radionet.sId,
			.uId = loraNode_cfg.radionet.uId,
			.time = 0x0f0e0d0c,
			.fwver = loraNode_cfg.fwver

	};
	loraNode_cfg.radionet.joinPending = true;
	Radio.Send((uint8_t*)&joinRequestPacket, sizeof(joinRequestPacket));
	PRINTF("INFO: Sending join request... size: %d \r\n", sizeof(joinRequestPacket));
}


//TODO: delta
//TODO: low power mode beside ADC???
void sendStatusInfo(){

	/* Measure temperature, battery level */
	HW_ADC_Init();
	uint8_t battery = HW_GetBatteryLevel();
	HW_ADC_DeInit();


	/* Get settings from config */
	uint16_t configurableSamplesNum [] = {64, 128, 512, 1024, 2048};
	uint16_t N = configurableSamplesNum[loraNode_cfg.adc_fft.fftSamplesNum];

	uint8_t fftPeaksNum = loraNode_cfg.adc_fft.fftPeaksNum;
	float32_t thresholdVoltage = loraNode_cfg.dsp.thresholdVoltage;
	uint8_t	 kurtosisTrimmedSamples = loraNode_cfg.dsp.kurtosisTrimmedSamples;


	/* Sample acc signal by ADC */
	uint16_t *signal = (uint16_t*) malloc(N * sizeof(uint16_t));

	HW_ADC_Init_Con();
	HW_ADC_Read_Continuous(signal, N);


	uint16_t temperature = TMP75_GetTemperature();
	while (!isConversionFinished());

	/* Start low power mode */
	HW_ADC_DeInit();


	/* Calculate signal processing */

	float32_t rmsDC, rmsAC, vpp, kurtosisRatio;
	uint8_t ringdownCounts;
	uint16_t riseTime, thresholdDuration, vppIndex, firstThresholdCrossingIndex;

		/* Calculate RMS, VPP, ringdown counts, rise time, threshold duration  and kurtosis*/
	calculateRMSofSignal(signal, N, &rmsDC, &rmsAC);
	calculatePeakofSignal(signal, N, &vpp, &vppIndex);
	calculateTimeDomainAnalysis(signal, N, thresholdVoltage, &firstThresholdCrossingIndex, &ringdownCounts, &thresholdDuration);
	calculateRiseTimeofSignal(vppIndex, firstThresholdCrossingIndex, &riseTime);
	calculateKurtosisRatioofSignal(signal, N, kurtosisTrimmedSamples, &kurtosisRatio);
	PRINTF("DEBUG: RMS - DC %.3f AC %.3f, VPP - %.3f , kurtosis ratio - %.3f, \r\n ringdown counts - %u, riseTime %u, thresholdDuration %u for N %u samples \r\n", rmsDC, rmsAC, vpp, kurtosisRatio, ringdownCounts, riseTime, thresholdDuration, N);



		/* Calculate FFT */
	float32_t *fftBuffer = (float32_t*) malloc(N * 2 * sizeof(float32_t));
	calculateFFT(signal, N, fftBuffer,loraNode_cfg.adc_fft.ifftFlag, loraNode_cfg.adc_fft.doBitReverse);
	free(signal);

		/* Extract Peaks */
	uint16_t *fftPeaksIndexes = (uint16_t*) malloc(fftPeaksNum * sizeof(uint16_t));
	float32_t *fftPeaksValues = (float32_t*) malloc(fftPeaksNum * sizeof(float32_t));
	PRINTF("DEBUG: FFT calculation for N %u samples, max indexes:\n\r", N);
    findFFTPeaks(fftBuffer, N, fftPeaksValues, fftPeaksIndexes, fftPeaksNum, 3.0, 0);
    for (int i=0; i < fftPeaksNum; i++)
  	    PRINTF("    FFT peak: %0.3f %u\r\n", fftPeaksValues[i], fftPeaksIndexes[i]);

    free(fftBuffer);



	/* Send LoRa packet */
    int packetSize = sizeof(radioprot_status_info_t) + (sizeof(uint16_t) + sizeof(float32_t)) * fftPeaksNum;
	radioprot_status_info_t *statusinfoPacket = (radioprot_status_info_t*)malloc(packetSize);
	statusinfoPacket->hdr.cmd = RADIOPROT_CMD_STATUS << 4;
	statusinfoPacket->hdr.sId = loraNode_cfg.radionet.sId;
	statusinfoPacket->battery = battery;
	statusinfoPacket->temperature = temperature;
	statusinfoPacket->rms = rmsAC * 4095 / 3.3;
	statusinfoPacket->vpp = vpp * 4095 / 3.3;
	statusinfoPacket->kurtosisRatio = kurtosisRatio;
	statusinfoPacket->ringdownCounts = ringdownCounts;
	statusinfoPacket->riseTime = riseTime;
	statusinfoPacket->thresholdDuration = thresholdDuration;
	statusinfoPacket->fftPeaksNum = fftPeaksNum;

	uint16_t peakIndexesSize = fftPeaksNum * sizeof(uint16_t);
	memcpy(statusinfoPacket->fftPeaksInfo, (uint8_t*) fftPeaksIndexes, peakIndexesSize);
	memcpy(statusinfoPacket->fftPeaksInfo + peakIndexesSize, (uint8_t*)fftPeaksValues, fftPeaksNum * sizeof(float32_t));
	PRINTF("INFO: Sending STATUS INFO packet (s. sizeof: %u, %u) temperature: %u battery: %u, rms: %u, vpp: %u\r\n",
			sizeof(radioprot_status_info_t), packetSize, statusinfoPacket->temperature, statusinfoPacket->battery, statusinfoPacket->rms, statusinfoPacket->vpp);
	Radio.Send((uint8_t*)statusinfoPacket, packetSize);

	/* Free memory */
	free(fftPeaksValues);
	free(fftPeaksIndexes);
	free(statusinfoPacket);



}



void OnTxDone( void )
{
    Radio.Sleep();
    PRINTF("OnTxDone\n\n\r");
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
		.hdr.sId = loraNode_cfg.radionet.sId,
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

