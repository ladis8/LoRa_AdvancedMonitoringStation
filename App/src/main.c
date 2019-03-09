

#include <string.h>
#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "low_power_manager.h"
#include "vcom.h"


#include "arm_math.h"
#include "arm_const_structs.h"


//include sensors handlers
#include "sensor_tmp75.h"


#include "app_protocol.h"


//TODO:
//connect ADXL1001
//FFT

//TODO: Questions
//What floating point format to use for FFT? q15



#define RF_FREQUENCY                                868500000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm



#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false



typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

typedef struct{


}loraNode_t;


#define RX_TIMEOUT_VALUE           5000
#define BUFFER_SIZE                  32 // Define the payload size here
#define LED_PERIOD_MS               200


#define LEDS_OFF   do{ \
                   LED_Off( LED_BLUE ) ;   \
                   LED_Off( LED_RED ) ;    \
                   LED_Off( LED_GREEN1 ) ; \
                   LED_Off( LED_GREEN2 ) ; \
                   } while(0) ;



uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];
uint8_t fftBuffer[256];

States_t State;;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

bool joinPending = false;
bool nodeJoined = false;
uint8_t id = 0x08;
int seqnum;
int numChunks;


 /* Led Timers objects*/
static  TimerEvent_t timerLed;
static RadioEvents_t RadioEvents;





void OnTxDone( void );
void FFT_OnTxDone(void);
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxTimeout( void );
void OnRxTimeout( void );
void OnRxError( void );
static void OnledEvent( void* context );

void sendStatusInfo();
void sendJoinRequest();
void calculateFFT();

void gwPacketHandler(radioprot_gw_packet_t *packet, uint8_t rssi);







#define TEST_LENGTH_SAMPLES 2048

/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */
//extern float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
static float32_t testOutput[TEST_LENGTH_SAMPLES/2];
static uint16_t ADCReadings[1024];
static float32_t testInput[2048];

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 213, testIndex = 0;

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



void sendFFTOutput(float32_t *fftOutput, uint8_t *fftBuf, int fftBufferSize){
	for (int i=0; i < 32; i++){
		memcpy((void*)fftBuf, (void*)fftOutput + fftBufferSize * i, fftBufferSize);
		Radio.Send(fftBuffer, fftBufferSize);
		PRINTF("Packet with id %d was successfully sent\r\n", i);
		//HAL_Delay(5000);
	}
}
void sendADCReadings(uint16_t *adcReadings, uint8_t *fftBuf, int fftBufferSize){
	//1024 samples = 2KiB
	//1 packet 64 samples = 128B
	for (int i=0; i < 16; i++){
			memcpy((void*)fftBuf, (void*)adcReadings + fftBufferSize * i, fftBufferSize);
			Radio.Send(fftBuffer, fftBufferSize);
			PRINTF("Packet with id %d was successfully sent\r\n", i);
			HAL_Delay(5000);
		}
}


int main( void ){

    HAL_Init( );

    //differs
    SystemClock_Config();

    PWM_Init();
    HW_Init( );
    HW_ADC_Init_Con( );
    //HW_ADC_Init();
    //takes approximately
    //1ADC cycle is
    HW_ADC_Read_Continuous(ADCReadings, 1024);
    HAL_Delay(1000);
    calculateFFT();

    /*PRINTF("SETUP AND TEST...\r\n");
    bool i2cStatus = scanI2C1();
     if (i2cStatus)
     	PRINTF("Temperature sensor detected...\r\n");

    TMP75_Init();


    int cnt = 0;
    while(cnt < 3){
    	uint16_t temp = TMP75_GetTemperature();
    	PRINTF("Measured temperature is: %d\r\n", temp);
    	uint16_t adc = HW_ADC_ReadChannel(ADC_READ_CHANNEL);
    	PRINTF("Measured ADC value is: %d \r\n", adc);
    	uint8_t battery = HW_GetBatteryLevel();
    	PRINTF("Measured battery level is: %d \r\n", battery);
    	HAL_Delay(500);
    	cnt++;
    }*/


    //DBG_Init( );


    /*Disbale Stand-by mode*/
    //LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );

    /* Led Timers*/
    TimerInit(&timerLed, OnledEvent);   
    TimerSetValue( &timerLed, LED_PERIOD_MS);

    TimerStart(&timerLed );

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetPublicNetwork(true);


    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 10000 );


    //TODO: Disable RX continuous for better power consumption
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );






    int counter = 0;

    State = TX;
    sendJoinRequest();
    //HAL_Delay(3000);
    State = RX;

    //sendFFTOutput(testOutput, fftBuffer, 128);
    //sendADCReadings(ADCReadings, fftBuffer, 128);
    //while(1);

    PRINTF("Starting message loop...");
    while( 1 )
    {
        switch( State )
        {
        case RX:

        	Radio.Rx( RX_TIMEOUT_VALUE );
        	State = LOWPOWER;
            break;

        case TX:
            // Indicates on a LED that we have sent a PING [Master]
            // Indicates on a LED that we have sent a PONG [Slave]
            //GpioWrite(&Led2, GpioRead( &Led2 ) ^ 1 );
            //sendStatusInfo();

            //Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
        	//SendStatusInfo(counter);
        	counter++;
        	Radio.Rx( RX_TIMEOUT_VALUE );
        	State = LOWPOWER;
            break;

        case LOWPOWER:
        default:
            // Set low power
            break;
        }

        DISABLE_IRQ( );
        /* if an interupt has occured after __disable_irq, it is kept pending 
         * and cortex will not enter low power anyway  */
        if (State == LOWPOWER)
        {
#ifndef LOW_POWER_DISABLE
          //LPM_EnterLowPower( );
#endif
        }
        ENABLE_IRQ( );
    }
}

void gwPacketHandler(radioprot_gw_packet_t *packet, uint8_t rssi){

	PRINTF("DEBUG: ");
	for (int i=0; i < 10; i++){
		PRINTF("%d ", *(((uint8_t*)packet)+i));
	}
	PRINTF("\r\n");

   switch(packet->hdr.cmd){

   case RADIOPROT_CMD_JOINREPLY:{

	   if (!joinPending)
		  PRINTF("ERROR: No Join request was sent\r\n");

	   else if (nodeJoined)
		   PRINTF("WARNING: Node already joined\r\n");

	   else if (!packet->join.result){
		   PRINTF("ERROR: GW declined join request...\r\n");
		   joinPending = false;
		   sendJoinRequest();
	   }
	   else{
		   joinPending = false;
		   nodeJoined = true;
		   //save sent parameters
		   PRINTF("INFO: Join parameters: \r\n");
		   PRINTF("bw: %u, sf %u, cr %u, statusInfoInt %u\r\n",
				   packet->join.bw, packet->join.sf, packet->join.cr, packet->join.statusInterval);
	   }
   }
   break;


   case RADIOPROT_CMD_STATUS:{
      sendStatusInfo();
   }
   break;

   case RADIOPROT_CMD_SENSOR_DATA:{

   }
   break;

   case RADIOPROT_CMD_DATA_CHUNKS:{
	   if (!nodeJoined){
		   PRINTF("ERROR: Node is not joined... !\r\n");
		   return;
	   }
	   if (packet->chunk_req.type == RADIOPROT_CHUNK_TYPE_FFT){
		   PRINTF("INFO: FFT spectrum requested to send...\r\n");
		   seqnum = 0; numChunks = 32;
		   RadioEvents.TxDone = FFT_OnTxDone;
		   FFT_OnTxDone();
		   State = TX;

	   }


   }
   break;

   default:
      break;
   }
}

void sendJoinRequest(){
	radioprot_join_req_t req = {
			.hdr.cmd = RADIOPROT_CMD_JOINREPLY << 4,
			.hdr.id = id,
			.time = 0x0f0e0d0c,
			.fwver = 0x01,

	};
	joinPending = true;
	Radio.Send((uint8_t*)&req, sizeof(req));
	PRINTF("INFO: Sending join request... %d \r\n", sizeof(req));
}



void sendStatusInfo(){

	int counter = 0;
	uint16_t temp = TMP75_GetTemperature();
	PRINTF("Measured temperature is: %d\r\n", temp);

	//uint16_t adc = HW_ADC_ReadChannel(ADC_READ_CHANNEL);
	uint16_t adc = 4095;
	PRINTF("Measured ADC value is: %d \r\n", adc);

	//uint8_t battery = HW_GetBatteryLevel();
	uint16_t battery = 4095;
	PRINTF("Measured battery level is: %d \r\n", battery);




	memcpy((void*) Buffer, &temp, sizeof(uint16_t));
	memcpy((void*) Buffer + sizeof(uint16_t), &adc, sizeof(uint16_t));
	memcpy((void*) Buffer + 2 * sizeof(uint16_t), &battery, sizeof(uint8_t));
	snprintf((void*) Buffer + 5, BufferSize - 5, "Packet %d",counter);

	radioprot_sensor_data_t res = {
				.hdr.cmd = RADIOPROT_CMD_SENSOR_DATA << 4,
				.hdr.id = id,
				.type = 0x01,
				.seq =0xFF,
				.nsamples = 1,
				.temp = {
					.timestamp = 0,
					.value = temp,
				},
		};
	PRINTF("Sending status info %d:\r\n", sizeof(res));
	Radio.Send((uint8_t*)&res, sizeof(res));
	PRINTF("Sent %d packet, status: temperature %d, adc %d, battery %d\r\n\n", counter, temp, adc, battery);

}


void calculateFFT(){
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

   for (int i=0; i < 1024; i++){
   		testInput[2*i] = ((float32_t)ADCReadings[i])/4095 * 3.3;
   		testInput[2*i +1] = 0;
   }

   float32_t maxValue;
   arm_cfft_f32(&arm_cfft_sR_f32_len1024, testInput, ifftFlag, doBitReverse);
   arm_cmplx_mag_f32(testInput, testOutput, fftSize);
   arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);
   PRINTF("FFT calculation - index with maximal amplitude is %d\n\r", testIndex);

}

void OnTxDone( void )
{
    Radio.Sleep();
    PRINTF("OnTxDone\n\r");
    State = RX;
}

void FFT_OnTxDone(void){
	if (seqnum == numChunks){
		PRINTF("FFT spectrum sent...\r\n");
		RadioEvents.TxDone = OnTxDone;
		State = RX;
		return;
	}
	radioprot_chunk_data_t chunk = {
		.hdr.cmd =  RADIOPROT_CMD_DATA_CHUNKS << 4,
		.hdr.id = id,
		.type = RADIOPROT_CHUNK_TYPE_FFT,
		.seq = seqnum,
		.nchunks = numChunks,
		.timestamp = 0,
	};
	memcpy((uint8_t*)chunk.fft.fftChunk, (void*)testOutput + 128 * seqnum, 128);
	Radio.Send((uint8_t*)&chunk, sizeof(chunk));
	seqnum++;
	PRINTF("FFT Packet with id %d was prepared for sending...\r\n", seqnum);
}



void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep();
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;

    PRINTF("OnRxDone: ");
    PRINTF("RssiValue=%d dBm, SnrValue=%d, bytes received=%d\n\r", rssi, snr, size);
    PRINTF("%s\r\n", payload);
    gwPacketHandler((radioprot_gw_packet_t*)payload, rssi);



}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;

    PRINTF("OnTxTimeout\n\r");
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
    PRINTF("OnRxTimeout\n\r");
    if (joinPending){
    	State = TX;
    	//Radio.Send();
    }
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    PRINTF("OnRxError\n\r");
}

static void OnledEvent( void* context )
{
  LED_Toggle( LED_BLUE ) ; 
  LED_Toggle( LED_RED1 ) ; 
  LED_Toggle( LED_RED2 ) ; 
  LED_Toggle( LED_GREEN ) ;   

  TimerStart(&timerLed );
}

