

#include <string.h>
#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "low_power_manager.h"
#include "vcom.h"



//include sensors handlers
#include "sensor_tmp75.h"



//TODO:
//bug when i2c turn on with i2c
//read ADC value
//create format


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

#define RX_TIMEOUT_VALUE          5000
#define BUFFER_SIZE                  32 // Define the payload size here
#define LED_PERIOD_MS               200

#define LEDS_OFF   do{ \
                   LED_Off( LED_BLUE ) ;   \
                   LED_Off( LED_RED ) ;    \
                   LED_Off( LED_GREEN1 ) ; \
                   LED_Off( LED_GREEN2 ) ; \
                   } while(0) ;


const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

 /* Led Timers objects*/
static  TimerEvent_t timerLed;
static RadioEvents_t RadioEvents;


void OnTxDone( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxTimeout( void );
void OnRxTimeout( void );
void OnRxError( void );

static void OnledEvent( void* context );

void SendStatusInfo(int counter);










int main( void )
{
    bool isMaster = true;

    HAL_Init( );

    //differs
    SystemClock_Config();

    HW_Init( );




    bool i2cStatus = scanI2C1();
     if (i2cStatus)
     	PRINTF("Temperature sensor detected...\r\n");

    TMP75_Init();

    //DBG_Init( );


    int cnt = 0;
    while(cnt < 3){
    	uint16_t temp = TMP75_GetTemperature();
    	PRINTF("Measured temperature is: %d\r\n", temp);
    	HAL_Delay(500);
    	cnt++;
    }








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
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );




    /*Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
                                   */





    int counter = 0;
    Radio.Rx( RX_TIMEOUT_VALUE );

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
            //GpioWrite( &Led2, GpioRead( &Led2 ) ^ 1 );
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
        	if( isMaster == true ){
        		SendStatusInfo(counter);
        	    counter++;
        	}
        	else
        	{
        		Radio.Rx( RX_TIMEOUT_VALUE );
        	}
        	State = LOWPOWER;
            break;

        case TX_TIMEOUT:
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


void SendStatusInfo(int counter){

	uint16_t temp = TMP75_GetTemperature();
	PRINTF("Measured temperature is: %d\r\n", temp);



	PRINTF("Sending status info:\r\n");

	/*Buffer[0] = temp >> 8;
	Buffer[1] = temp;
	Buffer[2] = (uint8_t)'T';
	Buffer[3] = 0;*/
	 // Send the reply to the PING string
	sprintf( ( char* )Buffer, "Ping %d", counter);
	DelayMs(1);
	int s = Radio.GetStatus();
	Radio.Send( Buffer, BufferSize );
	PRINTF("Sent %d PING status in RX_TIMEOUT state: temperature %d, status %d\r\n\n", counter, temp, s);

}



void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
    PRINTF("OnTxDone\n\r");
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;

    PRINTF("OnRxDone\n\r");
    PRINTF("RssiValue=%d dBm, SnrValue=%d\n\r", rssi, snr);
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

