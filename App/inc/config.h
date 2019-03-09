#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
 extern "C" {
#endif




#define TX_OUTPUT_POWER                             14        // dBm


#define LORA_FREQUENCY                              868500000 // Hz
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


 #define RX_TIMEOUT_VALUE           				5000


 typedef enum
 {
     LOWPOWER,
     RX,
     RX_TIMEOUT,
     RX_ERROR,
     TX,
     TX_TIMEOUT,
 }radioState_t;

 typedef struct
 {

    uint32_t idle_timeout;              // Idle timeout in ms before going to sleep mode
    uint32_t idle_wakeup_timeout;       // Idle wakeup timeout


    struct
    {
       uint32_t id;                    // Session ID
       uint8_t  joined;                 // Joined network flag
       uint8_t  joinPending;
       uint32_t joiningInterval;
       uint32_t statusinfoInterval;
       radioState_t state;

    } radionet;

    struct
	{
    	uint8_t bw;
    	uint8_t sf;
    	uint8_t cr;
	} lora;

    /** Sensor controller */
    struct
    {
       uint16_t lastTemp;
       uint32_t tempInterval;                // Temperature measure interval in ms

       uint16_t  lastLoadcell;
       uint32_t loadcellInterval;

    } sensor;

 } loraNode_cfg_t;



#endif
