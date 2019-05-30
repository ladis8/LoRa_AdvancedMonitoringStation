/*!
 * \file		radio_protocol.h
 *
 * \brief		Application radio protocol - definition of radio packets
 *
 * \copyright
 *
 * \author		Ladislav Stefka
 *
 */

#ifndef __APP_PROTOCOL_H__
#define __APP_PROTOCOL_H__

#ifndef __PACKED__
#define __PACKED__   __attribute__ ((__packed__))
#endif   // __PACKED__

#define APPPROT_DATA_MAXSIZE    128

#include <stdint.h>
#include "arm_math.h"



/** protocol commands */
typedef enum
{

   RADIOPROT_CMD_JOINREPLY                 = 0x01,
   RADIOPROT_CMD_CONFIGREPLY               = 0x02,
   RADIOPROT_CMD_STATUS                    = 0x03,
   RADIOPROT_CMD_SENSOR_DATA               = 0x04,
   RADIOPROT_CMD_DATA_CHUNKS               = 0x05,
   RADIOPROT_CMD_RESTART                   = 0x06,


} appprot_cmd_type_t;


typedef struct
{
   uint8_t cmd;
   uint8_t sId;
} __PACKED__ radioprot_header_t;



/********** Node  -- > GW **********/

/* Radio packet JOIN REQUEST */
typedef struct
{
   radioprot_header_t hdr;
   uint32_t uId;
   uint32_t time;				//sync purposes
   uint8_t fwver;

} __PACKED__ radioprot_join_req_t;


/* Radio packet CONFIG REQUEST */
typedef struct
{
   radioprot_header_t hdr;

} __PACKED__ radioprot_config_req_t;



/* Radio packet STATUS INFO */
typedef struct
{
	radioprot_header_t hdr;

	uint8_t   battery;
	uint16_t  temperature;
	uint16_t  rms;
	uint16_t  vpp;
	float32_t kurtosisRatio;
	uint8_t	  ringdownCounts;
	uint16_t  riseTime;
	uint16_t  thresholdDuration;
	uint8_t   fftPeaksNum;
	uint8_t  fftPeaksInfo[];

} __PACKED__ radioprot_status_info_t;




/********** Node  -- > GW **********/




/********** GW  -- > NODE **********/

/* Radio packet JOIN REPLY */
typedef struct
{
   uint8_t 	result;
   uint8_t 	bw;
   uint8_t 	sf;
   uint8_t 	cr;
   uint16_t joinInterval;
   uint8_t 	appMode;

} __PACKED__ radioprot_join_rep_t;

/* Radio packet CONFIG REPLY */
typedef struct
{
	// General
	uint16_t statusinfoInterval;
	uint16_t statusinfoListenInterval;
	uint8_t  tempAveragingNum;
	// FFT & ADC
	uint8_t  adcSamplingTime;
	uint8_t  adcClockDivider;
	uint8_t  fftSamplesNum;
	uint8_t  fftPeaksNum;
	uint8_t  fftPeaksDelta;
	//DSP
	uint16_t dspThresholdVoltage;
	uint8_t	 dspKurtosisTrimmedSamples;
	uint8_t  dspRmsAc;
	uint8_t  dspRmsAveragingNum;

 }__PACKED__ radioprot_config_rep_t;



 /* Radio packet RESTART */
 typedef struct
 {
    uint8_t 	resetConfig;

 } __PACKED__ radioprot_restart_t;





/*-----------SENSOR PACKETS-----------*/
 // not used in app in status mode
#define RADIOPROT_SENSOR_DATA_TYPE_TEMP       0x01
#define RADIOPROT_SENSOR_DATA_TYPE_FFT        0x02
#define RADIOPROT_SENSOR_DATA_TYPE_LOADCELL   0x03


typedef struct
{
   radioprot_header_t hdr;
   uint8_t type;                                      // Data type
   uint8_t nsamples;   								 // Number of samples

} __PACKED__ radioprot_sensor_data_req_t;

typedef struct
{
   uint32_t timestamp;
   uint16_t value;

} __PACKED__ radioprot_sensor_temp_data_t;


typedef struct
{
   uint32_t timestamp;
   uint16_t value;

} __PACKED__ radioprot_sensor_loadcell_data_t;


typedef struct
{
   radioprot_header_t hdr;
   uint8_t type;                                      // Data type
   uint8_t seq;                                       // Seq number
   uint8_t nsamples;                                  // Number of samples
   union
   {
      radioprot_sensor_temp_data_t temp;            // Temperature data array
      radioprot_sensor_loadcell_data_t loadcell;    // Load cell data array
   };

} __PACKED__ radioprot_sensor_data_t;

/*-----------SENSOR PACKETS-----------*/




/*-----------CHUNK PACKETS-----------*/
// not used in app in status mode
#define RADIOPROT_CHUNK_TYPE_FFT       0x01
#define RADIOPROT_CHUNK_TYPE_ADC       0x02

typedef struct
{
   uint8_t type;

} __PACKED__ radioprot_chunk_data_req_t;



typedef struct
{
   float32_t fftChunk[32];

} __PACKED__ radioprot_chunk_fft_t;

typedef struct
{
   uint16_t adcReadings[64];

} __PACKED__ radioprot_chunk_adcReadings_t;


typedef struct
{
	radioprot_header_t hdr;
	uint8_t type;
	uint8_t seq;
	uint8_t nchunks;
	uint32_t timestamp;
	 union
	 {
		radioprot_chunk_fft_t fft;
		radioprot_chunk_adcReadings_t adc;
	 };

} __PACKED__ radioprot_chunk_data_t;

/*-----------CHUNK PACKETS-----------*/








typedef struct
{
   radioprot_header_t hdr;

   union
   {
      uint8_t                      	 data[APPPROT_DATA_MAXSIZE];
      radioprot_join_req_t           join_req;
      radioprot_config_req_t         config_req;
      radioprot_status_info_t        statusinfo;
      radioprot_sensor_data_t        sensor_data;
      radioprot_chunk_data_t         chunk_data;
   };

} __PACKED__ radioprot_node_packet_t;


typedef struct
{
   radioprot_header_t hdr;
   union
   {
      uint8_t                         data[APPPROT_DATA_MAXSIZE];
      radioprot_join_rep_t            join_rep;
      radioprot_config_rep_t          config_rep;
      radioprot_restart_t			  restart;
      radioprot_sensor_data_req_t     sensor_req;
      radioprot_chunk_data_req_t      chunk_req;

   };

} __PACKED__ radioprot_gw_packet_t;


#endif
