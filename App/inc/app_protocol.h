#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H

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
   RADIOPROT_CMD_STATUS                    = 0x02,
   RADIOPROT_CMD_SENSOR_DATA               = 0x03,
   RADIOPROT_CMD_DATA_CHUNKS               = 0x04,


} appprot_cmd_type_t;


typedef struct
{
   uint8_t cmd;
   uint8_t id;
} __PACKED__ radioprot_header_t;


typedef struct
{
   radioprot_header_t hdr;
   uint32_t time;				//sync purposes
   uint8_t fwver;

} __PACKED__ radioprot_join_req_t;


/* GW  -- > Node packet JOIN RESPONSE */
typedef struct
{
   uint8_t 	result;
   uint8_t 	bw;
   uint8_t 	sf;
   uint8_t 	cr;
   uint16_t joinInterval;
   uint8_t 	appMode;
   union{
	   struct __PACKED__ {
		   uint8_t garbage[8];
	   }cmd_app;

	   struct __PACKED__ {
		   uint16_t statusinfoInterval;
		   uint8_t  rmsAveragingNum;
		   uint8_t  tempAveragingNum;
		   uint8_t  fftSamples;
		   uint8_t  adcTimings;
		   uint8_t  adcClockDivider;
		   uint8_t  fftPeaksNum;

	   }status_app;

   };

} __PACKED__ radioprot_join_res_t;


/* NODE -->  GW packet STATUS INFO */
//TODO: make peaks dynamic
typedef struct
{
	radioprot_header_t hdr;

	uint8_t   battery;
	float32_t rms;
	float32_t temperature;
	uint8_t   fftPeaksNum;
	uint16_t  fftPeaksIndexes[5];

} __PACKED__ radioprot_status_info_t;


typedef struct
{
	uint8_t nsamples;
} __PACKED__ radioprot_measure_info_req_t;

typedef struct
{
} __PACKED__ radioprot_measure_info_t;




/*-----------SENSOR PACKETS-----------*/
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
      radioprot_sensor_data_t        sensor_data;
      radioprot_chunk_data_t         chunk_data;
      radioprot_status_info_t        statusinfo;
   };

} __PACKED__ radioprot_node_packet_t;


typedef struct
{
   radioprot_header_t hdr;
   union
   {
      uint8_t                         data[APPPROT_DATA_MAXSIZE];
      radioprot_join_res_t            join;
      radioprot_sensor_data_req_t     sensor_req;
      radioprot_chunk_data_req_t      chunk_req;

   };

} __PACKED__ radioprot_gw_packet_t;


#endif
