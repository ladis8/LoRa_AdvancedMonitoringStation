

#include "app.h"
#include "util_console.h"


bool joinReplyCallback(radioprot_gw_packet_t *packet){


	PRINTF("INFO: Received JOINREPLY, new seassion id %d, running mode %d \r\n", packet->hdr.id, packet->join.appMode);
	loraNode_cfg.radionet.sId = packet->hdr.id;
	APPLICATION_MODE = packet-> join.appMode;

	/* Save sent LoRa parameters */
	loraNode_cfg.lora.bw = packet-> join.bw;
	loraNode_cfg.lora.sf = packet-> join.sf;
	loraNode_cfg.lora.cr = packet-> join.cr;

	/* Save sent configuration for status mode */
	if (APPLICATION_MODE == APP_STATUS_MODE){

		loraNode_cfg.radionet.joinInterval = packet-> join.joinInterval;
		loraNode_cfg.radionet.statusinfoInterval = packet-> join.status_app.statusinfoInterval;

		loraNode_cfg.adc_fft.fftSamples = packet-> join.status_app.fftSamples;
		loraNode_cfg.adc_fft.adcClockDivider = packet-> join.status_app.adcClockDivider;
		loraNode_cfg.adc_fft.adcTimings = packet-> join.status_app.adcTimings;
		loraNode_cfg.adc_fft.fftPeaksNum = packet-> join.status_app.fftPeaksNum;

		loraNode_cfg.adc_fft.rmsAveragingNum = packet-> join.status_app.rmsAveragingNum;
		loraNode_cfg.adc_fft.tempAveragingNum = packet-> join.status_app.tempAveragingNum;
	}


	/* Print config */
	printConfig();

	bool success = true;
	/* Configure Radio */
	//pass

	/* Configure ADC */
	//pass
	/* Select proper FFT structures */
	//pass

	/* Set timer for status interval */
	// in main???
	loraNode_cfg.radionet.state = TX;
	return success;
}




void gwPacketHandler(radioprot_gw_packet_t *packet, uint8_t rssi){



	switch(packet->hdr.cmd){


		/*  Join Reply handler */
    	case RADIOPROT_CMD_JOINREPLY:{

    		if (!loraNode_cfg.radionet.joinPending)		//stays in low power mode
    			PRINTF("ERROR: No Join request was sent\r\n");

		    else if (loraNode_cfg.radionet.nodeJoined)	//stays in low power mode
			    PRINTF("WARNING: Node already joined\r\n");

		    else if (!packet->join.result){
			    loraNode_cfg.radionet.joinPending = false;
			    loraNode_cfg.radionet.state = TX;
			    PRINTF("ERROR: GW declined join request...\r\n");
		    }
		    else{
			    loraNode_cfg.radionet.joinPending = false;
			    loraNode_cfg.radionet.nodeJoined = true;
			    joinReplyCallback(packet);
		    }
    	}
    	break;


    	case RADIOPROT_CMD_STATUS:{
    		//sendStatusInfo();
    	}
    	break;

    	case RADIOPROT_CMD_SENSOR_DATA:{

    	}
    	break;

    	case RADIOPROT_CMD_DATA_CHUNKS:{
    		/*if (!nodeJoined){
    			PRINTF("ERROR: Node is not joined... !\r\n");
    			return;
    		}
    		if (packet->chunk_req.type == RADIOPROT_CHUNK_TYPE_FFT){
    			PRINTF("INFO: FFT spectrum requested to send...\r\n");
    			seqnum = 0; numChunks = 32;
    			RadioEvents.TxDone = FFT_OnTxDone;
    			FFT_OnTxDone();
    			loraNode_cfg.radionet.state = TX;

    		}*/
    	}
    	break;

    	default:
    		break;
	}
}

