

#include "app.h"
#include "util_console.h"
#include "radio.h"


bool _joinReplyCallback(radioprot_gw_packet_t *packet){

	bool success = true;
	PRINTF("INFO: Received JOINREPLY, new session id %02x, running mode %s \r\n", packet->hdr.sId, (packet->join_rep.appMode)? "DEBUG": "STATUS");

	loraNode_cfg.radionet.sId = packet->hdr.sId;
	loraNode_cfg.radionet.joinInterval = packet-> join_rep.joinInterval;

	/* Save sent LoRa parameters */
	loraNode_cfg.lora.bw = packet-> join_rep.bw;
	loraNode_cfg.lora.sf = packet-> join_rep.sf;
	loraNode_cfg.lora.cr = packet-> join_rep.cr;

	APPLICATION_MODE = packet-> join_rep.appMode;

	/* TODO: Configure Radio */
	//pass

	return success;
}

bool _configReplyCallback(radioprot_gw_packet_t *packet){

		bool success = true;
		PRINTF("INFO: Received CONFIGREPLY for session id %02x...", packet->hdr.sId);
		//GENERAL
	    loraNode_cfg.radionet.statusinfoInterval = packet-> config_rep.statusinfoInterval;
	    loraNode_cfg.radionet.statusinfoListenInterval = packet-> config_rep.statusinfoListenInterval;
	    //TODO: Move somwhere else
	    loraNode_cfg.adc_fft.tempAveragingNum = packet-> config_rep.tempAveragingNum;

	    //ADC & FFT
		loraNode_cfg.adc_fft.adcClockDivider = packet-> config_rep.adcClockDivider;
		loraNode_cfg.adc_fft.adcSamplingTime = packet-> config_rep.adcSamplingTime;
		loraNode_cfg.adc_fft.fftSamplesNum = packet-> config_rep.fftSamplesNum;
		loraNode_cfg.adc_fft.fftPeaksNum = packet-> config_rep.fftPeaksNum;
		loraNode_cfg.adc_fft.fftPeaksDelta = packet-> config_rep.fftPeaksDelta;
		//DSP
		loraNode_cfg.dsp.thresholdVoltage = packet->config_rep.dspThresholdVoltage;
		loraNode_cfg.dsp.kurtosisTrimmedSamples = packet->config_rep.dspKurtosisTrimmedSamples;
		loraNode_cfg.dsp.rmsAc = packet-> config_rep.dspRmsAc;
		loraNode_cfg.dsp.rmsAveragingNum = packet-> config_rep.dspRmsAveragingNum;


		/* Print config */
		printConfig();
		return success;

}

bool _restartCallback(radioprot_gw_packet_t *packet){
	bool success = true;
	PRINTF("INFO: Received RESTART for session id %02x, restart: %s...", packet->hdr.sId, (packet->restart.resetConfig)? "CONFIG" : "HARD");

}

void rxGWPacketHandler(radioprot_gw_packet_t *packet){


	switch(packet->hdr.cmd){


		/*  Join Reply handler */
    	case RADIOPROT_CMD_JOINREPLY:{

    		if (loraNode_cfg.radionet.nodeJoined){				//stays in low power mode
    			PRINTF("WARNING: Node already joined\r\n");
    			loraNode_cfg.radionet.state = LOWPOWER;
    		}

    		else if (!loraNode_cfg.radionet.joinPending){		//not joined --> keep sending join
    			PRINTF("ERROR: No JOIN REQUEST was sent\r\n");
    			loraNode_cfg.radionet.state = TX;
    		}

		    else if (!packet->join_rep.result){					//not joined --> keep sending join
			    PRINTF("ERROR: GW declined join_rep request...\r\n");
			    loraNode_cfg.radionet.joinPending = false;
			    loraNode_cfg.radionet.state = TX;

		    }
		    else{												//joined --> send config request
			    loraNode_cfg.radionet.joinPending = false;
			    loraNode_cfg.radionet.nodeJoined = true;
			    _joinReplyCallback(packet);
				loraNode_cfg.radionet.state = TX;
		    }
    	}
    	break;

    	/*  Config Reply handler */
    	case RADIOPROT_CMD_CONFIGREPLY:{						//always come in STATUS mode

    		if (!loraNode_cfg.radionet.nodeJoined || loraNode_cfg.radionet.nodeConfigured){		//not joined or configured--> keep sending join/config
    			PRINTF("WARNING: Node is not joined or is already configured,  config packet was dropped\r\n");
    			loraNode_cfg.radionet.state = TX;
    		}

    		else if (!loraNode_cfg.radionet.configPending){		//not configured --> keep sending config
    		    PRINTF("ERROR: No CONFIG REQUEST was sent\r\n");
    			loraNode_cfg.radionet.state = TX;
    		}

    		else {												//configured --> decide
    			loraNode_cfg.radionet.configPending = false;
    			loraNode_cfg.radionet.nodeConfigured = true;
    			_configReplyCallback(packet);

    			if (APPLICATION_MODE == APP_STATUS_MODE)		//configured in STATUS -> send status info
    				loraNode_cfg.radionet.state = TX;

    			else if (APPLICATION_MODE == APP_DEBUG_MODE)	//configured in STATUS -> wait for command
    				loraNode_cfg.radionet.state = RX;
    		}
    	}
    	break;

    	case RADIOPROT_CMD_RESTART:{
    		bool resetConfig =true;
    	};
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

