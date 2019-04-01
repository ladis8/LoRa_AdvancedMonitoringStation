/**
 * \file app.h
 *
 *
 */
 
#ifndef __APP_H
#define __APP_H

#define APP_DEBUG_MODE 	1
#define APP_STATUS_MODE 0


#include <stdint.h>	//uint_types
#include <stdlib.h>	//malloc
#include <string.h>	//memcpy
#include <stdbool.h>

#include "arm_math.h"
#include "arm_const_structs.h"

#include "app_protocol.h"
#include "config.h"

//include sensors handlers
#include "sensor_tmp75.h"







//
// Globals
//

/** Global configuration */
extern loraNode_cfg_t loraNode_cfg;


static int APPLICATION_MODE = APP_STATUS_MODE; //default


//void system_restart(void);


#endif // __APP_H
