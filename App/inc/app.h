/*!
 * \file		app.h
 *
 * \brief		APPLICATION INCLUDES - definition of includes and externs needed by application
 *
 * \copyright
 *
 * \author		Ladislav Stefka
 *
 */

 
#ifndef __APP_H__
#define __APP_H__

#define APP_DEBUG_MODE 	1
#define APP_STATUS_MODE 0


#include <stdint.h>	//uint_types
#include <stdlib.h>	//malloc
#include <string.h>	//memcpy
#include <stdbool.h>

#include "arm_math.h"
#include "arm_const_structs.h"

#include "radio_protocol.h"
#include "config.h"








//
// Globals
//

/** Global configuration */
extern loraNode_cfg_t loraNode_cfg;


static int APPLICATION_MODE = APP_STATUS_MODE; //default


//void system_restart(void);


#endif
