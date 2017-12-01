/******************************************************************************

 @file  grideyeService.h

 @brief This file contains the grideye Service Interface
        Created on: Jul 6, 2017

 @author: Jacob Brooks

 ******************************************************************************/

#ifndef SERVICES_GRIDEYESERVICE_H_
#define SERVICES_GRIDEYESERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include <stdbool.h>
#include "pcFrameUtil.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

#define GE_FRAME_SIZE 64
#define GE_GRID_SIZE 8
#define MAX_GE_GRID_INDEX (GE_GRID_SIZE-1)

typedef enum ge_mode {
	GE_MODE_NORMAL = 0x00,
	GE_MODE_SLEEP = 0x10,
	GE_MODE_STANDBY_1 = 0x20,
	GE_MODE_STANDBY_2 = 0x21
} ge_mode_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the GRIDEYE Service.
 */
extern void grideyeService_createTask(void);

bool grideye_set_mode(ge_mode_t mode);

double grideye_get_ambient_temp(void);

void grideye_get_frame(frame_t frame_buffer);

void grideye_set_power(bool power);

void grideye_init(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_GRIDEYESERVICE_H_ */
