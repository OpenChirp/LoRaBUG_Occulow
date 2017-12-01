/****************************************************************
 @file pcService.h

 @brief This file contains the people counting service interface


 @author: Jacob Brooks
 ***************************************************************/

#ifndef SERVICES_PCSERVICE_H_
#define SERVICES_PCSERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * TYPEDEFS
 */

typedef struct pc_counter {
    int in_count;  //< Number of counts going in
    int out_count;  //< Number of counts going out
} pc_counter_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the occulow service
 */
extern void pcService_createTask(void);

void pc_get_counts(pc_counter_t *out_counter, bool reset);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_PCSERVICE_H_ */
