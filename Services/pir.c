/******************************************************************************

  @file  pir.c

 @brief This file contains the PIR interface
         Created on: Oct 10, 2017

 @author: Jacob Brooks

 ******************************************************************************/

#ifndef SERVICES_PIR_C_
#define SERVICES_PIR_C_

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>

#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* Board Header files */
#include "Config/Board_LoRaBUG.h"

#include "io.h"
#include "board.h"


/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

#define PIR_PIN Board_HDR_ADIO6

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

static PIN_Handle pirPinHandle;
static PIN_State pirPinState;

static PIN_Config pirPinTable[] = {
    PIR_PIN | PIN_INPUT_EN | PIN_PULLDOWN,
    PIN_TERMINATE
};

static PIN_IntCb callback;


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void pir_init(PIN_IntCb cb) {
    pirPinHandle = PIN_open(&pirPinState, pirPinTable);
    if (pirPinHandle == NULL)
    {
        uartputs("Failed to open board header pins\r\n");
    }
    callback = cb;
}

void pir_enable_interrupt() {
    if (PIN_registerIntCb(&pirPinState, callback) != PIN_SUCCESS) {
        uartputs("Error registering pin callback\r\n");
    }
    if (PIN_setInterrupt(&pirPinState, PIR_PIN | PIN_IRQ_POSEDGE) != PIN_SUCCESS) {
        uartputs("Error setting pin interrupt\r\n");
    }
}

void pir_disable_interrupt() {
    if (PIN_setInterrupt(&pirPinState, PIR_PIN | PIN_IRQ_DIS) != PIN_SUCCESS) {
        uartputs("Error disabling pin interrupt\r\n");
    }
}

int pir_get_value() {
    return PIN_getInputValue(PIR_PIN);
}


#endif /* SERVICES_PIR_C_ */
