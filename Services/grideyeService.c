/******************************************************************************

  @file  grideyeService.c

 @brief This file contains a simple I2C transaction with the GRIDEYE
         Created on: Jul 6, 2017

 @author: Abhinand Sukumar
 @author: Jacob Brooks

 ******************************************************************************/

#ifndef SERVICES_GRIDEYESERVICE_C_
#define SERVICES_GRIDEYESERVICE_C_

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* Board Header files */
#include "Config/Board_LoRaBUG.h"

//#include "loraCommon.h"

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include "io.h"
#include "board.h"

#include "grideyeService.h"
#include "pcFrameUtil.h"

/*******************************************************************************
 * MACROS
 */

#define DELAY_MS(i)    Task_sleep(((i) * 1000) / Clock_tickPeriod)
/*******************************************************************************
 * CONSTANTS
 */

#define GE_BUFFER_DATA_LENGTH 64

#define GE_READ_TIME 50

// Grideye registers and addresses
#define GE_SLAVE_ADDRESS 0x69
#define GE_REG_THERM_LSB 0x0E
#define GE_REG_THERM_MSB 0x0F
#define GE_REG_PIXEL_BASE 0x80
#define GE_REG_STATE 0x00
#define GE_REG_RESET 0x01

#define GE_CMD_INITIAL_RESET 0x3F
#define GE_CMD_FLAG_RESET 0x30

#define GE_POWER_PIN Board_HDR_PORTF6

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

static PIN_Handle enPinHandle;
static PIN_State enPinState;

static PIN_Config enPinTable[] = {
    GE_POWER_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// Grideye state
static uint8_t ge_mode = GE_MODE_NORMAL;
static uint8_t ge_write_buffer[GE_BUFFER_DATA_LENGTH];
static uint8_t ge_read_buffer[GE_BUFFER_DATA_LENGTH];

/*********************************************************************
 * @fn      grideye_read_byte
 * @param   addr Address to read from
 * @return  The byte read from the address
 */
static uint8_t grideye_read_byte(uint8_t addr) {
    I2C_Handle handle;
    I2C_Params params;
    I2C_Transaction i2cTrans;

    I2C_Params_init(&params);     // sets custom to NULL
    params.transferMode = I2C_MODE_BLOCKING;
    I2CCC26XX_I2CPinCfg pinCfg;
    pinCfg.pinSDA = Board_I2C0_SDA0;
    pinCfg.pinSCL = Board_I2C0_SCL0;
    params.custom = &pinCfg;

    handle = I2C_open(Board_I2C, &params);
    if(!handle) {
        //uartprintf("Error opening i2c handle during read\r\n");
    }

    i2cTrans.slaveAddress = GE_SLAVE_ADDRESS;
    i2cTrans.writeBuf = ge_write_buffer;
    i2cTrans.writeCount = 1;
    i2cTrans.readBuf = ge_read_buffer;
    i2cTrans.readCount = 1;

    ge_write_buffer[0] = addr;

    bool status = false;
    status = I2C_transfer(handle, &i2cTrans);
    if (!status) {
        //uartprintf("Failed to read i2c grideye data\r\n");
    }
    I2C_close(handle);

    return ge_read_buffer[0];
}

/*********************************************************************
 * @fn      grideye_write_bytes
 * @param   addr Address to write to
 * @param   data Data to write
 * @param   length Length of the data
 * @return  None
 */
static void grideye_write_bytes(uint8_t addr, uint8_t *data, uint8_t length) {
    I2C_Handle handle;
    I2C_Params params;
    I2C_Transaction i2cTrans;

    I2C_Params_init(&params);     // sets custom to NULL
    params.transferMode = I2C_MODE_BLOCKING;
    I2CCC26XX_I2CPinCfg pinCfg;
    pinCfg.pinSDA = Board_I2C0_SDA0;
    pinCfg.pinSCL = Board_I2C0_SCL0;
    params.custom = &pinCfg;

    handle = I2C_open(Board_I2C, &params);
    if(!handle) {
        //uartprintf("Error opening i2c handle during write\r\n");
    }

    i2cTrans.slaveAddress = GE_SLAVE_ADDRESS;
    i2cTrans.writeBuf = ge_write_buffer;
    i2cTrans.writeCount = length+1;
    i2cTrans.readBuf = NULL;
    i2cTrans.readCount = 0;

    ge_write_buffer[0] = addr;
    for (uint8_t i = 0; i < length; i++) {
        ge_write_buffer[i+1] = data[i];
    }

    bool status = false;
    status = I2C_transfer(handle, &i2cTrans);
    if (!status) {
        //uartprintf("Failed to write i2c grideye data\r\n");
    }
    I2C_close(handle);
}


/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * @fn      grideye_set_mode
 * @param   mode The mode to enable
 * @return  True if the mode was valid, false otherwise
 */
bool grideye_set_mode(ge_mode_t mode) {
    if (ge_mode == mode) {
        return false;  // Already in this mode
    }
    if (ge_mode != GE_MODE_NORMAL) {
        if (mode != GE_MODE_NORMAL) {
            return false;  // Other modes can only transition to normal state
        }
    }

    grideye_write_bytes(GE_REG_STATE, &mode, 1);

    // If we were in sleep, this is a wakeup command
    if (ge_mode == GE_MODE_SLEEP) {
        uint8_t rst_cmd;
        DELAY_MS(50);
        // Write initial reset
        rst_cmd = GE_CMD_INITIAL_RESET;
        grideye_write_bytes(GE_REG_RESET, &rst_cmd, 1);
        DELAY_MS(2);
        // Write flag reset
        rst_cmd = GE_CMD_FLAG_RESET;
        grideye_write_bytes(GE_REG_RESET, &rst_cmd, 1);
        // Wait 2 frames
        DELAY_MS(200);
    }

    ge_mode = mode;
    return true;
}

/*********************************************************************
 * @fn      grideye_get_ambient_temp
 * @param   None
 * @return  The double value of the current ambient temperature
 */
double grideye_get_ambient_temp(void)
{
    uint8_t lsb, msb;

    lsb = grideye_read_byte(GE_REG_THERM_LSB);
    msb = grideye_read_byte(GE_REG_THERM_MSB);
    return (((msb << 8) + lsb) * 0.0625);
}


/*********************************************************************
 * @fn      grideye_get_frame
 * @param   frame_buffer The buffer to fill. Must have 64 spaces
 * @return  None
 */
void grideye_get_frame(frame_t frame) {
    uint8_t lsb, msb;
    for (int i = 0; i < GE_FRAME_SIZE; i++) {
        lsb = grideye_read_byte(GE_REG_PIXEL_BASE + 2*i);
        msb = grideye_read_byte(GE_REG_PIXEL_BASE + 2*i + 1);
        frame[i] = ((msb <<8) + lsb);
    }
}

void grideye_set_power(bool power) {
    setPin(GE_POWER_PIN, !power);
}

void grideye_init() {
    grideye_set_power(true);

    // Reset grideye
    ge_mode = GE_MODE_SLEEP;
    grideye_set_mode(GE_MODE_NORMAL);
}

#endif /* SERVICES_GRIDEYESERVICE_C_ */
