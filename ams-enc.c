/*
 * Copyright (c) 2010, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Austria Microsystems AS5048B magnetic encoder I2C Interface
 *
 * by Duncan Haldane
 *
 * v.1.0
 *
 * Revisions:
 *  Duncan Haldane      2012-05-15    Initial release
 *  Andrew Pullin       2012-07-05    Ported to use i2c_driver module
 *                  
 * Notes:
 *  - This module uses the I2C1 port for communicating with the AMS encoder chips
 */
#include "i2c_driver.h"
#include "i2c.h"
#include "ams-enc.h"
#include "utils.h"
#include "string.h"
 
#define LSB2ENCDEG 0.0219

#define ENC_I2C_CHAN             2 //Encoder is on I2C bus 2
#define HALL_ADDR_RD             0x81    
#define HALL_ADDR_WR             0x80    // 0x40 <<1	
#define DATA_WAIT                50 //in ms 0.005

static union encdata {
    unsigned char chr_data[2];
    int int_data[1];
    //float float_data[1];
} EncData;

static union encspeeddata {
    unsigned char chr_data[4];
    float float_data[1];
}EncSpeedData;
	

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static inline void encoderSetupPeripheral(void);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
void encSetup(void) {
    //setup I2C port I2C1
    encoderSetupPeripheral();
}

/*****************************************************************************
 * Function Name : encGetPos
 * Description   : Read the angular position of the encoder[num], write to struct encPos
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
unsigned char* encGetPos(void) {

    unsigned char enc_data[2];

    i2cStartTx(ENC_I2C_CHAN); //Setup to burst read both registers, 0xFE and 0xFF
    i2cSendByte(ENC_I2C_CHAN, HALL_ADDR_WR);	//Write address
    i2cSendByte(ENC_I2C_CHAN, 0xFE);
    i2cEndTx(ENC_I2C_CHAN);

    i2cStartTx(ENC_I2C_CHAN);
    i2cSendByte(ENC_I2C_CHAN, HALL_ADDR_RD);		//Read address
    i2cReadString(ENC_I2C_CHAN,2,enc_data,DATA_WAIT);
    i2cEndTx(ENC_I2C_CHAN);
    EncData.chr_data[0] = enc_data[1];
    EncData.chr_data[1] = enc_data[0]; //+2 needed or not?

    return EncData.chr_data;
}

unsigned char* HallGetSpeed(void) {
    return EncSpeedData.chr_data;
}

void HallSpeedCalib(unsigned int count){

    unsigned int i;
    int prev, update;
    float rps;
    int deltas[500];
    int sumdeltas =0;
    int enc_counter=0;

    CRITICAL_SECTION_START
    // throw away first 100 data. Sometimes they are bad at the beginning.
    for (i = 0; i < 300; ++i) {
        encGetPos();
        delay_ms(1);
    }
    for (i = 0; i < count; i++) {
        prev = (EncData.chr_data[0]<<6)+(EncData.chr_data[1]&0x3F);
        encGetPos();
        update = (EncData.chr_data[0]<<6)+(EncData.chr_data[1]&0x3F);

        if(update-prev<0){
        deltas[i % 500] = 16384-(prev-update);
        }

        else{
        deltas[i % 500] = update-prev;
        }

        delay_ms(2);
        }

    for(i=0; i<count; i++){
                sumdeltas += deltas[i]; //500Hz, 500samples/1sec
            }
        rps= sumdeltas/(16384*5.0);
        EncSpeedData.float_data[0] = rps;
        sumdeltas= 0;
         // Sample at around 500Hz
    CRITICAL_SECTION_END
}

//Script for testing Slave Address of AS5048B I2C. Should give you 0
/*
unsigned char * Getaddr(void){

	unsigned char enc_data[1];

	i2cStartTx(ENC_I2C_CHAN);
    i2cSendByte(ENC_I2C_CHAN, HALL_ADDR_WR);	//Write address
    i2cSendByte(ENC_I2C_CHAN, 0x15);
    i2cEndTx(ENC_I2C_CHAN);

    i2cStartTx(ENC_I2C_CHAN);
    i2cSendByte(ENC_I2C_CHAN, HALL_ADDR_RD);		//Read address
    i2cReadString(ENC_I2C_CHAN,1,enc_data,500);
    i2cEndTx(ENC_I2C_CHAN);

    return enc_data;

}
*/
/**************************************************************
 * Function Name : encStorePos
 * Description   : put integer angular position value to union
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/

void encStorePos(void){
	unsigned char enc_data[2];

    i2cStartTx(ENC_I2C_CHAN); //Setup to burst read both registers, 0xFE and 0xFF
    i2cSendByte(ENC_I2C_CHAN, HALL_ADDR_WR);	//Write address
    i2cSendByte(ENC_I2C_CHAN, 0xFE);
    i2cEndTx(ENC_I2C_CHAN);

    i2cStartTx(ENC_I2C_CHAN);
    i2cSendByte(ENC_I2C_CHAN, HALL_ADDR_RD);		//Read address
    i2cReadString(ENC_I2C_CHAN,2,enc_data,10000);
    i2cEndTx(ENC_I2C_CHAN);
    EncData.int_data[0] = ((enc_data[1]<<6)+(enc_data[0] & 0x3F));
}
/***********************************************************************
 * Function Name : encGetFloatPos
 * Description   : Read the angular position of encoder[num] return as float
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
float encGetFloatPos(void) {

    float pos;
    encStorePos();
    pos = EncData.int_data[0]*LSB2ENCDEG; //calculate Float

    return pos;
}
/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/*****************************************************************************
 * Function Name : encoderSetupPeripheral
 * Description   : Setup I2C for encoders (Uses I2C 2 bus)
 * Parameters    : None
 * Return Value  : None
 ****************************************************************************/
static inline void encoderSetupPeripheral(void) { //same setup as ITG3200 for compatibility
    unsigned int I2C2CONvalue, I2C2BRGvalue;
    I2C2CONvalue = I2C2_ON & I2C2_IDLE_CON & I2C2_CLK_HLD &
                   I2C2_IPMI_DIS & I2C2_7BIT_ADD & I2C2_SLW_DIS &
                   I2C2_SM_DIS & I2C2_GCALL_DIS & I2C2_STR_DIS &
                   I2C2_NACK & I2C2_ACK_DIS & I2C2_RCV_DIS &
                   I2C2_STOP_DIS & I2C2_RESTART_DIS & I2C2_START_DIS;

    // BRG = Fcy(1/Fscl - 1/10000000)-1, Fscl = 909KHz  
    I2C2BRGvalue = 40; 
    OpenI2C2(I2C2CONvalue, I2C2BRGvalue);
    IdleI2C2();
}
