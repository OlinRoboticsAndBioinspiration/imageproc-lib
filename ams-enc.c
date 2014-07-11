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
 
#define LSB2ENCDEG 0.0219

#define ENC_I2C_CHAN             2 //Encoder is on I2C bus 2
#define HALL_ADDR_RD             0x81    
#define HALL_ADDR_WR             0x80    // 0x40 <<1	

static union encdata {
	unsigned char chr_data[2];
	float float_data[4];
} EncData;
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
    i2cReadString(1,2,enc_data,10000);
    i2cEndTx(ENC_I2C_CHAN);
    EncData.chr_data[0] = enc_data[1];
    EncData.chr_data[1] = enc_data[0]+2;

    return EncData.chr_data;
}

int encStorePos(void){
	unsigned char enc_data[2];

    i2cStartTx(ENC_I2C_CHAN); //Setup to burst read both registers, 0xFE and 0xFF
    i2cSendByte(ENC_I2C_CHAN, HALL_ADDR_WR);	//Write address
    i2cSendByte(ENC_I2C_CHAN, 0xFE);
    i2cEndTx(ENC_I2C_CHAN);

    i2cStartTx(ENC_I2C_CHAN);
    i2cSendByte(ENC_I2C_CHAN, HALL_ADDR_RD);		//Read address
    i2cReadString(1,2,enc_data,10000);
    i2cEndTx(ENC_I2C_CHAN);

    return ((enc_data[1]<<6)+(enc_data[0] & 0x3F));
}
/*****************************************************************************
 * Function Name : encSumPos
 * Description   : Count encoder[num] rollovers, write to struct encPos
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
/*void encSumPos(void) {

	int prev = EncData.chr_data;
	int update;
	encGetPos();
	update = encPos[0].POS;
	
	if( (update-prev) > 8192 ){		    	//Subtract one Rev count if diff > 180
		encPos[0].oticks--;
	}
		if( (prev-update) > 8192 ){			//Add one Rev count if -diff > 180
		encPos[0].oticks++;
	}
		
}*/

/*****************************************************************************
 * Function Name : encGetFloatPos
 * Description   : Read the angular position of encoder[num] return as float
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
float encGetFloatPos(ENCPOS *encPos) {

    float pos;
    int position;
    position=encStorePos();
    pos = position*LSB2ENCDEG; //calculate Float

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
