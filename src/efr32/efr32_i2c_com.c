/*
MIT License

Copyright (c) 2020 marticervia

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * efr32_i2c_com.c
 *
 *      Author: marti
 */


#include "lib_conf.h"

/* EFR 32 I2C coms implementation*/

#ifdef USE_EFR_VERSION
#include "i2c_com.h"

#include "em_cmu.h"
#include "ustimer.h"
#include PLATFORM_HEADER
#include CONFIGURATION_HEADER
#include "em_i2c.h"


#include "hal-config.h"
/*
 * i2c_com.c
 *
 *  Created on: 25 jul. 2019
 *      Author: marti
 *  Loosely based on sillicon i2c example from:
 *  https://github.com/SiliconLabs/peripheral_examples/blob/master/series1/i2c/i2c/src/main_efr.c
 */


void i2c_com_init(void){

	// Use ~100khz SCK master mode
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

	/*USTIMER is needed for implementing backoff after waking up the WuR.*/
	USTIMER_Init();


	CMU_ClockEnable(cmuClock_I2C0, true);
	GPIO_PinModeSet(WuR_I2C_SDA_PORT, WuR_I2C_SDA_PIN, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(WuR_I2C_SCL_PORT, WuR_I2C_SCL_PIN, gpioModeWiredAndPullUpFilter, 1);

	//prepare wake_i2c in
	GPIO_PinModeSet(WuR_I2C_WAKE_PORT, WuR_I2C_WAKE_LOC, gpioModePushPull, 0);

	I2C0->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
	I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SDALOC_MASK)) | I2C_ROUTELOC0_SDALOC_LOC16;
	I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SCLLOC_MASK)) | I2C_ROUTELOC0_SCLLOC_LOC14;

	i2cInit.freq = I2C_FREQ_FAST_MAX;
	I2C_Init(I2C0, &i2cInit);

}

static inline I2C_TransferReturn_TypeDef _i2c_com_master_transfer(uint8_t i2c_slave_addr, uint16_t i2c_mode, uint8_t* buffer_1, uint16_t buffer_1_len,
		uint8_t* buffer_2, uint16_t buffer_2_len){

	// Transfer structure
	I2C_TransferSeq_TypeDef i2cTransfer;
	I2C_TransferReturn_TypeDef result;

	// Initializing I2C transfer
	i2cTransfer.addr          = (i2c_slave_addr << 1);
	i2cTransfer.flags         = i2c_mode;
	i2cTransfer.buf[0].data   = buffer_1;
	i2cTransfer.buf[0].len    = buffer_1_len;
	i2cTransfer.buf[1].data   = buffer_2;
	i2cTransfer.buf[1].len    = buffer_2_len;
	result = I2C_TransferInit(I2C0, &i2cTransfer);

	// Sending data on a blocking manner
	while (result == i2cTransferInProgress)
	{
	result = I2C_Transfer(I2C0);
	}

	return result;
}

wur_errors_t i2c_com_write_register(uint8_t i2c_slave_addr, uint8_t reg_addr, uint8_t *write_buf, uint16_t write_buf_len){
	uint8_t reg_buffer[1];
	I2C_TransferReturn_TypeDef i2c_trans_res;
    USTIMER_Delay(50);

	reg_buffer[0] = reg_addr;

    GPIO_PinOutSet(WuR_I2C_WAKE_PORT, WuR_I2C_WAKE_LOC);
    USTIMER_Delay(5);
    GPIO_PinOutClear(WuR_I2C_WAKE_PORT, WuR_I2C_WAKE_LOC);

    USTIMER_Delay(150);

	i2c_trans_res = _i2c_com_master_transfer(i2c_slave_addr, I2C_FLAG_WRITE, reg_buffer,1, NULL, 0);
	if(i2c_trans_res != i2cTransferDone){
		return WUR_KO;
	}

    USTIMER_Delay(80);

    i2c_trans_res = _i2c_com_master_transfer(i2c_slave_addr, I2C_FLAG_WRITE, write_buf, write_buf_len, NULL, 0);
    if(i2c_trans_res != i2cTransferDone){
    	return WUR_KO;
    }
    return WUR_OK;
}

wur_errors_t i2c_com_read_register(uint8_t i2c_slave_addr, uint8_t reg_addr, uint8_t *read_buf, uint16_t read_buf_len){
	uint8_t reg_buffer[1];
	I2C_TransferReturn_TypeDef i2c_trans_res;
    USTIMER_Delay(50);

	reg_buffer[0] = reg_addr;

    GPIO_PinOutSet(WuR_I2C_WAKE_PORT, WuR_I2C_WAKE_LOC);
    USTIMER_Delay(5);
    GPIO_PinOutClear(WuR_I2C_WAKE_PORT, WuR_I2C_WAKE_LOC);

    USTIMER_Delay(150);

	i2c_trans_res = _i2c_com_master_transfer(i2c_slave_addr, I2C_FLAG_WRITE, reg_buffer,1, NULL, 0);
	if(i2c_trans_res != i2cTransferDone){
		return WUR_KO;
	}

    USTIMER_Delay(80);

    i2c_trans_res = _i2c_com_master_transfer(i2c_slave_addr, I2C_FLAG_READ, read_buf, read_buf_len, NULL, 0);
    if(i2c_trans_res != i2cTransferDone){
    	return WUR_KO;
    }
    return WUR_OK;

}

#endif
