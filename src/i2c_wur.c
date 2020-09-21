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
 * i2c_wur.c
 *
 *      Author: marti
 */


#include "i2c_wur.h"
#include "i2c_com.h"

//#define DEBUG
#define I2C_SLAVE_ADDR 0x14

#ifdef DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif


void wur_i2c_init(void){
	i2c_com_init();
}

uint8_t wur_set_address(uint16_t addr){
	uint8_t addrBuf[2] = {0};
	uint8_t reqByte = 0;
	wur_errors_t trans_result;

	reqByte |= (WUR_ADDR_REGISTER << 1);
	reqByte |= 1;
	addrBuf[0] = (addr & 0x0300) >> 8;
	addrBuf[1] = addr & 0x00FF;

	DEBUG_PRINT("Setting WuR Addr as 0x%02X%02X.\n", addrBuf[0], addrBuf[1]);

	trans_result = i2c_com_write_register(I2C_SLAVE_ADDR, reqByte, addrBuf, 2);
	if(trans_result != WUR_OK){
		DEBUG_PRINT("Error %d in write WUR addr.\n", trans_result);
		return WUR_KO;
	}

	return WUR_OK;
}

uint8_t wur_get_address(uint16_t* addr){
	uint8_t addrBuf[2] = {0};
	uint8_t reqByte = 0;
	wur_errors_t trans_result;

	reqByte |= (WUR_ADDR_REGISTER << 1);

	DEBUG_PRINT("Getting WuR Addr.\n");

	trans_result = i2c_com_read_register(I2C_SLAVE_ADDR, reqByte, addrBuf, 2);
	if(trans_result != WUR_OK){
		DEBUG_PRINT("Error %d in read WUR addr.\n", trans_result);
		return WUR_KO;
	}

	DEBUG_PRINT("Read WuR Addr as 0x%01X%01X.\n", addrBuf[0], addrBuf[1]);

	*addr = 0;
	*addr |= (addrBuf[0] & 0x03) << 8;
	*addr |= addrBuf[1];

	return WUR_OK;
}

uint8_t wur_get_status(i2c_wur_status_t* status){
	uint8_t statusBuf[2] = {0};
	uint8_t reqByte = 0;
	wur_errors_t trans_result;

	reqByte |= (WUR_STATUS_REGISTER << 1);

	DEBUG_PRINT("Getting WuR status.\n");

	trans_result = i2c_com_read_register(I2C_SLAVE_ADDR, reqByte, statusBuf, 2);
	//if(trans_result != WUR_OK){
		//DEBUG_PRINT("Error %d in read WUR status.\n", trans_result);
		//return WUR_KO;
	//}

	DEBUG_PRINT("Read WuR Status as 0x%02X%02X.\n", statusBuf[0], statusBuf[1]);

	status->wur_status = statusBuf[0];
	status->wur_frame_len = statusBuf[1];

	return WUR_OK;
}


uint8_t wur_get_frame(uint8_t* buffer, uint8_t len){
	uint8_t reqByte = 0;
	wur_errors_t trans_result;

	reqByte |= (WUR_FRAME_REGISTER << 1);

	DEBUG_PRINT("Getting WuR frame.\n");

	trans_result = i2c_com_read_register(I2C_SLAVE_ADDR, reqByte, buffer, len);
	if(trans_result != WUR_OK){
		DEBUG_PRINT("Error %d in read WUR frame buffer.\n", trans_result);
		return WUR_KO;
	}

	DEBUG_PRINT("Got WuR frame with len %d.\n", len);

	return WUR_OK;
}
