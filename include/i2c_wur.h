
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
 * i2c_wur.h
 *
 *      Author: marti
 */

#ifndef __I2C_WUR_H__
#define __I2C_WUR_H__
#include "lib_conf.h"

#define WUR_STATUS_REGISTER 1
#define WUR_ADDR_REGISTER 2
#define WUR_FRAME_REGISTER 3

typedef enum wurx_states{
	WURX_SLEEP = 0,
	WURX_DECODING_FRAME = 1,
	WURX_HAS_FRAME = 2,
}wurx_states_t;

typedef struct wur_status{
	wurx_states_t wur_status;
	uint8_t wur_frame_len;
}i2c_wur_status_t;


void wur_i2c_init(void);
uint8_t wur_set_address(uint16_t addr);
uint8_t wur_get_address(uint16_t* addr);
uint8_t wur_get_status(i2c_wur_status_t* status);
uint8_t wur_get_frame(uint8_t* buffer, uint8_t len);


#define WUR_OK 0
#define WUR_KO 1

#endif