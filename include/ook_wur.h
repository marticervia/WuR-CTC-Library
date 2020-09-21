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
 * ook_wur.h
 *
 *      Author: marti
 */

#ifndef OOK_WUR_H_
#define OOK_WUR_H_

#include "lib_conf.h"

#define WUR_WAKE_LEN 2
#define WUR_HEADER_LEN 5 //5 bytes with CRC
#define WUR_MAX_DATA_LEN 89
#define WUR_MAX_DATA_LEN 89

#define WAKE_FLAG 0b001
#define ACK_FLAG 0b010
#define DATA_FLAG 0b100

#define DEFAULT_WUR_FRAME { 														\
  0b01010101, /*sync word "11" and first byte of 12 bit address "010101010101"*/	\
  0b01010010, /*last nibble of 12 bit address, type "001" AND PARITY "0"*/			\
  0x0C, 	  /*length*/															\
  0x55, 0x33, 0x0F,																	\
  0x55, 0x33, 0x0F,																	\
  0x55, 0x33, 0x0F,																	\
  0x55, 0x33, 0x0F																	\
};

#define DEFAULT_WUR_LEN 16

typedef enum ook_tx_errors{
	OOK_WUR_TX_ERROR_SUCCESS = 0,
	OOK_WUR_TX_ERROR_QUEUED = 1,
	OOK_WUR_TX_ERROR_TIMEOUT = -1,
	OOK_WUR_TX_ERROR_BUSY = -2,
	OOK_WUR_TX_ERROR_FRAME_FORMAT = -3,
	OOK_WUR_TX_ERROR_FAILED = -4
}ook_tx_errors_t;

void ook_wur_init(void);

ook_tx_errors_t ook_wur_wake(uint16_t dest, uint16_t src, uint16_t ms_wake, uint8_t seq);
ook_tx_errors_t ook_wur_data(uint16_t dest, uint16_t src, uint8_t* data, uint8_t len, bool ack, uint8_t seq);
ook_tx_errors_t ook_wur_ack(uint16_t dest, uint16_t src, uint8_t seq);

#endif /* OOK_WUR_H_ */
