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
 * wur.h
 *
 *      Author: marti
 */

#ifndef WUR_H_
#define WUR_H_


#define WUR_DEFAULT_TIMEOUT 10000
#define WUR_WAKE_TIMEOUT 10000
#define WUR_DATA_TIMEOUT 30
#define WUR_PAYLOAD_OFFSET 3

#define GPIO_WAKE GPIO_NUM_2
#define GPIO_WAKE_PIN_SEL  (1<<GPIO_NUM_2)

typedef enum wur_tx_res{
	WUR_ERROR_TX_OK = 0,
	WUR_ERROR_TX_ACK_DATA_TIMEOUT = -1,
	WUR_ERROR_TX_ACK_WAKE_TIMEOUT = -2,
	WUR_ERROR_TX_NACK = -3,
	WUR_ERROR_TX_FAILED = -4,
	WUR_ERROR_TX_BUSY = -5
}wur_tx_res_t;

typedef enum wur_rx_res{
	WUR_ERROR_RX_OK = 0,
	WUR_ERROR_RX_FAILED = -1
}wur_rx_res_t;

typedef void (*wur_tx_cb) (wur_tx_res_t tx_res);
typedef void (*wur_rx_cb) (wur_rx_res_t rx_res, uint8_t* rx_bytes, uint8_t rx_bytes_len);

void wur_init(uint16_t addr);
void wur_tick(uint32_t systick);
void wur_set_tx_cb(wur_tx_cb tx_cb);
void wur_set_rx_cb(wur_rx_cb rx_cb);

wur_tx_res_t wur_send_wake(uint16_t addr, uint16_t ms);
wur_tx_res_t wur_send_data(uint16_t addr, uint8_t* data, uint8_t data_len, uint8_t is_ack, int8_t ack_seq_num);
wur_tx_res_t wur_send_ack(uint16_t addr, int8_t ack_seq_num);

#endif /* WUR_H_ */
