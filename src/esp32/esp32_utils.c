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
 * esp32_utils.c
 *
 *      Author: marti
 */

#include "lib_conf.h"

/* ESP 32 I2C coms implementation*/

#ifdef USE_ESP_VERSION

#include "utils.h"
#include <sys/time.h>

uint32_t get_timestamp_ms(void) {
	return (uint32_t)esp_timer_get_time()/1000;
}

void print_frame(uint8_t* buffer, uint8_t buffer_len){
	uint16_t i;

	printf("0x");
	for(i = 0; i < buffer_len - 1; i++){
		printf("%02X:", buffer[i]);
	}
	printf("%02X\n", buffer[i]);
}

#endif