
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
 * i2c_com.h
 *
 *      Author: marti
 */


#ifndef __I2C_H__
#define __I2C_H__

#include "lib_conf.h"

#define I2C_FLAG_WRITE          0x0001
#define I2C_FLAG_READ           0x0002

void i2c_com_init(void);
wur_errors_t i2c_com_write_register(uint8_t i2c_slave_addr, uint8_t reg_addr, uint8_t *write_buf, uint16_t write_buf_len);
wur_errors_t i2c_com_read_register(uint8_t i2c_slave_addr, uint8_t reg_addr, uint8_t *read_buf, uint16_t read_buf_len);

#endif
