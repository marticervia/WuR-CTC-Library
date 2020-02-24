
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