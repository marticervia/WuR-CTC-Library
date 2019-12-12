
#include "lib_conf.h"

/* ESP 32 I2C coms implementation*/

#ifdef USE_EFR_VERSION

#include "utils.h"

uint32_t get_timestamp_ms(void) {
	//TODO: Implement this
	return halCommonGetInt32uMillisecondTick();
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
