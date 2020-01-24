/*
 * wur.c
 *
 *  Created on: 29 jul. 2019
 *      Author: marti
 */
#include "lib_conf.h"

#include "wur.h"
#include "ook_wur.h"
#include "i2c_wur.h"
#include "utils.h"
#include <string.h>

#define ESP_INTR_FLAG_DEFAULT 0

#define FRAME_BUF_LEN (WUR_HEADER_LEN + WUR_MAX_DATA_LEN)

typedef enum wur_tx_status{
	WUR_STATUS_IDLE,
	WUR_STATUS_WAIT_DATA_ACK,
	WUR_STATUS_WAIT_WAKE_ACK
}wur_tx_status_t;

typedef struct wur_context{
	uint32_t tx_timestamp;
	uint32_t rx_timestamp;
	wur_tx_status_t wur_status;
	uint8_t expected_seq_num;
	wur_tx_cb tx_cb;
	wur_rx_cb rx_cb;
	uint8_t frame_buffer[FRAME_BUF_LEN];
	uint8_t frame_len;
	uint16_t wur_addr;
}wur_context_t;

static wur_context_t wur_context;
static volatile uint8_t wur_op_pending;

static WuRBinarySemaphoreHandle_t wur_semaphore;
static WuRRecursiveMutexHandle_t wur_mutex;

#ifdef USE_ESP_VERSION
/* handles the interruption genrated by the WUR GPIO pin. */
static IRAM_ATTR void wur_int_handler(void* arg)
{
	BaseType_t xTaskWokenByReceive = pdTRUE;
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_num == GPIO_WAKE){
		wur_op_pending = true;
		xSemaphoreGiveFromISR(wur_semaphore, &xTaskWokenByReceive);
		portYIELD_FROM_ISR();
    }
}
#endif

#ifdef USE_EFR_VERSION
/* handles the interruption genrated by the WUR GPIO pin. */
void wur_int_handler(uint8_t interrupt){
  wur_op_pending = 1;
}
#endif

#ifdef USE_FREERTOS
static void wur_tick_task(void* args){
	printf("Wur Tick task started!\n");
	while(1){
		uint32_t tick = get_timestamp_ms();
		wur_tick(tick);
	}
}
#endif

void wur_init(uint16_t addr){

#ifdef USE_ESP_VERSION
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO0 here
    io_conf.pin_bit_mask = GPIO_WAKE_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_WAKE, wur_int_handler, (void*) GPIO_WAKE);
#endif
#ifdef USE_EFR_VERSION
	/* prepare WuR interrupt pin*/
	GPIO_PinModeSet(WuR_INT_WAKE_PORT, WuR_INT_WAKE_LOC, gpioModeInputPullFilter, 1);
	GPIOINT_Init();
	GPIOINT_CallbackRegister(WuR_INT_WAKE_LOC, wur_int_handler);
	GPIO_ExtIntConfig(WuR_INT_WAKE_PORT, WuR_INT_WAKE_LOC, WuR_INT_WAKE_LOC, 1, 0, true);
#endif

    wur_semaphore = WuRBinarySemaphoreCreate();
    WuRBinarySemaphoreGive(wur_semaphore);

    wur_mutex = WuRRecursiveMutexCreate();
    WuRRecursiveMutexGive(wur_mutex);

	ook_wur_init();
	wur_i2c_init();

	wur_context.tx_timestamp = 0;
	wur_context.rx_timestamp = 0;
	wur_context.wur_status = WUR_STATUS_IDLE;
	wur_context.expected_seq_num = 1;
	wur_context.tx_cb = NULL;
	wur_context.rx_cb = NULL;
	memset(wur_context.frame_buffer, 0, FRAME_BUF_LEN);
	wur_context.frame_len = 0;
	wur_context.wur_addr = addr;
	wur_set_address(addr);

#ifdef USE_FREERTOS

	//TODO: get a real tack priority
	xTaskCreate(wur_tick_task,
	            "wur_task",
	            2048,
	            NULL,
	            24,
	            NULL
	         );
#endif
}

void wur_tick(uint32_t systick){

	WuRRecursiveMutexTake(wur_mutex, WuRMaxDelayMS);

	if(wur_context.wur_status == WUR_STATUS_WAIT_DATA_ACK){
		if(systick - wur_context.tx_timestamp > WUR_DATA_TIMEOUT){
			printf("Timeout to ack last data frame\n");
			WuRRecursiveMutexGive(wur_mutex);
			if(wur_context.tx_cb){
				wur_context.wur_status = WUR_STATUS_IDLE;
				wur_context.tx_cb(WUR_ERROR_TX_ACK_DATA_TIMEOUT);
			}
			printf("Wur Returns to idle\n");
			return;
		}
		else{
			uint32_t remaining_time = WUR_DATA_TIMEOUT - (systick - wur_context.tx_timestamp);
			printf("Wur waits %d ms for a DATA ACK\n", remaining_time);
			WuRRecursiveMutexGive(wur_mutex);
			WuRBinarySemaphoreTake(wur_semaphore, remaining_time/WuRTickPeriodMS);
			WuRRecursiveMutexTake(wur_mutex, WuRMaxDelayMS);
		}
	}else if(wur_context.wur_status == WUR_STATUS_WAIT_WAKE_ACK){
		if(systick - wur_context.tx_timestamp > WUR_WAKE_TIMEOUT){
			printf("Timeout to ack last wake frame\n");
			WuRRecursiveMutexGive(wur_mutex);
			if(wur_context.tx_cb){
				wur_context.wur_status = WUR_STATUS_IDLE;
				wur_context.tx_cb(WUR_ERROR_TX_ACK_WAKE_TIMEOUT);
			}
			printf("Wur Returns to idle\n");
			return;
		}
		else{
			uint32_t remaining_time = WUR_WAKE_TIMEOUT - (systick - wur_context.tx_timestamp);
			printf("Wur waits %d ms for a WAKE ACK\n", remaining_time);
			WuRRecursiveMutexGive(wur_mutex);
			WuRBinarySemaphoreTake(wur_semaphore, remaining_time/WuRTickPeriodMS);
			WuRRecursiveMutexTake(wur_mutex, WuRMaxDelayMS);

		}
	}else{
		WuRRecursiveMutexGive(wur_mutex);
		WuRBinarySemaphoreTake(wur_semaphore, WUR_DEFAULT_TIMEOUT/WuRTickPeriodMS);
		WuRRecursiveMutexTake(wur_mutex, WuRMaxDelayMS);
	}

	i2c_wur_status_t wurx_state;

#ifdef USE_EFR_VERSION
	if(!wur_op_pending){
		if(systick % 10000 == 0){
			emberAfCorePrintln("[%d]: Device IDLE, asking for WuR status", systick);
			if(wur_get_status(&wurx_state) != WUR_OK){
				printf("Warning: failed to get state from WuR after interrupt!\n");
				goto exit;
			}
			if(wurx_state.wur_status != WURX_HAS_FRAME){
				goto exit;
			}
			goto get_frame;
		}else{
			goto exit;
		}
	}
#endif
	if(!wur_op_pending){
		goto exit;
	}
	wur_op_pending = false;
	if(wur_get_status(&wurx_state) != WUR_OK){
		printf("Warning: failed to get state from WuR after interrupt!\n");
		goto exit;
	}
	if(wurx_state.wur_status != WURX_HAS_FRAME){
		goto exit;
	}

#ifdef USE_EFR_VERSION
	get_frame:
#endif
	if(wur_get_frame(wur_context.frame_buffer, wurx_state.wur_frame_len) != WUR_OK){
		printf("Warning: failed to get frame from WuR!\n");
		goto exit;
	}

	//printf("Got WuR Frame:\n");
	//print_frame(wur_context.frame_buffer, wurx_state.wur_frame_len);

	wur_context.frame_len = wurx_state.wur_frame_len;
	uint8_t frame_type = (wur_context.frame_buffer[2] & 0x0E) >> 1;
	uint8_t seq_num = wur_context.frame_buffer[2] & 0x01;

	uint16_t src_addr;
	memcpy(&src_addr, &wur_context.frame_buffer[1], 2);
	src_addr = ntohs(src_addr);
	src_addr = (src_addr >> 4);
	src_addr &= 0x03FF;	

	/* check the frame flags! */
	if(frame_type & ACK_FLAG){
		if(wur_context.expected_seq_num == seq_num){
			if(wur_context.tx_cb){
				//printf("Got ACK!\n");
				wur_context.wur_status = WUR_STATUS_IDLE;
				wur_context.tx_cb(WUR_ERROR_TX_OK);
				wur_context.expected_seq_num ^= 1;
			}
		}
		else{
			if(wur_context.tx_cb){
				//printf("Got NACK!\n");
				wur_context.wur_status = WUR_STATUS_IDLE;
				wur_context.tx_cb(WUR_ERROR_TX_NACK);
				wur_context.expected_seq_num ^= 1;
			}
		}
	}
	else if((frame_type & DATA_FLAG) || (frame_type & WAKE_FLAG)){
		//printf("Got DATA or WAKE!\n");
		/* an ack can piggiback a response frame, so continue*/
		if((wur_context.frame_len > 3) && wur_context.rx_cb){
			//printf("parse DATA!\n");
			wur_context.rx_cb(WUR_ERROR_RX_OK, wur_context.frame_buffer, wur_context.frame_len);
			wur_context.rx_timestamp = systick;
		}
		if(!(frame_type & ACK_FLAG)){
			//printf("Acknowledge DATA frame to 0x%02X!\n", addr);
			wur_send_ack(src_addr, seq_num);
		}
	}else{
		printf("Got FLAGless frame! (protocol error?)\n");
	}

	memset(wur_context.frame_buffer, 0, FRAME_BUF_LEN);
	wur_context.frame_len = 0;

	exit:
	WuRRecursiveMutexGive(wur_mutex);
}

void wur_set_tx_cb(wur_tx_cb tx_cb){
	wur_context.tx_cb = tx_cb;
}

void wur_set_rx_cb(wur_rx_cb rx_cb){
	wur_context.rx_cb = rx_cb;
}

wur_tx_res_t wur_send_wake(uint16_t addr, uint16_t ms){
	ook_tx_errors_t tx_res;
	wur_tx_res_t res;

	WuRRecursiveMutexTake(wur_mutex, WuRMaxDelayMS);

	if(wur_context.wur_status != WUR_STATUS_IDLE){
		res = WUR_ERROR_TX_BUSY;
		goto exit;
	}

	tx_res = ook_wur_wake(addr, wur_context.wur_addr, ms, wur_context.expected_seq_num);
	if(tx_res != OOK_WUR_TX_ERROR_SUCCESS){
		printf("Warning: failed to start WAKE transmission!\n");
		wur_context.wur_status = WUR_STATUS_IDLE;
		res = WUR_ERROR_TX_FAILED;
		goto exit;
	}

	wur_context.tx_timestamp = get_timestamp_ms();
	wur_context.wur_status = WUR_STATUS_WAIT_WAKE_ACK;

	res = WUR_ERROR_TX_OK;
	WuRBinarySemaphoreGive(wur_semaphore);

	exit:
	WuRRecursiveMutexGive(wur_mutex);
	return res;
}

wur_tx_res_t wur_send_data(uint16_t addr, uint8_t* data, uint8_t data_len, uint8_t is_ack, int8_t ack_seq_num){
	ook_tx_errors_t tx_res;
	wur_tx_res_t res;

	WuRRecursiveMutexTake(wur_mutex, WuRMaxDelayMS);

	if(wur_context.wur_status != WUR_STATUS_IDLE){
		res = WUR_ERROR_TX_BUSY;
		goto exit;
	}

	if(ack_seq_num < 0){
		ack_seq_num = wur_context.expected_seq_num;
	}

	tx_res = ook_wur_data(addr, wur_context.wur_addr, data, data_len, is_ack, ack_seq_num);
	if(tx_res != OOK_WUR_TX_ERROR_SUCCESS){
		printf("Warning: failed to start DATA transmission!\n");
		wur_context.wur_status = WUR_STATUS_IDLE;
		res = WUR_ERROR_TX_FAILED;
		goto exit;
	}

	wur_context.tx_timestamp = get_timestamp_ms();
	wur_context.wur_status = WUR_STATUS_WAIT_DATA_ACK;

	res = WUR_ERROR_TX_OK;
	WuRBinarySemaphoreGive(wur_semaphore);

	exit:
	WuRRecursiveMutexGive(wur_mutex);
	return res;
}


wur_tx_res_t wur_send_ack(uint16_t addr, int8_t ack_seq_num){
	ook_tx_errors_t tx_res;
	wur_tx_res_t res;
	//printf("Sending WuR ACK!\n");

	WuRRecursiveMutexTake(wur_mutex, WuRMaxDelayMS);

	if(ack_seq_num < 0){
		ack_seq_num = wur_context.expected_seq_num;
	}

	tx_res = ook_wur_ack(addr, wur_context.wur_addr, ack_seq_num);
	if(tx_res != OOK_WUR_TX_ERROR_SUCCESS){
		printf("Warning: failed to start ACK transmission!\n");
		res = WUR_ERROR_TX_FAILED;
		goto exit;
	}
	printf("Sent WuR ACK!\n");
	res = WUR_ERROR_TX_OK;

	exit:
	WuRRecursiveMutexGive(wur_mutex);
	return res;
}




