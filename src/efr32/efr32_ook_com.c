#include "lib_conf.h"

/* 
*	EFR_32 specific version of OOK frame management API
*	as defined in ook_com.h.
.*/

#ifdef USE_EFR_VERSION

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include PLATFORM_HEADER
#include CONFIGURATION_HEADER

#include "rail.h"
#include "rail_types.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_emu.h"
#include <hal/micro/led.h>
#include <ook_com.h>
#include "gpiointerrupt.h"
#include "rail_config.h"
#include "bsp.h"

#include "ook_wur.h"

#define WAKE_FLAG 0b001
#define ACK_FLAG 0b010
#define DATA_FLAG 0b100
#define DEFAULT_WUR_LEN 16

/******************************************************************************
 * Prototypes
 *****************************************************************************/

void RAILCb_Generic(RAIL_Handle_t railHandle, RAIL_Events_t events);
void radioInit();


/******************************************************************************
 * Private Typedefs
 *****************************************************************************/

#define DEFAULT_WUR_FRAME { 														\
  0b01010101, /*sync word "11" and first byte of 12 bit address "010101010101"*/	\
  0b01010010, /*last nibble of 12 bit address, type "001" AND PARITY "0"*/			\
  0x0C, 	  /*length*/															\
  0x55, 0x33, 0x0F,																	\
  0x55, 0x33, 0x0F,																	\
  0x55, 0x33, 0x0F,																	\
  0x55, 0x33, 0x0F																	\
};


typedef enum ook_tx_status{
	OOK_WUR_TX_STATUS_IDLE = 0,
	OOK_WUR_TX_STATUS_BUSY = 1
}ook_tx_status_t;


typedef struct ButtonArray{
  GPIO_Port_TypeDef   port;
  unsigned int        pin;
} ButtonArray_t;

typedef struct ook_wur_ctxt{
	ook_tx_errors_t tx_result;
	ook_tx_status_t tx_status;
}ook_wur_ctxt_t;

/* will be modified in RAIL ISR */
static volatile ook_wur_ctxt_t ook_wur_ctxt;


/******************************************************************************
 * Private Defines
 *****************************************************************************/

#define LED_TX (0)
#define LED_RX (1)

// Memory manager configuration
#define MAX_BUFFER_SIZE  256

// Minimum allowed size of the TX FIFO
#define RAIL_TX_FIFO_SIZE 64
#define DEFAULT_RAIL_FRAME_SIZE 15

// General application memory sizes
#define APP_MAX_PACKET_LENGTH  (MAX_BUFFER_SIZE - 12) /* sizeof(RAIL_RxPacketInfo_t) == 12) */


#define RAIL_CSMA_CONFIG_WuR_2p4_GHz_OOK_CSMA {                    \
    /* CSMA per WuR OOK on 2.4 GHz OOK, based on the specification on IEEE 802.11-2003
     * for OFDM on mixed bg networks. The maxCCA exponent is reduced from 10 to 8 for
     *  limitations on EFR hardware     */ \
    /* csmaMinBoExp */ 4,   /* 2^4-1 for 0..15 backoffs on 1st try           */ \
    /* csmaMaxBoExp */ 8,   /* 2^8-1 for 0..255 backoffs on 4rd+ tries       */ \
    /* csmaTries    */ 7,   /* 5 tries overall (4 re-tries)                 */ \
    /* ccaThreshold */ -82, /* Sensitivity for 6mbps OFDM                   */ \
    /* ccaBackoff   */ 20, /*  Slot time for mixed mode IEEE 802.11g        */ \
    /* ccaDuration  */ 15, /*  As specified on IEEE 802.11-2003.
    / * TODO: correct it for EFR32 radio turnaround times.                 */ \
    /* csmaTimeout  */ 0,   /* No timeout                                   */ \
}

#define RAIL_WUR_SCHEDULE_INFO {			\
				.priority = 1,				\
				.slipTime = 100,			\
				.transactionTime = 500		\
				}

#define RAIL_WUR_EVENTS (			\
	RAIL_EVENT_CAL_NEEDED			\
	| RAIL_EVENT_TX_PACKET_SENT		\
	| RAIL_EVENT_TX_UNDERFLOW		\
	| RAIL_EVENT_TX_BLOCKED			\
	| RAIL_EVENT_TX_CHANNEL_BUSY	\
	| RAIL_EVENT_TX_ABORTED)


/******************************************************************************
 * Private Static variables
 *****************************************************************************/

static RAIL_CsmaConfig_t csmaTxConf = RAIL_CSMA_CONFIG_WuR_2p4_GHz_OOK_CSMA;

static RAIL_SchedulerInfo_t scheduleTxInfo = RAIL_WUR_SCHEDULE_INFO;

static const RAIL_Events_t events = RAIL_WUR_EVENTS;


RAIL_Handle_t railHandle = NULL;

uint8_t channel = 0;

volatile bool packetTx = true; //go into transfer mode
volatile bool packetRx = false;  //go into receive mode

static uint8_t transmitFIFO[RAIL_TX_FIFO_SIZE] = {0};

static RAILSched_Config_t railSchedState;

static RAIL_Config_t railCfg = {
  .eventsCallback = &RAILCb_Generic,
  .scheduler = &railSchedState,
  .protocol = NULL
};


/******************************************************************************
 * Public Functions
 *****************************************************************************/


wur_errors_t ook_wur_init_context(void)
{

  emberAfCorePrintln("----------- Start of OOK Init! -----------");

  // Initialize Radio
  radioInit();

  // Initialize the PA now that the HFXO is up and the timing is correct
  RAIL_TxPowerConfig_t txPowerConfig = {
#if HAL_PA_2P4_LOWPOWER
    .mode = RAIL_TX_POWER_MODE_2P4_LP,
#else
    .mode = RAIL_TX_POWER_MODE_2P4_HP,
#endif
    .voltage = BSP_PA_VOLTAGE,
    .rampTime = HAL_PA_RAMP,
  };
  emberAfCorePrintln("Configuring RAIL TxPower!");
  if (RAIL_ConfigTxPower(railHandle, &txPowerConfig) != RAIL_STATUS_NO_ERROR) {
	  emberAfCorePrintln("----------- Power conf ERROR! -----------");
	  return WUR_KO;
  }

  emberAfCorePrintln("Set RAIL TxPower!");
  RAIL_SetTxPower(railHandle, HAL_PA_POWER);

  emberAfCorePrintln("Prepare RAIL FIFO!");
  int ires = RAIL_SetTxFifo(railHandle, &transmitFIFO[0], 0, RAIL_TX_FIFO_SIZE);
  if(ires != RAIL_TX_FIFO_SIZE){
	  emberAfCorePrintln("----------- FIFO ERROR %d! -----------", ires);
	  return WUR_KO;
  }
  emberAfCorePrintln("--------- Configured OOK RAIL! -----------");

  GPIO_PinModeSet(WuR_FRAME_PORT, WuR_FRAME_LOC, gpioModePushPull, 0);

  ook_wur_ctxt.tx_status = OOK_WUR_TX_STATUS_IDLE;
  ook_wur_ctxt.tx_result = OOK_WUR_TX_ERROR_SUCCESS;

  return WUR_OK;
}


static wur_errors_t _do_ook_wur_transmit_frame(uint8_t* data, uint8_t len){
	//if a frame is not pending
	if(len > RAIL_TX_FIFO_SIZE){
		emberAfCorePrintln("Frame too large: %d is larger than max frame size %d", len, RAIL_TX_FIFO_SIZE);
		return WUR_KO;
	}

	int ires = RAIL_WriteTxFifo(railHandle, data, len, true);
	if(ires != len){
	  emberAfCorePrintln("----------- TxFIFO ERROR %d! -----------", ires);
	  return WUR_KO;
	}

	if(RAIL_StartCcaCsmaTx(railHandle, channel, 0, &csmaTxConf, &scheduleTxInfo) == RAIL_STATUS_NO_ERROR){
	  GPIO_PinOutSet(WuR_FRAME_PORT, WuR_FRAME_LOC);
	  return WUR_OK;
	} else {
	  emberAfCorePrintln("----------- Send ERROR %d! ------------", ires);
	  return WUR_KO;
	}
}

wur_errors_t ook_wur_transmit_frame(uint8_t* data, uint8_t len){
	int32_t res;

	if(ook_wur_ctxt.tx_status != OOK_WUR_TX_STATUS_IDLE){
		return WUR_KO;
	}

	int8_t retry = 3;
	while(retry > 0){
		ook_wur_ctxt.tx_status = OOK_WUR_TX_STATUS_BUSY;
		res = _do_ook_wur_transmit_frame(data, len);
		if(res != WUR_OK){
			ook_wur_ctxt.tx_status = OOK_WUR_TX_STATUS_IDLE;
			return WUR_KO;
		}

		/* busy wait until transmission (at most a few ms) */
		while(ook_wur_ctxt.tx_status == OOK_WUR_TX_STATUS_BUSY);
		if(ook_wur_ctxt.tx_result == OOK_WUR_TX_ERROR_SUCCESS){
			return WUR_OK;
		}
		retry--;
	}

	return WUR_KO;

}

/******************************************************************************
 * Static Functions
 *****************************************************************************/

static void RAILCb_RadioConfigChanged(RAIL_Handle_t railHandle,
                                      const RAIL_ChannelConfigEntry_t *entry)
{
  // Ensure that the correct PA is configured for use. If it is correct,
  // we don't need to do anything as RAIL library takes care of setting
  // the power level according to channel limits. If the PA needs to change
  // however, the app needs to make that change explicitly and re-set the
  // power.
  RAIL_TxPowerConfig_t txConfig;
  RAIL_GetTxPowerConfig(railHandle, &txConfig);
  if ( entry->baseFrequency > 1000000000UL && txConfig.mode == RAIL_TX_POWER_MODE_SUBGIG ) {
#if HAL_PA_2P4_LOWPOWER
    txConfig.mode = RAIL_TX_POWER_MODE_2P4_LP;
#else
    txConfig.mode = RAIL_TX_POWER_MODE_2P4_HP;
#endif
    RAIL_ConfigTxPower(railHandle, &txConfig);
    RAIL_SetTxPower(railHandle, HAL_PA_POWER);
  } else if ( entry->baseFrequency < 1000000000UL && (txConfig.mode == RAIL_TX_POWER_MODE_2P4_HP
                                                      || txConfig.mode == RAIL_TX_POWER_MODE_2P4_LP)) {
    txConfig.mode = RAIL_TX_POWER_MODE_SUBGIG;
    RAIL_ConfigTxPower(railHandle, &txConfig);
    RAIL_SetTxPower(railHandle, HAL_PA_POWER);
  }
}

void radioInit()
{
  //emberAfCorePrintln("Prepare RAIL handle");
  railHandle = RAIL_Init(&railCfg, NULL);
  if (railHandle == NULL) {
    while (1) ;
  }
  //emberAfCorePrintln("RAIL handle ready!");
  RAIL_ConfigCal(railHandle, RAIL_CAL_ALL_PENDING);
  emberAfCorePrintln("Rail config CAL");
  // Set us to a valid channel for this config and force an update in the main
  // loop to restart whatever action was going on
  RAIL_ConfigChannels(railHandle, channelConfigs[0], RAILCb_RadioConfigChanged);

  RAIL_StateTransitions_t defaultStateTransition = {
    .error = RAIL_RF_STATE_IDLE,
    .success = RAIL_RF_STATE_IDLE
  };

  emberAfCorePrintln("Config RAIL events!");
  RAIL_ConfigEvents(railHandle,
                    events,
                    events);

  RAIL_SetRxTransitions(railHandle, &defaultStateTransition);
  RAIL_SetTxTransitions(railHandle, &defaultStateTransition);

}

void ook_wur_callback(ook_tx_errors_t dest){
	ook_wur_ctxt.tx_status = OOK_WUR_TX_STATUS_IDLE;
	ook_wur_ctxt.tx_result = dest;
}

/******************************************************************************
 * RAIL OOK Callback Implementation
 *****************************************************************************/
void RAILCb_Generic(RAIL_Handle_t railHandle, RAIL_Events_t events)
{
  (void)railHandle;
  if (events & RAIL_EVENT_CAL_NEEDED) {
	// Calibrate if necessary
	RAIL_Calibrate(railHandle, NULL, RAIL_CAL_ALL_PENDING);
  }
  if( events & RAIL_EVENT_TX_BLOCKED){
	RAIL_ResetFifo(railHandle, true, false);
	ook_wur_callback(OOK_WUR_TX_ERROR_FAILED);
	halToggleLed(LED_TX);
  }
  if( events & RAIL_EVENT_TX_CHANNEL_BUSY){
	RAIL_ResetFifo(railHandle, true, false);
	ook_wur_callback(OOK_WUR_TX_ERROR_FAILED);
	halToggleLed(LED_TX);
  }
  if (events & RAIL_EVENT_TX_ABORTED) {
	RAIL_ResetFifo(railHandle, true, false);
	ook_wur_callback(OOK_WUR_TX_ERROR_FAILED);
	halToggleLed(LED_TX);
  }
  if (events & RAIL_EVENT_TX_PACKET_SENT) {
	ook_wur_callback(OOK_WUR_TX_ERROR_SUCCESS);
	halToggleLed(LED_TX);
  }
  if (events & RAIL_EVENT_TX_UNDERFLOW) {
	RAIL_ResetFifo(railHandle, true, false);
	ook_wur_callback(OOK_WUR_TX_ERROR_FAILED);
	halToggleLed(LED_TX);
  }
  GPIO_PinOutClear(WuR_FRAME_PORT, WuR_FRAME_LOC);
  RAIL_YieldRadio(railHandle);
}


#endif
