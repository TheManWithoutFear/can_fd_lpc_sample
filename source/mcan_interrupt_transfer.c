/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_mcan.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_gint.h"
#include "fsl_common.h"
#include "fsl_rng.h"
#include "fsl_usart.h"
#include "stdlib.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define BOARD_MASTER		(0U)
#define BOARD_ECHO			(1U)
#define CAN_FRAMES_QTY		(100U)
#define CAN_ARB_BRPS		(500000U)
#define CAN_DATA_BRPS		(3750000U)

#define PRINT_MESSAGES		(0U)
#define PRINT_RESULTS		(1U)
#define	MESSAGE_DELAY		(3U)

#define USE_CANFD (1U)
/*
 *    CAN_DATASIZE   DLC    BYTES_IN_MB
 *    8              8      kMCAN_8ByteDatafield
 *    12             9      kMCAN_12ByteDatafield
 *    16             10     kMCAN_16ByteDatafield
 *    20             11     kMCAN_20ByteDatafield
 *    24             12     kMCAN_24ByteDatafield
 *    32             13     kMCAN_32ByteDatafield
 *    48             14     kMCAN_48ByteDatafield
 *    64             15     kMCAN_64ByteDatafield
 *
 *  CAN data size (pay load size), DLC and Bytes in Message buffer must align.
 *
 */
#define DLC         (15)
#define BYTES_IN_MB kMCAN_64ByteDatafield
/* If not define USE_CANFD or define it 0, CAN_DATASIZE should be 8. */
#define CAN_DATASIZE (64U)
/* If user need to auto execute the improved timming configuration. */
#define USE_IMPROVED_TIMING_CONFIG (1U)
#define EXAMPLE_MCAN_IRQHandler    CAN0_IRQ0_IRQHandler
#define EXAMPLE_MCAN_IRQn          CAN0_IRQ0_IRQn
#define EXAMPLE_MCAN               CAN0
#define MCAN_CLK_FREQ              CLOCK_GetMCanClkFreq()
#define MCU_CLK_FREQ				60000000
#define STDID_OFFSET               (0U)
#define MSG_RAM_BASE               0x04000000U
#define STD_FILTER_OFS 0x0
#define RX_FIFO0_OFS   0x10U
#define TX_BUFFER_OFS  0x20U
#define MSG_RAM_SIZE   (TX_BUFFER_OFS + 8 + CAN_DATASIZE)

#define DEMO_GINT0_PORT kGINT_Port1
/* Select one input, active low for GINT0 */
#define DEMO_GINT0_POL_MASK 	~(1U << BOARD_SW3_GPIO_PIN)
#define DEMO_GINT0_ENA_MASK 	(1U << BOARD_SW3_GPIO_PIN)
#define BOARD_LED_PORT 			BOARD_LED_BLUE_GPIO_PORT
#define BOARD_LED_PIN  			BOARD_LED_BLUE_GPIO_PIN
#define BOARD_LED_PIN_GREEN 	BOARD_LED_GREEN_GPIO_PIN

#define RNG_EXAMPLE_RANDOM_NUMBERS     (64)
#define RNG_EXAMPLE_RANDOM_BYTES       (64)
#define RNG_EXAMPLE_RANDOM_NUMBER_BITS (RNG_EXAMPLE_RANDOM_NUMBERS * 8 * sizeof(uint32_t))

#define DEMO_USART          USART3
#define DEMO_USART_CLK_SRC  kCLOCK_Flexcomm0
#define DEMO_USART_CLK_FREQ CLOCK_GetFlexCommClkFreq(0U)

#define EXAMPLE_ADDRESS 0x7EU
#define TRANSFER_SIZE   8U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* USART user callback */
void USART_UserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if (MCU_CLK_FREQ == 60000000)

uint32_t possible_arbitration_boudrates_qty = 24;
uint32_t possible_arbitration_boudrates[24] = { 100000, 120000, 125000, 150000,
		156250, 160000, 187500, 200000, 234375, 240000, 250000, 300000, 312500,
		375000, 400000, 468750, 480000, 500000, 600000, 625000, 750000, 800000,
		937500, 1000000, };

uint32_t possible_data_boudrates_qty = 35;
uint32_t possible_data_boudrates[35] = { 100000, 120000, 125000, 150000, 156250,
		160000, 187500, 200000, 234375, 240000, 250000, 300000, 312500, 375000,
		400000, 468750, 480000, 500000, 600000, 625000, 750000, 800000, 937500,
		1000000, 1200000, 1250000, 1500000, 1875000, 2000000, 2400000, 2500000,
		3000000, 3750000, 4000000, 5000000, };

#elif (MCU_CLK_FREQ == 96000000)

uint32_t possible_arbitration_boudrates_qty = 27;
uint32_t possible_arbitration_boudrates[27] = { 100000, 120000, 125000, 128000,
		150000, 153600, 160000, 187500, 192000, 200000, 240000, 250000, 256000,
		300000, 320000, 375000, 384000, 400000, 480000, 500000, 600000, 640000,
		750000, 768000, 800000, 960000, 1000000, };

uint32_t possible_data_boudrates_qty = 39;
uint32_t possible_data_boudrates[39] =
		{ 100000, 120000, 125000, 128000, 150000, 153600, 160000, 187500,
				192000, 200000, 240000, 250000, 256000, 300000, 320000, 375000,
				384000, 400000, 480000, 500000, 600000, 640000, 750000, 768000,
				800000, 960000, 1000000, 1200000, 1280000, 1500000, 1600000,
				1920000, 2000000, 2400000, 3000000, 3200000, 3840000, 4000000,
				4800000, };

#endif

volatile bool txComplete = false;
volatile bool rxComplete = false;
mcan_tx_buffer_frame_t txFrame;
mcan_rx_buffer_frame_t rxFrame;
uint8_t tx_data[CAN_DATASIZE];
uint8_t rx_data[CAN_DATASIZE];
mcan_handle_t mcanHandle;
mcan_buffer_transfer_t txXfer;
mcan_fifo_transfer_t rxXfer;
uint32_t txIdentifier;
uint32_t rxIdentifier;
#ifndef MSG_RAM_BASE
SDK_ALIGN(uint8_t msgRam[MSG_RAM_SIZE], 1U << CAN_MRBA_BA_SHIFT);
#else
#define msgRam MSG_RAM_BASE
#endif
volatile bool SWpress = false;
volatile uint32_t g_systickCounter;
volatile uint32_t error_counter = 0;

uint8_t g_txBuffer[TRANSFER_SIZE * 2U] = {0};
uint8_t g_rxBuffer[TRANSFER_SIZE + 1U] = {0};
volatile bool uart_txComplete               = false;
volatile bool uart_rxComplete               = false;
usart_handle_t g_usartHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/

/* USART user callback */
void USART_UserCallback(USART_Type *base, usart_handle_t *handle,
		status_t status, void *userData) {
	if (kStatus_USART_TxIdle == status) {
		uart_txComplete = true;
	}
	if (kStatus_USART_RxIdle == status) {
		uart_rxComplete = true;
	}
}

uint8_t GetCELValue(CAN_Type *base) {
	uint8_t counter = 0;
	counter = (base->ECR & CAN_ECR_CEL_MASK) >> CAN_ECR_CEL_SHIFT;
	return counter;
}

uint8_t GetLECValue(CAN_Type *base) {
	uint8_t code = 0;
	code = (base->PSR & CAN_PSR_LEC_MASK) >> CAN_PSR_LEC_SHIFT;
	return code;
}

bool GetBOStatus(CAN_Type *base) {
	bool BOStatus = (base->PSR & CAN_PSR_BO_MASK) >> CAN_PSR_BO_SHIFT;
	return BOStatus;
}

void BORecovery(CAN_Type *base) {
	base->TXBCR |= CAN_TXBCR_CR_MASK;
	base->CCCR &= ~CAN_CCCR_INIT_MASK;
	while ((base->CCCR & CAN_CCCR_INIT_MASK) != 0U) {
	}
//    MCAN_EnterNormalMode(base);
	while (GetLECValue(base) == 0x5 || GetBOStatus(base) == 0x1) {
	}
	PRINTF("Buss off recovery successful \r\n");
}

void SysTick_Handler(void) {
	if (g_systickCounter != 0U) {
		g_systickCounter--;
	}
}

void SysTick_DelayTicks(uint32_t n) {
	g_systickCounter = n;
	while (g_systickCounter != 0U) {
	}
}

/*!
 * @brief Call back for GINT0 event
 */
void gint0_callback(void) {
	PRINTF(">> USR button pressed.\r\n");
	if (!SWpress) {
		SWpress = true;
	}
}

/*!
 * @brief MCAN Call Back function
 */
static void mcan_callback(CAN_Type *base, mcan_handle_t *handle,
		status_t status, uint32_t result, void *userData) {
	switch (status) {
	case kStatus_MCAN_RxFifo0Idle: {
		rxComplete = true;
//            GPIO_PortClear(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN);
	}
		break;

	case kStatus_MCAN_TxIdle: {
		txComplete = true;
//            SWpress = false;
	}
		break;

	case kMCAN_BusOffIntFlag: {
		SWpress = false;
	}

	default:
		break;
	}
}

/*!
 * @brief Main function
 */
int main(void) {
	mcan_config_t mcanConfig;
	mcan_frame_filter_config_t rxFilter;
	mcan_ext_filter_element_config_t stdFilter;
	mcan_rx_fifo_config_t rxFifo0;
	mcan_tx_buffer_config_t txBuffer;
	uint8_t data[18];
	status_t status = kStatus_Fail;

	/* Init output LED GPIO. */
	GPIO_PortInit(GPIO, BOARD_LED_PORT);
	BOARD_InitPins();
	BOARD_BootClockFRO12M();
	//BOARD_BootClockFROHF96M();
	//BOARD_BootClockPLL96M();
	BOARD_InitDebugConsole();
	RNG_Init(RNG);

	SystemCoreClockUpdate();

	/* Initialize board hardware. */
	/* attach 12 MHz clock to FLEXCOMM0 (debug console) */
	CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

	/* Initialize GINT0 */
	GINT_Init(GINT0);
	/* Setup GINT0 for edge trigger, "OR" mode */
	GINT_SetCtrl(GINT0, kGINT_CombineOr, kGINT_TrigEdge, gint0_callback);
	/* Select pins & polarity for GINT0 */
	GINT_ConfigPins(GINT0, DEMO_GINT0_PORT, DEMO_GINT0_POL_MASK,
	DEMO_GINT0_ENA_MASK);
	/* Enable callbacks for GINT0 */
	GINT_EnableCallback(GINT0);

	/* Initialize usart instance. */
	/*
	 * usartConfig->baudRate_Bps = 115200U;
	 * usartConfig->parityMode = kUSART_ParityDisabled;
	 * usartConfig->stopBitCount = kUSART_OneStopBit;
	 * usartConfig->bitCountPerChar = kUSART_8BitsPerChar;
	 * usartConfig->loopback = false;
	 * usartConfig->enableTx = false;
	 * usartConfig->enableRx = false;
	 */
	usart_config_t config;
	USART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableTx = true;
	config.enableRx = true;
	USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);
	USART_Enable9bitMode(DEMO_USART, true);
	/* Configure address. */
	USART_SetMatchAddress(DEMO_USART, EXAMPLE_ADDRESS);
	/* Enable match address. */
	USART_EnableMatchAddress(DEMO_USART, true);

	/* Set up transfer data */
	for (uint8_t i = 0U; i < TRANSFER_SIZE * 2U; i++) {
		g_txBuffer[i] = i;
	}

//	for (uint8_t i = 0U; i < TRANSFER_SIZE + 1U; i++) {
//		g_rxBuffer[i] = 0;
//	}

	/* Create usart handle. */
	USART_TransferCreateHandle(DEMO_USART, &g_usartHandle, USART_UserCallback,
			NULL);
	/* Start receiving. */
	usart_transfer_t g_receiveXfer;
	usart_transfer_t g_sendXfer;
    g_receiveXfer.data     = g_rxBuffer;
    g_receiveXfer.dataSize = TRANSFER_SIZE + 1U;
    USART_TransferReceiveNonBlocking(DEMO_USART, &g_usartHandle, &g_receiveXfer, NULL);


	/* Set systick reload value to generate 1ms interrupt */
	if (SysTick_Config(SystemCoreClock / 1000U)) {
		while (1) {
		}
	}

	GPIO_PortSet(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN);
//	SDK_DelayAtLeastUs(1000, MCU_CLK_FREQ);
	SysTick_DelayTicks(100);
	GPIO_PortClear(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN);

	/* Select mailbox ID. */
#if (defined(BOARD_MASTER) && BOARD_MASTER)
	txIdentifier = 0xFDE321U;
	rxIdentifier = 0x123EDFU;
#elif (defined(BOARD_ECHO) && BOARD_ECHO)
        txIdentifier = 0x123EDFU;
        rxIdentifier = 0xFDE321U;
#endif

	MCAN_GetDefaultConfig(&mcanConfig);

#if (defined(USE_CANFD) && USE_CANFD)
	mcanConfig.enableCanfdNormal = true;
	mcanConfig.enableCanfdSwitch = true;
//    mcanConfig.enableDAR = true;
	mcanConfig.baudRateA = CAN_ARB_BRPS;
	mcanConfig.baudRateD = CAN_DATA_BRPS;
#endif

#if (defined(USE_IMPROVED_TIMING_CONFIG) && USE_IMPROVED_TIMING_CONFIG)
	mcan_timing_config_t timing_config;
	memset(&timing_config, 0, sizeof(timing_config));
#if (defined(USE_CANFD) && USE_CANFD)
	if (MCAN_FDCalculateImprovedTimingValues(mcanConfig.baudRateA,
			mcanConfig.baudRateD, MCAN_CLK_FREQ, &timing_config)) {
//        /* Update the improved timing configuration*/
//    	mcanConfig.timingConfig.preDivider = 0x4U;
//    	//mcanConfig.timingConfig.rJumpwidth = 0U;
//    	mcanConfig.timingConfig.seg1 = 0xDU;
//    	mcanConfig.timingConfig.seg2 = 0x4U;
//
//    	mcanConfig.timingConfig.datapreDivider = 0x4U;
//    	//mcanConfig.timingConfig.datarJumpwidth = 0U;
//    	mcanConfig.timingConfig.dataseg1 = 0x5U;
//    	mcanConfig.timingConfig.dataseg2 = 0x4U;
		timing_config.enableTDC = false;

		memcpy(&(mcanConfig.timingConfig), &timing_config,
				sizeof(mcan_timing_config_t));
	} else {
		PRINTF(
				"No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
	}
#else
    if (MCAN_CalculateImprovedTimingValues(mcanConfig.baudRateA, MCAN_CLK_FREQ, &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(mcanConfig.timingConfig), &timing_config, sizeof(mcan_timing_config_t));
    }
    else
    {
        PRINTF("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#endif
#endif

	MCAN_Init(EXAMPLE_MCAN, &mcanConfig, MCAN_CLK_FREQ);

	EXAMPLE_MCAN->TDCR = CAN_TDCR_TDCO(0x04U);

	/* Create MCAN handle structure and set call back function. */
	MCAN_TransferCreateHandle(EXAMPLE_MCAN, &mcanHandle, mcan_callback, NULL);
	MCAN_EnableInterrupts(EXAMPLE_MCAN, 0U, CAN_IE_BOE_MASK);
	/* Set Message RAM base address and clear to avoid BEU/BEC error. */
	MCAN_SetMsgRAMBase(EXAMPLE_MCAN, (uint32_t) msgRam);
	memset((void*) msgRam, 0, MSG_RAM_SIZE * sizeof(uint8_t));

//    /* STD filter config. */
//    rxFilter.address  = STD_FILTER_OFS;
//    rxFilter.idFormat = kMCAN_FrameIDExtend;
//    rxFilter.listSize = 1U;
//    rxFilter.nmFrame  = kMCAN_reject0;
//    rxFilter.remFrame = kMCAN_rejectFrame;
//    MCAN_SetFilterConfig(EXAMPLE_MCAN, &rxFilter);

//    stdFilter.efec = kMCAN_storeinFifo0;
//    /* Classic filter mode, only filter matching ID. */
//    stdFilter.eft   = kMCAN_classic;
//    stdFilter.efid1 = rxIdentifier;
//    stdFilter.efid2 = 0x7FFU;
//    MCAN_SetEXTFilterElement(EXAMPLE_MCAN, &rxFilter, &stdFilter, 0);

	/* RX fifo0 config. */
	rxFifo0.address = RX_FIFO0_OFS;
	rxFifo0.elementSize = 1U;
	rxFifo0.watermark = 0;
	rxFifo0.opmode = kMCAN_FifoBlocking;
#if (defined(USE_CANFD) && USE_CANFD)
	rxFifo0.datafieldSize = BYTES_IN_MB;
#else
    rxFifo0.datafieldSize = kMCAN_8ByteDatafield;
#endif

	MCAN_SetRxFifo0Config(EXAMPLE_MCAN, &rxFifo0);

	/* TX buffer config. */
	memset(&txBuffer, 0, sizeof(txBuffer));
	txBuffer.address = TX_BUFFER_OFS;
	txBuffer.dedicatedSize = 1U;
	txBuffer.fqSize = 0;
#if (defined(USE_CANFD) && USE_CANFD)
	txBuffer.datafieldSize = BYTES_IN_MB;
#else
    txBuffer.datafieldSize = kMCAN_8ByteDatafield;
#endif

	MCAN_SetTxBufferConfig(EXAMPLE_MCAN, &txBuffer);

	/* Finish software initialization and enter normal mode, synchronizes to
	 CAN bus, ready for communication */
	MCAN_EnterNormalMode(EXAMPLE_MCAN);

#if (defined(BOARD_MASTER) && BOARD_MASTER)
	PRINTF("Current board type: MASTER\r\n");
	PRINTF("Press USR button to start test sequence:\r\n");
#elif (defined(BOARD_ECHO) && BOARD_ECHO)
 	   PRINTF("Current board type: ECHO\r\n");
 	   PRINTF("Wait for master board to start test sequence\r\n");
#else
 	  PRINTF("Board type not chosen! Recompile code with right directive!\r\n");
#endif

	while (1) {
#if (defined(BOARD_MASTER) && BOARD_MASTER)

		if (SWpress) {
			    USART_SendAddress(DEMO_USART, EXAMPLE_ADDRESS);
		    PRINTF("USART will send the other piece of data out:\n\r");
			for (uint8_t i = TRANSFER_SIZE; i < TRANSFER_SIZE * 2U; i++) {
				if (i % 8U == 0U) {
					PRINTF("\r\n");
				}
				PRINTF("0x%2x  ", g_txBuffer[i]);
			}
			PRINTF("\r\n\r\n");
			g_sendXfer.data = &g_txBuffer[TRANSFER_SIZE];
			g_sendXfer.dataSize = TRANSFER_SIZE;
			USART_TransferSendNonBlocking(DEMO_USART, &g_usartHandle,
					&g_sendXfer);

			/* Waiting for transfer completion */
			while ((uart_txComplete == false) || (uart_rxComplete == false)) {
			}
			bool success = true;


			PRINTF("TEST STARTED.\r\n");
			PRINTF("MCU Clock frequency: %0d Hz.\r\n\r\n", CLOCK_GetCoreSysClkFreq());
			for (uint32_t i = 0; i < CAN_FRAMES_QTY; i++) {
//            GETCHAR();
				/* Config TX frame data. */
				memset(tx_data, 0, sizeof(uint8_t) * CAN_DATASIZE);

				status = RNG_GetRandomData(RNG, &data,
						RNG_EXAMPLE_RANDOM_BYTES);

				if (status != kStatus_Success) {
					return status;
				}

				for (uint8_t cnt = 0; cnt < CAN_DATASIZE; cnt++) {
					tx_data[cnt] = data[cnt];
				}

				tx_data[0] = (i & 0xFF00) >> 8;
				tx_data[1] = i & 0xFF;
				txFrame.xtd = kMCAN_FrameIDExtend;
				txFrame.rtr = kMCAN_FrameTypeData;
				txFrame.id = txIdentifier << STDID_OFFSET;
				txFrame.data = tx_data;
				txFrame.size = CAN_DATASIZE;
#if (defined(USE_CANFD) && USE_CANFD)
				txFrame.fdf = 1;
				txFrame.brs = 1;
				txFrame.dlc = DLC;
#else
            txFrame.fdf  = 0;
            txFrame.brs  = 0;
            txFrame.dlc  = 8U;
#endif
				txXfer.frame = &txFrame;
				txXfer.bufferIdx = 0;
				MCAN_TransferSendNonBlocking(EXAMPLE_MCAN, &mcanHandle,
						&txXfer);

				while (!txComplete) {
					if ((EXAMPLE_MCAN->PSR & CAN_PSR_BO_MASK)
							>> CAN_PSR_BO_SHIFT)
						break;
				}
				txComplete = false;

				/* Start receive data through Rx FIFO 0. */
				memset(rx_data, 0, sizeof(uint8_t) * CAN_DATASIZE);
				/* the MCAN engine can't auto to get rx payload size, we need set it. */
				rxFrame.size = CAN_DATASIZE;
				rxXfer.frame = &rxFrame;
				MCAN_TransferReceiveFifoNonBlocking(EXAMPLE_MCAN, 0,
						&mcanHandle, &rxXfer);

				/* Wait until message received. */
				while (!rxComplete) {
					if ((EXAMPLE_MCAN->PSR & CAN_PSR_BO_MASK)
							>> CAN_PSR_BO_SHIFT)
						break;
				}
				rxComplete = false;

				if (GetBOStatus(EXAMPLE_MCAN)) {
					PRINTF("TEST ENDED DUE TO BUSS OFF,\r\n");
					PRINTF("ERROR Counter:%0d\r\n", error_counter);
					PRINTF("SUCCESSFUL FRAME NUMBER:%0d\r\n", i);
					SWpress = false;
					BORecovery(EXAMPLE_MCAN);
					break;
				}

				/* After call the API of rMCAN_TransferReceiveFifoNonBlocking success, we can
				 * only get a point (rxFrame.data) to the fifo reading entrance.
				 * Copy the received frame data from the FIFO by the pointer(rxFrame.data). */
				memcpy(rx_data, rxFrame.data, rxFrame.size);

#if (defined(PRINT_MESSAGES) && PRINT_MESSAGES)
            PRINTF("Received Frame ID: 0x%x\r\n", rxFrame.id >> STDID_OFFSET);
            PRINTF("Received Frame DATA: ");
            cnt = 0;
            while (cnt < rxFrame.size)
            {
                PRINTF("0x%x ", rx_data[cnt++]);
            }
            PRINTF("\r\n FRAME NUMBER:%0d\r\n", i);

            PRINTF("\r\n");
            PRINTF("Press any key to trigger the next transmission!\r\n\r\n");

//            PRINTF("ERROR Counter:0x%08x\r\n", GetCELValue(EXAMPLE_MCAN));
            	if (GetCELValue(EXAMPLE_MCAN) != 0)
            	{
            		error_counter++;
            	}
            	 PRINTF("ERROR Counter:0x%0x\r\n", error_counter);
#endif

				if (GetCELValue(EXAMPLE_MCAN) != 0) {
					error_counter++;
				}
#if (defined(PRINT_RESULTS) && PRINT_RESULTS)
				SysTick_DelayTicks(MESSAGE_DELAY);
#endif

			}

#if (defined(PRINT_RESULTS) && PRINT_RESULTS)
			PRINTF("TEST ENDED,\r\n");
			PRINTF("ERROR Counter:%0d\r\n", error_counter);
#endif
			SWpress = false;
		}
#elif (defined(BOARD_ECHO) && BOARD_ECHO)


		USART_TransferReceiveNonBlocking(DEMO_USART, &g_usartHandle,
				&g_receiveXfer, NULL);
		PRINTF("USART received data:\n\r");
		for (uint8_t i = 0; i < TRANSFER_SIZE; i++) {
			if (i % 8U == 0U) {
				PRINTF("\r\n");
				GPIO_PortToggle(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN);
			}
			PRINTF("0x%2x  ", g_rxBuffer[i + 1U]);
		}
		PRINTF("\r\n\r\n");

            memset(rx_data, 0, sizeof(uint8_t) * CAN_DATASIZE);
            /* the MCAN engine can't auto to get rx payload size, we need set it. */
            rxFrame.size = CAN_DATASIZE;
            rxXfer.frame = &rxFrame;
            MCAN_TransferReceiveFifoNonBlocking(EXAMPLE_MCAN, 0, &mcanHandle, &rxXfer);
            while (!rxComplete)
            {
            	if ((EXAMPLE_MCAN->PSR & CAN_PSR_BO_MASK) >> CAN_PSR_BO_SHIFT)
            	            		break;
            }
            rxComplete = false;
            GPIO_PortSet(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN);
            /* After call the API of rMCAN_TransferReceiveFifoNonBlocking success, we can
             * only get a point (rxFrame.data) to the fifo reading entrance.
             * Copy the received frame data from the FIFO by the pointer(rxFrame.data). */
            memcpy(rx_data, rxFrame.data, rxFrame.size);
#if (defined(PRINT_MESSAGES) && PRINT_MESSAGES)
            PRINTF("Received Frame ID: 0x%x\r\n", rxFrame.id >> STDID_OFFSET);
            PRINTF("Received Frame DATA: ");

            cnt = 0;
            while (cnt < rxFrame.size)
            {
                PRINTF("0x%x ", rx_data[cnt++]);
            }
            PRINTF("\r\n");
#endif

            SysTick_DelayTicks(MESSAGE_DELAY);
            GPIO_PortClear(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN);
            /* Copy received frame data to tx frame. */
            memcpy(tx_data, rx_data, CAN_DATASIZE);

            txFrame.xtd      = rxFrame.xtd;
            txFrame.rtr      = rxFrame.rtr;
            txFrame.fdf      = rxFrame.fdf;
            txFrame.brs      = rxFrame.brs;
            txFrame.dlc      = rxFrame.dlc;
            txFrame.id       = txIdentifier << STDID_OFFSET;
            txFrame.data     = tx_data;
            txFrame.size     = rxFrame.size;
            txXfer.frame     = &txFrame;
            txXfer.bufferIdx = 0;

            MCAN_TransferSendNonBlocking(EXAMPLE_MCAN, &mcanHandle, &txXfer);

            while (!txComplete)
            {
            	if ((EXAMPLE_MCAN->PSR & CAN_PSR_BO_MASK) >> CAN_PSR_BO_SHIFT)
            	            		break;
            }
            txComplete = false;
#if (defined(PRINT_MESSAGES) && PRINT_MESSAGES)
            PRINTF("Wait Node A to trigger the next transmission!\r\n\r\n");
#endif
            /* Delay 1000 ms */
//            		GPIO_PortSet(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN);
//                    SysTick_DelayTicks(100U);
//                    GPIO_PortClear(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN);
#endif
	}
}
