/*
 * Copyright 2018-2019 NXP.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_ftm.h"
#include "pin_mux.h"

#define FTM0_CH0_PIN            1U
#define FTM0_CH2_PIN            5U
#define PIN2_IDX                2U
#define PIN3_IDX                3U
#define PIN10_IDX               10U
#define PIN11_IDX               11U

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
	BOARD_InitPins();
}

void BOARD_InitPins(void) {
	CLOCK_EnableClock(kCLOCK_PortE); /* Port E Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortC); /* Port C Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortB); /* Port B Clock Gate Control: Clock enabled */

	/*
	 * Enable ADC
	 */
	CLOCK_EnableClock(kCLOCK_Adc0);
	CLOCK_EnableClock(kCLOCK_Adc1);

	/*
	 * Enable Timers for  PWM signals
	 */
	PORT_SetPinMux(PORTC, FTM0_CH0_PIN, kPORT_MuxAlt4); /* PORTC1 is configured as FTM0_CH0 */
	PORT_SetPinMux(PORTC, FTM0_CH2_PIN, kPORT_MuxAlt7); /* PORTC5 is configured as FTM0_CH2 */

	const port_pin_config_t BUTTON_config = {
			kPORT_PullUp, 				/* Internal pull-up resistor is enabled */
			kPORT_FastSlewRate, 		/* Fast slew rate is configured */
			kPORT_PassiveFilterDisable, /* Passive filter is disabled */
			kPORT_OpenDrainDisable, 	/* Open drain is disabled */
			kPORT_HighDriveStrength, 	/* High drive strength is configured */
			kPORT_MuxAsGpio, 			/* Pin is configured as PTA4 */
			kPORT_UnlockRegister 		/* Pin Control Register fields [15:0] are not locked */
	};

	gpio_pin_config_t LED_config = {
			kGPIO_DigitalOutput,
			0U
	};

	/*
	 * LEDs
	 */
	GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, &LED_config);
	PORT_SetPinMux(BOARD_LED_GREEN_PORT, BOARD_LED_GREEN_PIN, kPORT_MuxAsGpio);

	GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, &LED_config);
	PORT_SetPinMux(BOARD_LED_RED_PORT, BOARD_LED_RED_PIN, kPORT_MuxAsGpio);

	GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, &LED_config);
	PORT_SetPinMux(BOARD_LED_BLUE_PORT, BOARD_LED_BLUE_PIN, kPORT_MuxAsGpio);

	/*
	 * Push button
	 */
	PORT_SetPinInterruptConfig(BOARD_SW2_PORT, BOARD_SW2_PIN, kPORT_InterruptFallingEdge);
	EnableIRQ(PORTC_IRQn);
	PORT_SetPinConfig(BOARD_SW2_PORT, BOARD_SW2_PIN, &BUTTON_config);
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
