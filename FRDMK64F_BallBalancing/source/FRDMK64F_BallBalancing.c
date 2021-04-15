/*
 * FRDMK64F - Ball Balancing
 *
 * Copyright (C) 2021 Luca D'Onofrio.
 *
 * This file is part of 'Ball Balancing' Project
 *
 * 'Ball Balancing' is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * 'Ball Balancing' is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "MK64F12.h"

#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "fsl_adc16.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "control.h"

/******************************************************************************
 * Defines
 ******************************************************************************/

/*-----------------------------------------------------------------------------
 * Connections
 -----------------------------------------------------------------------------*/
/*
 * FTM CH0 --> Servo X:
 *   - PWM              -->  PORTC_1
 *
 * FTM CH2 --> Servo Y:
 *   - PWM              -->  PORTC_5
 *
 * ADC0, ADC1 --> Touch screen:
 *   - Y-   			-->  PORTB_2
 *   - X+   			-->  PORTB_3
 *   - Y+   			-->  PORTB_10
 *   - X-  				-->  PORTB_11
 *
 * GPIO -> Board LEDs:
 *   - Red   			-->  PORTB_22
 *   - Green      		-->  PORTE_26
 *   - Blue				-->	 PORTB_21
 *
 * GPIO -> Push button:
 *   - SW2      		-->  PORTC_6
 *
 */

#define CONTROLLER_DELAY		10U		// Controller loop delay

#define SERVO_NUMBER	 		2U		// Number of servo used
#define SERVO_FREQUENCY	 		50U		// Servo frequency
#define SERVO_TIME		 		20.0f	// 20ms (Datasheet: Hitex HS-325HB)
#define SERVO_MIN_PULSE	 		0.9f	// 0.9us ~ 1ms (Datasheet: Hitex HS-325HB)
#define SERVO_MAX_PULSE	 		2.10f	// 2.1us ~ 2ms (Datasheet: Hitex HS-325HB)
#define SERVO_MAX_ANGLE 		100U	// Max servo angle
#define SERVO_X_CHANNEL 		0U		// X servo PWM channel
#define SERVO_Y_CHANNEL 		2U		// Y servo PWM channel

#define SCREEN_WIDTH			640U	// Touch screen width
#define SCREEN_HEIGHT			480U	// Touch screen height

#define ADC16_Xp_CHANNEL 		13U		// X+ touch screen ADC channel
#define ADC16_Yp_CHANNEL 		14U		// Y- touch screen ADC channel

#define PIN_Ym_IDX               2U		// Y- touch screen pin
#define PIN_Xp_IDX               3U		// X+ touch screen pin
#define PIN_Yp_IDX              10U		// Y+ touch screen pin
#define PIN_Xm_IDX              11U		// X- touch screen pin

#define TS_ADC_MINX 			7400	// Measured ADC value when X ~= 0
#define TS_ADC_MAXX 			54000	// Measured ADC value when X ~= SCREEN_WIDTH
#define TS_ADC_MINY 			10200	// Measured ADC value when Y ~= 0
#define TS_ADC_MAXY 			46000	// Measured ADC value when Y ~= SCREEN_WIDTH

#define NO_TOUCH_TIMEOUT		2000U	// Timeout to reset the plate when no touch is received

#define CENTER					0U		// Mode: set to center
#define CIRCLE					1U		// Mode: move along a circle
#define VERTEX					2U		// Mode: move to the vertex of a square
#define MAX_MODE				3U		// Available modes
#define TIMEOUT_MODE_CIRLE 		100U	// Timeout to update the circle path
#define TIMEOUT_MODE_VERTEX 	4000U	// Timeout to keep the ball on a vertex

/*
 * Modes constants
 */
#define PI 						3.14f
#define ROTATION_RADIUS			130U
#define ROTATION_ANGLE_STEP		10U

#define OFFSET_X				150U
#define OFFSET_Y				120U

/*
 * Touch screen macros
 */
#define Read_TouchScreenY()	Read_TouchScreen(ADC1, ADC16_Yp_CHANNEL, PIN_Yp_IDX, PIN_Ym_IDX, PIN_Xp_IDX, PIN_Xm_IDX)
#define Read_TouchScreenX()	Read_TouchScreen(ADC0, ADC16_Xp_CHANNEL, PIN_Xp_IDX, PIN_Xm_IDX, PIN_Yp_IDX, PIN_Ym_IDX)

/******************************************************************************
 * Prototypes
 ******************************************************************************/
static void ControlTask(void *pvParameters);

/******************************************************************************
 * Internal Variables
 ******************************************************************************/
static ControlInit controlInit = {
 		0.23f, 0.0f, -0.090f,			// X PID initialization
 		0.25f, 0.0f, -0.095f,			// Y PID initialization
 		0.05f, 0.2f,					// X Kalman filter initialization
 		0.05f, 0.2f,					// Y Kalman filter initialization
 };

static uint8_t setPointMode = CENTER;
static uint16_t modeTimeout = 0;
static long startModeTime = 0;
static long touchTime = 0;
static uint8_t vertex = 0;
static int16_t rotationAngle = 0;

/******************************************************************************
 * Functions
 ******************************************************************************/

/*
 * Command the servo by changing the PWM duty cycle
 */
static void Command_Servo(ftm_chnl_t channel, int8_t angle) {
	uint32_t mod = FTM0->MOD; // 20ms = 37499
	uint32_t pulseStart = (uint32_t) (SERVO_MIN_PULSE * (float) (mod / SERVO_TIME));
	uint32_t pulseEnd = (uint32_t) (SERVO_MAX_PULSE * (float) (mod / SERVO_TIME));

	if (angle > (int8_t) (SERVO_MAX_ANGLE))
		angle = (int8_t) SERVO_MAX_ANGLE;
	else if (angle < (int8_t) (-SERVO_MAX_ANGLE))
		angle = (int8_t) (-SERVO_MAX_ANGLE);

	// PWM pulse width, the uint of this value is timer ticks.
	uint32_t pulse = map(angle, (int8_t)(-SERVO_MAX_ANGLE),
			(int8_t)(SERVO_MAX_ANGLE), pulseStart, pulseEnd);

	if (FTM0->CONTROLS[(ftm_chnl_t) channel].CnV != pulse) {
		FTM0->CONTROLS[(ftm_chnl_t) channel].CnV = pulse;

		/* Software trigger to update registers */
		FTM_SetSoftwareTrigger(FTM0, true);
		FTM_UpdateChnlEdgeLevelSelect(FTM0, channel, kFTM_HighTrue);
	}
}

/*
 * Read the touch screen values for a generic coordinate.
 *
 * It works like this:
 *
 *  - Measure axis Voltage on pin 'pinMeasure'
 *  - Make pin 'pinTristate' tristate (high impedence)
 *  - Form a voltage divider in 'pinDividerHigh'(Vref) and 'pinDividerLow'(GND)
 *  - Read the ADC from 'pinMeasure' pin
 */
static uint32_t Read_TouchScreen(ADC_Type *adc, uint32_t channel, uint32_t pinMeasure,
		uint32_t pinTristate, uint32_t pinDividerHigh, uint32_t pinDividerLow) {
	gpio_pin_config_t pin_config;

	/* Measure voltage on 'pinMeasure' */
	PORT_SetPinMux(PORTB, pinMeasure, kPORT_PinDisabledOrAnalog);

	/* 'pinTristate' Tristate */
	pin_config.pinDirection = kGPIO_DigitalInput;
	pin_config.outputLogic = 0U;
	GPIO_PinInit(GPIOB, pinTristate, &pin_config);
	GPIO_PortClear(GPIOB, 1U << pinTristate);
	PORT_SetPinMux(PORTB, pinTristate, kPORT_MuxAsGpio);

	/* Form a voltage divider in 'pinDividerHigh'(Vref) and 'pinDividerLow'(GND) */
	pin_config.pinDirection = kGPIO_DigitalOutput;
	pin_config.outputLogic = 0U;
	GPIO_PinInit(GPIOB, pinDividerLow, &pin_config);
	GPIO_PinInit(GPIOB, pinDividerHigh, &pin_config);

	PORT_SetPinMux(PORTB, pinDividerLow, kPORT_MuxAsGpio);
	PORT_SetPinMux(PORTB, pinDividerHigh, kPORT_MuxAsGpio);

	GPIO_PortSet(GPIOB, 1U << pinDividerHigh);
	GPIO_PortClear(GPIOB, 1U << pinDividerLow);

	/* Read voltage from 'pinMeasure' */
	adc16_channel_config_t adc16ChannelConfigStruct;
	adc16ChannelConfigStruct.channelNumber = channel;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
	adc16ChannelConfigStruct.enableDifferentialConversion = false;

	/* Allow voltage to settle */
	vTaskDelay(10 / portTICK_PERIOD_MS);

	ADC16_SetChannelConfig(adc, 0U, &adc16ChannelConfigStruct);
	while (0U
			== (kADC16_ChannelConversionDoneFlag
					& ADC16_GetChannelStatusFlags(adc, 0U))) {
	}

	/* Reset pins */
	GPIO_PortClear(GPIOB, 1U << pinDividerHigh);
	PORT_SetPinMux(PORTB, pinMeasure, kPORT_PinDisabledOrAnalog);
	PORT_SetPinMux(PORTB, pinDividerHigh, kPORT_PinDisabledOrAnalog);
	PORT_SetPinMux(PORTB, pinDividerLow, kPORT_PinDisabledOrAnalog);
	PORT_SetPinMux(PORTB, pinTristate, kPORT_PinDisabledOrAnalog);

	return ADC16_GetChannelConversionValue(adc, 0U);
}

/*
 * Initialize PWM channels for the servo
 */
static void Init_Servos(void) {
	ftm_config_t ftmInfo;
	ftm_chnl_pwm_signal_param_t ftmParam[SERVO_NUMBER];

	ftmParam[0].chnlNumber = (ftm_chnl_t) SERVO_X_CHANNEL;
	ftmParam[0].level = kFTM_HighTrue;
	ftmParam[0].dutyCyclePercent = 0U;
	ftmParam[0].firstEdgeDelayPercent = 0U;
	ftmParam[0].enableDeadtime = false;

	ftmParam[1].chnlNumber = (ftm_chnl_t) SERVO_Y_CHANNEL;
	ftmParam[1].level = kFTM_HighTrue;
	ftmParam[1].dutyCyclePercent = 0U;
	ftmParam[1].firstEdgeDelayPercent = 0U;
	ftmParam[1].enableDeadtime = false;

	FTM_GetDefaultConfig(&ftmInfo);
	ftmInfo.prescale = kFTM_Prescale_Divide_32;

	/* Initialize FTM module */
	FTM_Init(FTM0, &ftmInfo);

	/*
	 * NOTE: PWM frequency is multiplied by 2 since, I suspect, there is a bug in the code.
	 * Indeed, measuring the frequency on the PWM pin, it appears is set to half the required frequency.
	 */
	FTM_SetupPwm(FTM0, ftmParam, SERVO_NUMBER,
			kFTM_EdgeAlignedPwm, SERVO_FREQUENCY * 2, CLOCK_GetFreq(kCLOCK_CoreSysClk));
	FTM_StartTimer(FTM0, kFTM_SystemClock);

	// Reset to default position
	Command_Servo(SERVO_X_CHANNEL, 0);
	Command_Servo(SERVO_Y_CHANNEL, 0);

	vTaskDelay(1000 / portTICK_PERIOD_MS);
}

/*
 * Initialize the touch screen
 */
static void Init_TouchScreen(void) {
	adc16_config_t adc16ConfigStruct0;
	adc16_config_t adc16ConfigStruct1;

	ADC16_GetDefaultConfig(&adc16ConfigStruct0);
	adc16ConfigStruct0.resolution = kADC16_ResolutionSE16Bit;
	adc16ConfigStruct0.clockDivider = kADC16_ClockDivider1;

	ADC16_GetDefaultConfig(&adc16ConfigStruct1);
	adc16ConfigStruct1.resolution = kADC16_ResolutionSE16Bit;
	adc16ConfigStruct1.clockDivider = kADC16_ClockDivider1;

	ADC16_Init(ADC0, &adc16ConfigStruct0);
	ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */

	ADC16_Init(ADC1, &adc16ConfigStruct1);
	ADC16_EnableHardwareTrigger(ADC1, false); /* Make sure the software trigger is used. */

#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	if (kStatus_Success != ADC16_DoAutoCalibration(ADC0)) {
		PRINTF("#0 ADC16_DoAutoCalibration() Failed.\r\n");
	}
	if (kStatus_Success != ADC16_DoAutoCalibration(ADC1)) {
		PRINTF("#1 ADC16_DoAutoCalibration() Failed.\r\n");
	}
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

	ADC16_SetHardwareAverage(ADC0, kADC16_HardwareAverageCount32);
	ADC16_SetHardwareAverage(ADC1, kADC16_HardwareAverageCount32);
}

/*
 * Update the set point according to the current mode
 */
static void UpdateSetPoint() {
	uint16_t xRef = 0;
	uint16_t yRef = 0;

	if ((xTaskGetTickCount() * portTICK_PERIOD_MS - startModeTime)
			> modeTimeout) {
		startModeTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
		switch (setPointMode) {
		case CENTER:
			xRef = SCREEN_WIDTH / 2;
			yRef = SCREEN_HEIGHT / 2;
			break;
		case VERTEX:
			xRef = SCREEN_WIDTH / 2 + ( (vertex == 0 || vertex == 3) ? -OFFSET_X: OFFSET_X);
			yRef = SCREEN_HEIGHT / 2 + ( (vertex == 0 || vertex == 1) ? -OFFSET_Y : OFFSET_Y);
			vertex = (vertex + 1) % 4;
			break;
		case CIRCLE:
			rotationAngle = (rotationAngle + ROTATION_ANGLE_STEP) % 360;
			xRef = ROTATION_RADIUS * cos(rotationAngle * PI / 180.0) + SCREEN_WIDTH / 2;
			yRef = ROTATION_RADIUS * sin(rotationAngle * PI / 180.0) + SCREEN_HEIGHT / 2;
			break;
		}

		Control_SetPoint(xRef, yRef);
	}
}

/*------------------------------------------------------------------------------
 Application OS threads
 -----------------------------------------------------------------------------*/

/*
 * Control task
 */
static void ControlTask(void *pvParameters) {
	long currentTime = 0;
	uint16_t x = 0;
	uint16_t y = 0;
	int8_t angleX = 0;
	int8_t angleY = 0;

	Init_TouchScreen();
	PRINTF("TouchScreen Initialised.\r\n");

	Init_Servos();
	PRINTF("Servo Initialised.\r\n");

	startModeTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
	PRINTF("\nStarting controller....\r\n\r\n");

	while (true) {

		UpdateSetPoint();

		x = Read_TouchScreenX();
		y = Read_TouchScreenY();

		if (x >= TS_ADC_MINX && x <= TS_ADC_MAXX && y >= TS_ADC_MINY && y <= TS_ADC_MAXY) {

			/* A touch is recognised in the valid range */
			x = map(x, TS_ADC_MINX, TS_ADC_MAXX, 0, SCREEN_WIDTH);
			y = map(y, TS_ADC_MINY, TS_ADC_MAXY, 0, SCREEN_HEIGHT);

			switch (setPointMode) {
			case CENTER:
				GPIO_PortToggle(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_PIN);
				break;
			case VERTEX:
				GPIO_PortToggle(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_PIN);
				break;
			case CIRCLE:
				GPIO_PortToggle(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_PIN);
				break;
			}
		} else {
			/* ADC reads bad data or no touch is recognised */

			/* Reset the plate if no touch */
			if ((xTaskGetTickCount() * portTICK_PERIOD_MS - touchTime) > NO_TOUCH_TIMEOUT ) {
				touchTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
				Command_Servo(SERVO_X_CHANNEL, 0);
				Command_Servo(SERVO_Y_CHANNEL, 0);
			}

			GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_PIN);
			GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_PIN);
			GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_PIN);
			continue;
		}

		currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

		Control_Update(x, y, &angleX, &angleY, currentTime);

		Command_Servo(SERVO_X_CHANNEL, angleX);
		Command_Servo(SERVO_Y_CHANNEL, angleY);

		vTaskDelay(CONTROLLER_DELAY / portTICK_PERIOD_MS);
	}
}

/*------------------------------------------------------------------------------
 Handlers
 -----------------------------------------------------------------------------*/

/*
 * Push button IRQ handler
 */
void PORTC_IRQHandler(void) {
	/* Clear external interrupt flag. */
	GPIO_PortClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_PIN);

	GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_PIN);
	GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_PIN);
	GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_PIN);

	setPointMode = (setPointMode + 1) % MAX_MODE;

	switch (setPointMode) {
	case CENTER:
		modeTimeout = 0;
		GPIO_PortClear(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_PIN);
		break;
	case VERTEX:
		vertex = 0;
		modeTimeout = TIMEOUT_MODE_VERTEX;
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_PIN);
		break;
	case CIRCLE:
		rotationAngle = 0;
		modeTimeout = TIMEOUT_MODE_CIRLE;
		GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_PIN);
		break;
	}
}

/*------------------------------------------------------------------------------
 Application entry point
 -----------------------------------------------------------------------------*/
int main(void) {
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();

	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	GPIO_PortClear(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_PIN);
	GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_PIN);
	GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_PIN);

	PRINTF("Board Initialised.\r\n");

	Control_Init(controlInit);

	if (xTaskCreate(ControlTask, "Control", configMINIMAL_STACK_SIZE * 2,
			NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}

	vTaskStartScheduler();

	for (;;)
		;

	return 0;
}

