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
#include "control.h"

/******************************************************************************
 * Internal Variables
 ******************************************************************************/
static PID pidX;			// PID for X coordinate
static PID pidY;			// PID for Y coordinate
static Kalman kalmanX;		// Kalman filter for X coordinate
static Kalman kalmanY;		// Kalman filter for X coordinate

static uint16_t setPoint_x;		// X coordinate of the set point
static uint16_t setPoint_y;		// Y coordinate of the set point

/******************************************************************************
 * API
 ******************************************************************************/
void Control_Init(ControlInit controlInit) {
	PID_Init(&pidX, controlInit.xP, controlInit.xI, controlInit.xD);
	PID_Init(&pidY, controlInit.yP, controlInit.yI, controlInit.yD);

	Kalman_Init(&kalmanX, controlInit.xQ, controlInit.xR);
	Kalman_Init(&kalmanY, controlInit.yQ, controlInit.yR);

	setPoint_x = 0;
	setPoint_y = 0;
}

void Control_SetPoint(uint16_t x, uint16_t y) {
	setPoint_x = x;
	setPoint_y = y;
}

void Control_Update(uint16_t x, uint16_t y, int8_t *angleX, int8_t *angleY,
		long timeInMilliseconds) {
	/*
	 * Estimate x and y coordinates
	 */
	float x_est = Kalman_Update(&kalmanX, (float) x);
	float y_est = Kalman_Update(&kalmanY, (float) y);

	/*
	 * Update PIDs
	 */
	pidX.currentTime = timeInMilliseconds;
	pidY.currentTime = timeInMilliseconds;
	float ctrl_x = PID_Update(&pidX, setPoint_x, x_est);
	float ctrl_y = PID_Update(&pidY, setPoint_y, y_est);

	/*
	 * Provide PIDs output to motor angles
	 */
	*angleX = -ctrl_x;
	*angleY = -ctrl_y;
}
