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

#ifndef CONTROL_H_
#define CONTROL_H_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>

#include "pid.h"
#include "kalman.h"

/******************************************************************************
 * Defines
 ******************************************************************************/
#define map(in, inMin, inMax, outMin, outMax) 	(((float)in - (float)inMin) / \
													((float)inMax - (float)inMin) * \
														((float)outMax - (float)outMin) + (float)outMin)

/******************************************************************************
 * Types
 ******************************************************************************/
typedef struct {
	float xP;	// PID x
	float xI;	// PID x
	float xD;	// PID x

	float yP;	// PID y
	float yI;	// PID y
	float yD;	// PID y

	float xQ;	// Kalman filter x
	float xR;	// Kalman filter x

	float yQ;	// Kalman filter y
	float yR;	// Kalman filter y
} ControlInit;

/******************************************************************************
 * Prototypes
 ******************************************************************************/
void Control_Init(ControlInit controlInit);
void Control_Update(uint16_t x, uint16_t y, int8_t *angleX, int8_t *angleY, long timeInMilliseconds);
void Control_SetPoint(uint16_t x, uint16_t y);

#endif /* CONTROL_H_ */
