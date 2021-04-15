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


#ifndef PID_H_
#define PID_H_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdbool.h>

/******************************************************************************
 * Types
 ******************************************************************************/
typedef struct {
	float P; 					// Proportional gain.
	float I; 					// Integral gain.
	float D; 					// Derivative gain.
	float lastPosition; 		// Last reference value.
	long previousPIDTime; 		// Previous PID update time.
	long currentTime; 			// Current PID update time.
	bool firstPass; 			// Initialization flag.
	float cumulativeError; 		// Last integral error.
} PID;

/******************************************************************************
 * Prototypes
 ******************************************************************************/
void PID_Init(PID *pid, float P, float I, float D);
void PID_SetP(PID *pid, float val);
void PID_SetI(PID *pid, float val);
void PID_SetD(PID *pid, float val);
float PID_Update(PID *pid, float targetPosition, float currentPosition);

#endif /* PID_H_ */
