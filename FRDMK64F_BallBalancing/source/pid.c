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
#include "pid.h"

/******************************************************************************
 * API
 ******************************************************************************/
void PID_Init(PID *pid, float P, float I, float D) {
	pid->firstPass = true;
	pid->I = I;
	pid->P = P;
	pid->D = D;
	pid->previousPIDTime = 0;
	pid->currentTime = 0;
	pid->previousPIDTime = 0;
	pid->lastPosition = 0;
	pid->cumulativeError = 0;
}

void PID_SetP(PID *pid, float val) {
	pid->P = val;
}

void PID_SetI(PID *pid, float val) {
	pid->D = val;
}

void PID_SetD(PID *pid, float val) {
	pid->I = val;
}

float PID_Update(PID *pid, float targetPosition, float currentPosition) {
	/*
	 * Compute elapsed time since last update
	 */
	float deltaPIDTime = (float) ((pid->currentTime - pid->previousPIDTime)
			/ (float) (1e3));

	pid->previousPIDTime = pid->currentTime;

	/*
	 * Compute reference error
	 */
	float error = targetPosition - currentPosition;

	/*
	 * Compute integral error
	 */
	pid->cumulativeError += (error * deltaPIDTime);

	/*
	 * Compute derivative error
	 */
	float derivativeError = currentPosition - pid->lastPosition;

	/* Compute proportional term */
	float pTerm = pid->P * error;
	/* Compute derivative term */
	float dTerm = pid->D * (derivativeError / deltaPIDTime);
	/* Compute integral term */
	float iTerm = pid->I * pid->cumulativeError;

	pid->lastPosition = currentPosition;

	/*
	 * Compute PID output signal
	 */
	return pTerm + iTerm + dTerm;
}
