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

#ifndef KALMAN_H_
#define KALMAN_H_

/******************************************************************************
 * Types
 ******************************************************************************/
typedef struct {
	float x_est_last;	// Last output estimation.
	float P_last;		// Last prediction.
	float Q;			// Noise covariance.
	float R;			// Noise covariance.
	float K;			// Kalman gain.
	float P;			// Prediction.
	float P_temp;		// System inner state temp value.
	float x_temp_est;	// Output estimation temp value.
	float x_est;		// Output estimation.
} Kalman;

/******************************************************************************
 * Prototypes
 ******************************************************************************/

void Kalman_Init(Kalman *filter, float Q, float R);
float Kalman_Update(Kalman *filter, float measure);

#endif /* KALMAN_H_ */
