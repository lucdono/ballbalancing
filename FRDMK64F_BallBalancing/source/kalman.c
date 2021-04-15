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
#include "kalman.h"

/******************************************************************************
 * API
 ******************************************************************************/
void Kalman_Init(Kalman *filter, float Q, float R) {
	filter->Q = Q;
	filter->R = R;
	filter->x_est_last = 0;
	filter->P_last = 0;
	filter->K = 0;
	filter->P = 0;
	filter->P_temp = 0;
	filter->x_temp_est = 0;
	filter->x_est = 0;
}

float Kalman_Update(Kalman *filter, float measure) {
	/*
	 *  Estimate the value
	 */
	filter->x_temp_est = filter->x_est_last;
	filter->P_temp = filter->P_last + filter->Q;

	/*
	 * Calculate the Kalman gain
	 */
	filter->K = filter->P_temp * (1.0 / (filter->P_temp + filter->R));

	/*
	 * Corrections and update
	 */
	filter->x_est = filter->x_temp_est + filter->K * (measure - filter->x_temp_est);

	filter->P = (1 - filter->K) * filter->P_temp;

	filter->P_last = filter->P;
	filter->x_est_last = filter->x_est;

	return(filter->x_est);
}
