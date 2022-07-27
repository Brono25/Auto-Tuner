/*
 * state_machine.c
 *
 *  Created on: 27 Jul. 2022
 *      Author: brono
 */

#include "mpm.h"
#include "arm_math.h"
#include "oled_print.h"
#include "state_machine.h"





void get_frequency(float32_t *signal, float32_t target_freq, float32_t *out_freq, int *callback_state)
{
	if (*callback_state == 1)
	{
		float32_t curr_freq = 0;
		mpm_mcleod_pitch_method_f32(&signal[0], &curr_freq);
		*callback_state = 0;

		if(target_freq - L_FREQ_ERROR < curr_freq && curr_freq < target_freq + U_FREQ_ERROR)
		{
			*out_freq = curr_freq;
		} else
		{
			*out_freq = 0;
		}
	}
}
