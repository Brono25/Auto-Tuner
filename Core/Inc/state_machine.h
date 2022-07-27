/*
 * state_machine.h
 *
 *  Created on: 27 Jul. 2022
 *      Author: brono
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_


#define E2 82.41
#define A2 110.0
#define D3 146.83
#define G3 196.0
#define B3 246.94
#define E4 329.63

#define U_FREQ_ERROR 20
#define L_FREQ_ERROR 20



#endif /* INC_STATE_MACHINE_H_ */
void get_frequency(float32_t *signal, float32_t target_freq, float32_t *out_freq, int *callback_state);
