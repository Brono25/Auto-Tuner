/*
 * mpm.h
 *
 *  Created on: 26 Jul. 2022
 *      Author: brono
 */

#ifndef INC_MPM_H_
#define INC_MPM_H_
#endif

#ifndef BLOCK_SIZE
#define BLOCK_SIZE 1024
#endif


#include "arm_math.h"
#define FS 40000
#define PEAK_THRESHOLD 0.9
#define FIRST_CROSSING_PASSED 1
#define FIRST_CROSSING_NOT_PASSED 0
#define VALID_PEAK_FOUND 1
#define VALID_PEAK_NOT_FOUND 0

float32_t mpm_reserved_memory[2 * BLOCK_SIZE] = {0};

void mpm_mcleod_pitch_method_f32(float32_t *pSrc, float32_t *pitch_estimate);
void mpm_sum_f32(float32_t *pSrc, uint16_t scrLen, float32_t *pRes);
void mpm_find_peak_f32(float32_t *pSrc, uint16_t *tau);
void mpm_NSDF_f32(float32_t *pSrc, float32_t **pDst);
void mpm_parabolic_interpolation_f32(uint16_t x_pos, float32_t a, float32_t b, float32_t c, float32_t *delta_tau);




