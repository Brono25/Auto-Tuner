/*
 * oled_print.h
 *
 *  Created on: Jul 16, 2022
 *      Author: brono
 */

#ifndef INC_OLED_PRINT_H_
#define INC_OLED_PRINT_H_



#endif /* INC_OLED_PRINT_H_ */





void oled_init(void);
void oled_print_string(char *string);
void oled_print_f32(float *var);
void oled_clear_screen(void);
void oled_print_int16(int16_t var);
void oled_print_pitch_indicator_screen(char *guit_string, int error);
void oled_clear_pitch_indicator_tick(char *guit_string);
void oled_tone_screen(int tone);
void oled_timing_screen(int timing);
void oled_selection_screen(void);
void oled_print_string_tuning(char *guit_string, int error);
void oled_clear_tuning_screen(char *guit_string);


