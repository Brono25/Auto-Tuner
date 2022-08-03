

/*------------------------------------------------------------

OLED 64 x 128 display handling
This function uses the OLED driver by Aleksander Alekseev
https://github.com/afiskon/stm32-ssd1306.git

------------------------------------------------------------*/




#include <stdio.h>
#include "ssd1306.h"
#include "oled_print.h"
#include <math.h>



#define CHAR_PIXEL 5
#define FONT_LARGE Font_16x26


#define FONT_MED   Font_11x18
#define MID_Y_POS_MED 24
#define MID_X_POS_MED 10

#define WIDTH 128
#define HEIGHT 64

#define SCREEN_CENTRE_X WIDTH  / 2
#define SCREEN_CENTRE_Y HEIGHT / 2
#define RADIUS 30



void oled_init(void)
{
	ssd1306_Init1();
	ssd1306_Fill(White);
	ssd1306_DrawCircle(SCREEN_CENTRE_X, SCREEN_CENTRE_Y, RADIUS, Black);
	/*
	char home_screen[11] = "Auto-Tuner";
	for(int x = WIDTH; x > MID_X_POS_MED; x--)
	{
		ssd1306_SetCursor(x, MID_Y_POS_MED);
		ssd1306_WriteString(home_screen, FONT_MED, Black);
		ssd1306_UpdateScreen();
	}

	HAL_Delay(1000);
 	 */
	ssd1306_Fill(White);
	ssd1306_DrawCircle(SCREEN_CENTRE_X, SCREEN_CENTRE_Y, RADIUS, Black);
	ssd1306_UpdateScreen();
}

void oled_print_string(char *string)
{
	 ssd1306_SetCursor(10, 24);
	 ssd1306_WriteString(string, FONT_MED, Black);
	 ssd1306_UpdateScreen();

}


void oled_print_f32(float *var)
{
	char var_string[10] = {0};
	sprintf(var_string,"%.1f", *var);

	ssd1306_WriteString(var_string, FONT_LARGE, Black);
	ssd1306_SetCursor(25, 24);
	ssd1306_UpdateScreen();
}

void oled_print_int16(int16_t var)
{
	char var_string[10] = {0};
	sprintf(var_string,"%hd", var);

	ssd1306_WriteString(var_string, FONT_LARGE, Black);
	ssd1306_SetCursor(25, 24);
	ssd1306_UpdateScreen();
}


void oled_clear_screen(void)
{

	ssd1306_Fill(White);
	ssd1306_DrawCircle(SCREEN_CENTRE_X, SCREEN_CENTRE_Y, RADIUS, Black);
	ssd1306_UpdateScreen();


}

void oled_print_pitch_indicator_screen(char *pitch)
{
	ssd1306_Fill(White);
	ssd1306_SetCursor(SCREEN_CENTRE_X - 2*CHAR_PIXEL , CHAR_PIXEL);
	ssd1306_WriteString(pitch, FONT_MED, Black);
	char indicator = '|';
	ssd1306_SetCursor(SCREEN_CENTRE_X - CHAR_PIXEL , 4 * CHAR_PIXEL);
	ssd1306_WriteChar(indicator, FONT_MED, Black);



	//ssd1306_SetCursor(freq , 8 * CHAR_PIXEL);
	//ssd1306_WriteString(indicator, FONT_MED, Black);

	ssd1306_UpdateScreen();


}


void oled_update_pitch_indicator_tick(float freq)
{

	int q = (int) 2 *  floor((2 * freq) + 0.5) / 2;
	//char var_string[10] = {0};

	int tick_pos = q - 102;

	ssd1306_SetCursor(tick_pos, 8 * CHAR_PIXEL);
	ssd1306_WriteChar('|', FONT_MED, Black);
	ssd1306_UpdateScreen();




}



void oled_tone_screen(int tone)
{

	ssd1306_Fill(White);

	char tone_screen[5] = "Tone";
	char tone_scc[7] = "Screen";
	for(int x = WIDTH; x > MID_X_POS_MED; x--)
	{
		ssd1306_SetCursor(x, 10);
		ssd1306_WriteString(tone_screen, FONT_MED, Black);
		ssd1306_SetCursor(x, 40);
		ssd1306_WriteString(tone_scc, FONT_MED, Black);
		ssd1306_UpdateScreen();
	}

	HAL_Delay(3000);

	ssd1306_Fill(White);
	ssd1306_SetCursor(84, MID_Y_POS_MED);
	char tone_string[10];
	sprintf(tone_string,"%d", tone);
	ssd1306_WriteString(tone_string, FONT_MED, Black);
	ssd1306_SetCursor(18, MID_Y_POS_MED);
	ssd1306_WriteString("Tone: ", FONT_MED, Black);
	ssd1306_UpdateScreen();

	HAL_Delay(3000);
}

void oled_timing_screen(int timing)
{

	ssd1306_Fill(White);

	char timing_screen[7] = "Timing";
	char timing_scc[7] = "Screen";
	for(int x = WIDTH; x > MID_X_POS_MED; x--)
	{
		ssd1306_SetCursor(x, 10);
		ssd1306_WriteString(timing_screen, FONT_MED, Black);
		ssd1306_SetCursor(x, 40);
		ssd1306_WriteString(timing_scc, FONT_MED, Black);
		ssd1306_UpdateScreen();
	}

	HAL_Delay(3000);

	ssd1306_Fill(White);
	ssd1306_SetCursor(90, MID_Y_POS_MED);
	char timing_string[10];
	sprintf(timing_string,"%d", timing);
	ssd1306_WriteString(timing_string, FONT_MED, Black);
	ssd1306_SetCursor(5, MID_Y_POS_MED);
	ssd1306_WriteString("Timing: ", FONT_MED, Black);
	ssd1306_UpdateScreen();

	HAL_Delay(3000);
}


void oled_selection_screen(void)
{
	ssd1306_Fill(White);

	char selection_screen[5] = "Tone";
	char select_screen[7] = "Timing";
	for(int x = 98; x > MID_X_POS_MED; x--)
	{
		ssd1306_SetCursor(40, 10);
		ssd1306_WriteString(selection_screen, FONT_MED, Black);
		ssd1306_SetCursor(30, 40);
		ssd1306_WriteString(select_screen, FONT_MED, Black);
		ssd1306_UpdateScreen();

	}
	HAL_Delay(3000);
}




