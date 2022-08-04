

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

	char home_screen[11] = "Auto-Tuner";
	for(int x = WIDTH; x > MID_X_POS_MED; x--)
	{
		ssd1306_SetCursor(x, MID_Y_POS_MED);
		ssd1306_WriteString(home_screen, FONT_MED, Black);
		ssd1306_UpdateScreen();
	}

	HAL_Delay(3000);

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
	ssd1306_SetCursor(25, 24);
	ssd1306_WriteString(var_string, FONT_LARGE, Black);
	ssd1306_UpdateScreen();
}

void oled_print_int16(int16_t var)
{
	char var_string[10] = {0};
	sprintf(var_string,"%hd", var);
	ssd1306_SetCursor(25, 24);
	ssd1306_WriteString(var_string, FONT_LARGE, Black);
	ssd1306_UpdateScreen();
}


void oled_clear_screen(void)
{

	ssd1306_Fill(White);
	ssd1306_DrawCircle(SCREEN_CENTRE_X, SCREEN_CENTRE_Y, RADIUS, Black);
	ssd1306_UpdateScreen();


}

void oled_print_pitch_indicator_screen(char *guit_string, int error)
{

	ssd1306_Fill(White);
	ssd1306_DrawCircle(SCREEN_CENTRE_X, SCREEN_CENTRE_Y, RADIUS, Black);
	ssd1306_SetCursor(50, 5);
	ssd1306_WriteString(guit_string, FONT_LARGE, Black);

	int x_pos = round(error/2) + 62;
	if (x_pos < 1)
	{
		x_pos = 1;
	}
	else if (x_pos > 126)
	{
		x_pos = 126;
	}
	ssd1306_SetCursor(x_pos , 30);

	char indicator = '|';
	ssd1306_WriteChar(indicator, FONT_LARGE, Black);

	ssd1306_UpdateScreen();


}


void oled_clear_pitch_indicator_tick(char *guit_string)
{

	ssd1306_Fill(White);
	ssd1306_DrawCircle(SCREEN_CENTRE_X, SCREEN_CENTRE_Y, RADIUS, Black);
	ssd1306_SetCursor(50, 5);
	ssd1306_WriteString(guit_string, FONT_LARGE, Black);
	ssd1306_UpdateScreen();

}



void oled_tone_screen(int tone)
{



	ssd1306_Fill(White);
	ssd1306_SetCursor(84, MID_Y_POS_MED);
	char tone_string[10];
	sprintf(tone_string,"%d", tone);
	ssd1306_WriteString(tone_string, FONT_MED, Black);
	ssd1306_SetCursor(18, MID_Y_POS_MED);
	ssd1306_WriteString("Tone: ", FONT_MED, Black);
	ssd1306_UpdateScreen();
	HAL_Delay(100);

}

void oled_timing_screen(int timing)
{

	ssd1306_Fill(White);


	ssd1306_Fill(White);
	ssd1306_SetCursor(30, MID_Y_POS_MED);
	char timing_string[10];
	sprintf(timing_string,"%d", timing);
	ssd1306_WriteString(timing_string, FONT_MED, Black);
	ssd1306_SetCursor(70, MID_Y_POS_MED);
	ssd1306_WriteString(" BPM", FONT_MED, Black);
	ssd1306_UpdateScreen();
	HAL_Delay(100);

}


void oled_selection_screen(void)
{
	ssd1306_Fill(White);

	char selection_screen[5] = "Tone";
	char *timing_screen = "Metronome";
	for(int x = 98; x > MID_X_POS_MED; x--)
	{
		ssd1306_SetCursor(40, 10);
		ssd1306_WriteString(selection_screen, FONT_MED, Black);
		ssd1306_SetCursor(15, 40);
		ssd1306_WriteString(timing_screen, FONT_MED, Black);
		ssd1306_UpdateScreen();

	}
}


void oled_print_string_tuning(char *guit_string, int error)
{

	char err_string[20] = {0};
	sprintf(err_string,"%d cents",  error);

	ssd1306_SetCursor(35, 40);
	ssd1306_WriteString(err_string, FONT_MED, Black);

	ssd1306_WriteString(guit_string, FONT_LARGE, Black);
	ssd1306_SetCursor(50, 5);

	ssd1306_UpdateScreen();


}



void oled_clear_tuning_screen(char *guit_string)
{

	ssd1306_Fill(White);
	ssd1306_DrawCircle(SCREEN_CENTRE_X, SCREEN_CENTRE_Y, RADIUS, Black);
	ssd1306_SetCursor(50, 5);
	ssd1306_WriteString(guit_string, FONT_LARGE, Black);
	ssd1306_UpdateScreen();



}


