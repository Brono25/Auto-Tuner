

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








