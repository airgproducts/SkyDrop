/*
 * gui.h
 *
 *  Created on: 22.1.2015
 *      Author: horinek
 */

#ifndef GUI_H_
#define GUI_H_

#include "../common.h"
#include "../drivers/lcd_disp.h"
#include "../drivers/led.h"
#include "../fc/fc.h"

extern lcd_display disp;
extern FILE * lcd_out;

#define GUI_DISP_WIDTH	lcd_width
#define GUI_DISP_HEIGHT	lcd_height

#define GUI_BLINK_TGL(PER) (task_get_ms_tick() % (PER) > ((PER) / 2))

#define GUI_CONTRAST_STEPS 	10

void gui_switch_task(uint8_t new_task);

void gui_init();
void gui_load_eeprom();

void gui_stop();
void gui_loop();
void gui_irqh(uint8_t type, uint8_t * buff);

void gui_force_loop();

void gui_trigger_backlight();
void gui_change_disp_cfg();
void gui_update_disp_cfg();

void gui_showmessage_P(const char * msg);
void gui_showmessage(char * msg);
void gui_forcemessage();
void gui_hidemessage();
void gui_messageduration(uint16_t seconds);

void gui_dialog_P(const char * title);
void gui_dialog(char * title);

void gui_reset_timeout();

#define GUI_DIALOG_TOP 		13
#define GUI_DIALOG_LEFT 	2
#define GUI_DIALOG_RIGHT 	(GUI_DISP_WIDTH - 3)
#define GUI_DIALOG_BOTTOM 	(GUI_DISP_HEIGHT - 3)
#define GUI_DIALOG_WIDTH	(GUI_DIALOG_RIGHT - GUI_DIALOG_LEFT)
#define GUI_DIALOG_HEIGHT	(GUI_DIALOG_BOTTOM - GUI_DIALOG_TOP)

void gui_raligh_text_P(const char * text, uint8_t x, uint8_t y);
void gui_raligh_text(char * text, uint8_t x, uint8_t y);

void gui_caligh_text_P(const char * text, uint8_t x, uint8_t y);
void gui_caligh_text(char * text, uint8_t x, uint8_t y);

void gui_fit_text(char * in, char * out, uint8_t size);

void gui_statusbar();

#define GUI_NONE				0xFF
#define GUI_SETTINGS			0
#define GUI_SPLASH				1
#define GUI_SET_VAL				2
#define GUI_SET_AUDIO			3
#define GUI_SET_DISPLAY			4
#define GUI_USB					5
#define GUI_FTEST				6
#define GUI_SET_SYSTEM			7
#define GUI_SET_GPS				8
#define GUI_SET_GPS_DETAIL		9
#define GUI_SET_DEBUG			10
#define GUI_DIALOG				11
#define GUI_SET_BLUETOOTH		12
#define GUI_UPDATE				13
#define GUI_SET_AUDIO_MENU		14
#define GUI_TEXT				15
#define GUI_SET_ADVANCED		16
#define GUI_SET_CALIB			17
#define GUI_SET_CALIB_ACC		18
#define GUI_SET_CALIB_MAG		19
#define GUI_FILEMANAGER			20
#define GUI_SET_CALIB_GYRO		21
#define GUI_SET_COMPASS			22

#define NUMBER_OF_GUI_TASKS		23

#define GUI_LAST_TASK			0xFF

extern uint8_t lcd_contrast_min;
extern uint8_t lcd_contrast_max;

extern volatile uint8_t gui_task;
extern volatile uint8_t gui_new_task;
extern volatile bool gui_buttons_override;

#endif /* GUI_H_ */
