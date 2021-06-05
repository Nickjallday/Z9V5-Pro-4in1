/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * DWIN LCD
 */

#include "../../../inc/MarlinConfigPre.h"

#if HAS_DWIN_LCD

#include "dwin.h"
#include "../language/dwin_multi_language.h"

#if ANY(AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_3POINT) && DISABLED(PROBE_MANUALLY)
  //#define HAS_ONESTEP_LEVELING 1
#endif


#if ENABLED(MIXING_EXTRUDER)
  #include "../../../feature/mixing.h"
#endif

#if ENABLED(OPTION_REPEAT_PRINTING)
  #include "../../../feature/repeat_printing.h"
#endif

#if ENABLED(OPTION_3DTOUCH)
  #include "../../../feature/bltouch.h"
#endif

#if ENABLED(FWRETRACT)
  #include "../../../feature/fwretract.h"
#endif

#include <WString.h>
#include <stdio.h>
#include <string.h>

#include "../../fontutils.h"
#include "../../ultralcd.h"

#include "../../../sd/cardreader.h"

#include "../../../MarlinCore.h"
#include "../../../core/serial.h"
#include "../../../core/macros.h"
#include "../../../gcode/queue.h"

#include "../../../module/temperature.h"
#include "../../../module/printcounter.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../module/tool_change.h"
#include "../../lcd/thermistornames.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../../../module/settings.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../../feature/host_actions.h"
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #include "../../../feature/runout.h"
  #include "../../../feature/pause.h"
#endif

#if HAS_ONESTEP_LEVELING
  #include "../../../feature/bedlevel/bedlevel.h"
#endif

#if HAS_BED_PROBE
  #include "../../../module/probe.h"
#endif

#if ENABLED(BABYSTEPPING)
  #include "../../../feature/babystep.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#ifndef MACHINE_SIZE
  #define MACHINE_SIZE "300x300x400"
#endif

#ifndef CORP_WEBSITE_C
  #define CORP_WEBSITE_C "www.zonestar3d.com"
#endif

#ifndef CORP_WEBSITE_E
  #define CORP_WEBSITE_E "www.zonestar3d.com"
#endif

#if HAS_LEVELING
  #include "../../../feature/bedlevel/bedlevel.h"
#endif


#define PAUSE_HEAT

#define USE_STRING_HEADINGS

#define DWIN_FONT_MENU font8x16
#define DWIN_FONT_STAT font10x20
#define DWIN_FONT_HEAD font10x20
#define DWIN_FONT_MIX  font14x28

#define MENU_CHAR_LIMIT  24
#define STATUS_Y_START 	 360
#define STATUS_Y_END 	 DWIN_HEIGHT

#define STATUS_MIXER_Y_START 	 State_text_mix_Y
#define STATUS_MIXER_Y_END 	 STATUS_MIXER_Y_START +28

// Fan speed limit
#define FANON           255
#define FANOFF          0

// Print speed limit
#define MAX_PRINT_SPEED   999
#define MIN_PRINT_SPEED   10

// Temp limits
#if HAS_HOTEND
  #define MAX_E_TEMP    (HEATER_0_MAXTEMP - (HOTEND_OVERSHOOT))
  #define MIN_E_TEMP    HEATER_0_MINTEMP
#endif

#if HAS_HEATED_BED
  #define MIN_BED_TEMP  BED_MINTEMP
#endif

// Feedspeed limit (max feedspeed = DEFAULT_MAX_FEEDRATE * 2)
#define MIN_MAXFEEDSPEED      1
#define MIN_MAXACCELERATION   1
#define MIN_MAXJERK           0.1
#define MIN_STEP              1


// Mininum unit (0.1) : multiple (10)
#define MINUNITMULT     10

#define ENCODER_WAIT    20
#define DWIN_SCROLL_UPDATE_INTERVAL 1000
#define DWIN_REMAIN_TIME_UPDATE_INTERVAL 60000

constexpr uint16_t TROWS = 6, MROWS = TROWS - 1,        // Total rows, and other-than-Back
                   TITLE_HEIGHT = 30,                   // Title bar height
                   MLINE = 53,                          // Menu line height
                   LBLX = 55,                           // Menu item label X
                   LBLX_INFO = 25,                      // Info Menu item label X
                   MENU_CHR_W = 8, STAT_CHR_W = 10,
				   MENU_CHR_H = 16, STAT_CHR_H = 20;

#define MBASE(L) (49 + MLINE * (L))

/* Value Init */
HMI_value_t HMI_ValueStruct;
HMI_Flag_t HMI_flag = {0};
FIL_CFG FIL;
MIXER_CFG MixerCfg;
MIXER_DIS MixerDis;

millis_t dwin_heat_time = 0;

uint8_t checkkey = 0;

typedef struct {
  uint8_t now, last;
  void set(uint8_t v) { now = last = v; }
  void reset() { set(0); }
  bool changed() { bool c = (now != last); if (c) last = now; return c; }
  bool dec() { if (now) now--; return changed(); }
  bool inc(uint8_t v) { if (now < (v - 1)) now++; else now = (v - 1); return changed(); }
} select_t;

select_t select_page{0}, select_file{0}, select_print{0}, select_prepare{0} \
         , select_control{0}, select_axis{0}, select_temp{0}, select_motion{0}, select_mixer{0}, select_tune{0} \
         , select_PLA{0}, select_ABS{0} \
         , select_speed{0} 							\
         , select_acc{0} 								\
         , select_jerk{0} 							\
         , select_step{0} 							\
				 , select_manual{0} 						\
				 , select_auto{0} 							\
				 , select_random{0} 						\
				 , select_vtool{0} 							\
				 , select_leveling{0} 					\
				 , select_home{0} 							\
				 , select_option{0} 						\
				 , select_bltouch{0} 						\
				 , select_powerdown{0} 					\
				 , select_language{0} 					\
				 , select_info{0} 							\
				 , select_config{0} 						\
				 , select_retract{0} 						\
				 , select_reprint{0} 						\
         ;

uint8_t index_file     	= MROWS,
        index_prepare  	= MROWS,
        index_control  	= MROWS,
        index_leveling 	= MROWS,
        index_home	 	= MROWS,
        index_tune     	= MROWS,
				index_axismove  = MROWS,
				index_manual  	= MROWS,
				index_auto  	= MROWS,
				index_random  	= MROWS,
				index_bltouch  	= MROWS,
        index_language  = MROWS,
        index_info  	= MROWS,
				index_config  	= MROWS,
				index_retract  	= MROWS,
				index_reprint  	= MROWS;


bool dwin_abort_flag = false; // Flag to reset feedrate, return to Home

constexpr float default_max_feedrate[]        = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_acceleration[]    = DEFAULT_MAX_ACCELERATION;
constexpr float default_max_jerk[]            = { DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK };
constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

uint8_t Percentrecord = 0;
uint16_t remain_time = 0;

#if ENABLED(PAUSE_HEAT)
  #if HAS_HOTEND
    uint16_t temphot = 0;
  #endif
  #if HAS_HEATED_BED
    uint16_t tempbed = 0;
  #endif
#endif

#if ENABLED(BABYSTEPPING)
static millis_t Babysteps_timer_first;
static millis_t Babysteps_timer_second;
static float babyz_offset = 0.0;
static float last_babyz_offset = 0.0;
static float prevouis_babyz_offset = 0.0;
#endif

#ifdef IIC_BL24CXX_EEPROM
#define DWIN_LANGUAGE_EEPROM_ADDRESS 0x01   // Between 0x01 and 0x63 (EEPROM_OFFSET-1)
                                            // BL24CXX::check() uses 0x00
#else
#define DWIN_LANGUAGE_EEPROM_ADDRESS 80    
#endif
/*
inline bool HMI_IsChinese() { return HMI_flag.language == DWIN_CHINESE; }
*/
void HMI_SetLanguageCache() {
  //DWIN_JPG_CacheTo1(HMI_IsChinese() ? Language_Chinese : Language_English);
  //HMI_flag.language = 2;
  DWIN_JPG_CacheToN(1,HMI_flag.language + 1);
  if(HMI_flag.language < 3) HMI_flag.Title_Menu_Backup = 7;
  else HMI_flag.Title_Menu_Backup = 6;
  //DWIN_JPG_CacheToN(1,8);
}

void HMI_SetLanguage() {
/*
  #if ENABLED(EEPROM_SETTINGS)
    HMI_flag.language = DWIN_ENGLISH;
	#ifdef IIC_BL24CXX_EEPROM
    BL24CXX::read(DWIN_LANGUAGE_EEPROM_ADDRESS, (uint8_t*)&HMI_flag.language, sizeof(HMI_flag.language));
	#else
	//EEPROM_READ();
	#endif
  #endif
  */
  HMI_SetLanguageCache();
}

void HMI_ToggleLanguage() {
  //HMI_flag.language = HMI_IsChinese() ? DWIN_ENGLISH : DWIN_CHINESE;
  //settings.save();
  HMI_SetLanguageCache();
  /*
  #if ENABLED(EEPROM_SETTINGS)
    #ifdef IIC_BL24CXX_EEPROM
    BL24CXX::write(DWIN_LANGUAGE_EEPROM_ADDRESS, (uint8_t*)&HMI_flag.language, sizeof(HMI_flag.language));
	#else
	uint16_t working_crc = 0;
	//EEPROM_WRITE();
	#endif
  #endif
  */
}

FORCE_INLINE void DWIN_Draw_Signed_Float(uint8_t size, uint16_t Color, uint16_t bColor, uint8_t iNum, uint8_t fNum, uint16_t x, uint16_t y, long value) {		
	uint8_t w = 6;
	if(size <= 5) 
		w = 6 + size*2;
	else if(size <= 9) 
		w = 6 + size*4;
	if (value < 0) {
    DWIN_Draw_String(false, true, size, Color, bColor, x-w, y, F("-"));
    DWIN_Draw_FloatValue(true, true, 0, size, Color, bColor, iNum, fNum, x, y, -value);
  }
  else {
    DWIN_Draw_String(false, true, size, Color, bColor, x-w, y, F(" "));
    DWIN_Draw_FloatValue(true, true, 0, size, Color, bColor, iNum, fNum, x, y, value);
  }
}

unsigned int GenRandomString(int length)
{
   int i;
   srand(millis());   
   for (i = 0; i < length; i++)
   {          
      return rand()%length;  
   }
   return 0;
}

uint8_t  Check_Percent_equal(){
	for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) {
		if(MixerCfg.Manual_Percent[mixer.selected_vtool][i] != MixerCfg.Manual_Percent[mixer.selected_vtool][i+1]) return 1;
	}
	return 0;
}

void updata_mixer_from_vtool(){
	float ctot = 0;
	int16_t sum_mix = 0;
    MIXER_STEPPER_LOOP(i) ctot += mixer.color[mixer.selected_vtool][i];
    MIXER_STEPPER_LOOP(i) mixer.mix[i] = (int8_t)(100.0f * mixer.color[mixer.selected_vtool][i] / ctot);
	for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) sum_mix += mixer.mix[i];
	mixer.mix[MIXING_STEPPERS-1] = 100 - sum_mix;
}

void recalculation_mixer_percent(){
    uint16_t sum_mix = 0;
	MIXER_STEPPER_LOOP(i) sum_mix+=mixer.mix[i];
	const float scaleMix = 100/sum_mix;
	MIXER_STEPPER_LOOP(i) mixer.mix[i] *= scaleMix;
	sum_mix = 0;
	for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) sum_mix += mixer.mix[i];
	mixer.mix[MIXING_STEPPERS-1] = 100 - sum_mix;
}


void ICON_Print() {
  uint16_t Coordinate_Temp;
  Coordinate_Temp = Print_X_Coordinate[HMI_flag.language];
  if (select_page.now == 0) {
    DWIN_ICON_Show(ICON, ICON_Print_1, 17, 130);
    DWIN_Draw_Rectangle(0, Color_White, 17, 130, 126, 229);
  	//DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Print, Menu_Coordinate,53,201);
  	DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Print], Coordinate_Temp, 201);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Print_0, 17, 130);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Main_Menu_Print0, Menu_Coordinate, Coordinate_Temp,201);
  }
  DWIN_UpdateLCD();
}

void ICON_Prepare() {
  uint16_t Coordinate_Temp;
  Coordinate_Temp = Prepare_X_Coordinate[HMI_flag.language];
  if (select_page.now == 1) {
    DWIN_ICON_Show(ICON, ICON_Prepare_1, 145, 130);
    DWIN_Draw_Rectangle(0, Color_White, 145, 130, 254, 229);
  	//DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Prepare, Menu_Coordinate,172,201);
  	DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Prepare], Coordinate_Temp, 201);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Prepare_0, 145, 130);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Main_Menu_Prepare0, Menu_Coordinate, Coordinate_Temp,201);
  }
  DWIN_UpdateLCD();
}

void ICON_Control() {
  uint16_t Coordinate_Temp;
  Coordinate_Temp = Control_X_Coordinate[HMI_flag.language];
  if (select_page.now == 2) {
    DWIN_ICON_Show(ICON, ICON_Control_1, 17, 246);
    DWIN_Draw_Rectangle(0, Color_White, 17, 246, 126, 345);
  	//DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Control, Menu_Coordinate,44,318);
  	DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Control], Coordinate_Temp, 318);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Control_0, 17, 246);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Main_Menu_Control0, Menu_Coordinate, Coordinate_Temp,318);
  }
  DWIN_UpdateLCD();
}

void ICON_StartInfo(bool show) {
  uint16_t Coordinate_Temp;
  Coordinate_Temp = StartInfo_X_Coordinate[HMI_flag.language];
  if (show) {
    DWIN_ICON_Show(ICON, ICON_Info_1, 145, 246);
    DWIN_Draw_Rectangle(0, Color_White, 145, 246, 254, 345);
  	//DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_StartInfo, Menu_Coordinate,184,318);
  	DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_StartInfo], Coordinate_Temp, 318);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Info_0, 145, 246);
		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Main_Menu_StartInfo0, Menu_Coordinate, Coordinate_Temp,318);
  }
  DWIN_UpdateLCD();
}

#ifdef HAS_ONESTEP_LEVELING
void ICON_Leveling(bool show) {
  if (show) {
    DWIN_ICON_Show(ICON, ICON_Leveling_1, 145, 246);
    DWIN_Draw_Rectangle(0, Color_White, 145, 246, 254, 345);
    DWIN_Frame_AreaCopy(1, 84, 465, 120, 478, 182, 318);
  }
}
#endif

void ICON_Tune() {
  if (select_print.now == 0) {
    DWIN_ICON_Show(ICON, ICON_Setup_1, 8, 252);
    DWIN_Draw_Rectangle(0, Color_White, 8, 252, 87, 351);
    //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Tune, Menu_Coordinate,32, 325);
    DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Tune], Tune_X_Coordinate[HMI_flag.language], 325);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Setup_0, 8, 252);
		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Tune, Menu_Coordinate,Tune_X_Coordinate[HMI_flag.language], 325);
  }
  DWIN_UpdateLCD();
}

void ICON_Pause() {
  if ((select_print.now == 1)) {
    DWIN_ICON_Show(ICON, ICON_Pause_1, 96, 252);
    DWIN_Draw_Rectangle(0, Color_White, 96, 252, 175, 351);
    //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Pause, Menu_Coordinate,116, 325);
    DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Pause], Pause_X_Coordinate[HMI_flag.language], 325);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Pause_0, 96, 252);
		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Pause, Menu_Coordinate,Pause_X_Coordinate[HMI_flag.language], 325);
  }
  DWIN_UpdateLCD();
}

void ICON_Continue() {
  if (select_print.now == 1) {
    DWIN_ICON_Show(ICON, ICON_Continue_1, 96, 252);
    DWIN_Draw_Rectangle(0, Color_White, 96, 252, 175, 351);
    //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Print, Menu_Coordinate,116, 325);
    DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Continue], Continue_X_Coordinate[HMI_flag.language], 325);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Continue_0, 96, 252);
		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Continue, Menu_Coordinate,Continue_X_Coordinate[HMI_flag.language], 325);
  }
  DWIN_UpdateLCD();
}

void ICON_Stop() {
  if (select_print.now == 2) {
    DWIN_ICON_Show(ICON, ICON_Stop_1, 184, 252);
    DWIN_Draw_Rectangle(0, Color_White, 184, 252, 263, 351);
    //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Stop, Menu_Coordinate,208, 325);
    DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Stop], Stop_X_Coordinate[HMI_flag.language], 325);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Stop_0, 184, 252);
		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Stop, Menu_Coordinate,Stop_X_Coordinate[HMI_flag.language], 325);
  }
  DWIN_UpdateLCD();
}

void ICON_YESorNO(uint8_t Option){
	if (Option == false) {
    	DWIN_ICON_Show(ICON, ICON_YES_0, 26, 168);
			DWIN_ICON_Show(ICON, ICON_NO_1, 146, 168);
    	DWIN_Draw_Rectangle(0, Color_White, 26, 168, 126, 206);
  	}
	else{
			DWIN_ICON_Show(ICON, ICON_YES_1, 26, 168);
			DWIN_ICON_Show(ICON, ICON_NO_0, 146, 168);
    	DWIN_Draw_Rectangle(0, Color_White, 146, 168, 246, 206);
	}
}

void ICON_YESorNO_Powerdown(uint8_t Option){
	if (Option == false) {
    	DWIN_ICON_Show(ICON, ICON_NO_0, 26, 228);
			DWIN_ICON_Show(ICON, ICON_YES_1, 146, 228);
    	DWIN_Draw_Rectangle(0, Color_White, 26, 228, 126, 266);
	}
	else{
		DWIN_ICON_Show(ICON, ICON_NO_1, 26, 228);
		DWIN_ICON_Show(ICON, ICON_YES_0, 146, 228);
  	DWIN_Draw_Rectangle(0, Color_White, 146, 228, 246, 266);
	}
}


inline void Clear_Title_Bar() {
  DWIN_Draw_Rectangle(1, Color_Bg_Blue, 0, 0, DWIN_WIDTH, 30);
}

inline void Draw_Title(const char * const title) {
  DWIN_Draw_String(false, false, DWIN_FONT_HEAD, Color_White, Color_Bg_Blue, 14, 4, (char*)title);
}

inline void Draw_Wifi_Title(const char * const title) {
  DWIN_Draw_String(false, false, DWIN_FONT_HEAD, Color_White, Color_Bg_Blue, 14, 65, (char*)title);
}

inline void Draw_Reprint_Title(const char * const title) {
  DWIN_Draw_String(false, false, DWIN_FONT_HEAD, Color_White, Color_Bg_Blue, 14, 65, (char*)title);
}

inline void Draw_Title(const __FlashStringHelper * title) {
  DWIN_Draw_String(false, false, DWIN_FONT_HEAD, Color_White, Color_Bg_Blue, 14, 4, (char*)title);
}

inline void Clear_Menu_Area() {
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 31, DWIN_WIDTH, STATUS_Y_START);
}

inline void Clear_Main_Window() {
  Clear_Title_Bar();
  Clear_Menu_Area();
}

inline void Clear_Bottom_Area() {
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 448, DWIN_WIDTH, STATUS_Y_END);
}

inline void Clear_Popup_Area() {
  Clear_Title_Bar();
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 31, DWIN_WIDTH, DWIN_HEIGHT);
}

void Draw_Popup_Bkgd_105() {
  DWIN_Draw_Rectangle(1, Color_Bg_Window, 14, 105, 258, 374);
}

inline void Draw_More_Icon(const uint8_t line) {
  DWIN_ICON_Show(ICON, ICON_More, 226, MBASE(line) - 3);
}

inline void Draw_Menu_Cursor(const uint8_t line) {
  // DWIN_ICON_Show(ICON,ICON_Rectangle, 0, MBASE(line) - 18);
  DWIN_Draw_Rectangle(1, Rectangle_Color, 0, MBASE(line) - 18, 14, MBASE(line + 1) - 20);
}

inline void Erase_Menu_Cursor(const uint8_t line) {
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, MBASE(line) - 18, 14, MBASE(line + 1) - 20);
}

inline void Move_Highlight(const int16_t from, const uint16_t newline) {
  Erase_Menu_Cursor(newline - from);
  Draw_Menu_Cursor(newline);
}

inline void Add_Menu_Line() {
  Move_Highlight(1, MROWS);
  DWIN_Draw_Line(Line_Color, 16, MBASE(MROWS + 1) - 20, 256, MBASE(MROWS + 1) - 19);
}

inline void Scroll_Menu(const uint8_t dir) {
  DWIN_Frame_AreaMove(1, dir, MLINE, Color_Bg_Black, 0, 31, DWIN_WIDTH, 349);
  switch (dir) {
    case DWIN_SCROLL_DOWN: Move_Highlight(-1, 0); break;
    case DWIN_SCROLL_UP:   Add_Menu_Line(); break;
  }
}

inline uint16_t nr_sd_menu_items() {
  return card.get_num_Files() + !card.flag.workDirIsRoot;
}

inline void Draw_Menu_Icon(const uint8_t line, const uint8_t icon) {
  DWIN_ICON_Show(ICON, icon, 26, MBASE(line) - 3);
}

inline void Erase_Menu_Text(const uint8_t line) {
  DWIN_Draw_Rectangle(1, Color_Bg_Black, LBLX, MBASE(line) - 14, 271, MBASE(line) + 28);
}

inline void Draw_Menu_Line(const uint8_t line, const uint8_t icon=0, const char * const label=nullptr) {
  if (label) DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(line) - 1, (char*)label);
  if (icon) Draw_Menu_Icon(line, icon);
  DWIN_Draw_Line(Line_Color, 16, MBASE(line) + 33, 256, MBASE(line) + 34);
}

// The "Back" label is always on the first line
inline void Draw_Back_Label() {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Menu_Back, Menu_Coordinate,LBLX, MBASE(0));
  DWIN_UpdateLCD();
}

// Draw "Back" line at the top
inline void Draw_Back_First(const bool is_sel=true) {
  Draw_Menu_Line(0, ICON_Back);
  Draw_Back_Label();
  if (is_sel) Draw_Menu_Cursor(0);
}

inline bool Apply_Encoder(const ENCODER_DiffState &encoder_diffState, auto &valref) {
  if (encoder_diffState == ENCODER_DIFF_CW)
    valref += EncoderRate.encoderMoveValue;
  else if (encoder_diffState == ENCODER_DIFF_CCW)
    valref -= EncoderRate.encoderMoveValue;
  else if (encoder_diffState == ENCODER_DIFF_ENTER)
    return true;
  return false;
}

//
// Draw Menus
//
#define MOTION_CASE_RATE   		1
#define MOTION_CASE_ACCEL  		2
#define MOTION_CASE_JERK   		(MOTION_CASE_ACCEL + ENABLED(HAS_CLASSIC_JERK))
#define MOTION_CASE_STEPS  		(MOTION_CASE_JERK + 1)
#define MOTION_CASE_TOTAL  		MOTION_CASE_STEPS

#define PREPARE_CASE_HOME  		1
#define PREPARE_CASE_MOVE  		2
#define PREPARE_CASE_DISA  		3
#define PREPARE_CASE_LEVELING  	4
#define PREPARE_CASE_POWERDOWN  5
//#define PREPARE_CASE_ZOFF 		(PREPARE_CASE_POWERDOWN + ENABLED(BABYSTEPPING))
#define PREPARE_CASE_PLA  		(PREPARE_CASE_POWERDOWN + ENABLED(HAS_HOTEND))
#define PREPARE_CASE_ABS  		(PREPARE_CASE_PLA + ENABLED(HAS_HOTEND))
#define PREPARE_CASE_COOL 		(PREPARE_CASE_ABS + EITHER(HAS_HOTEND, HAS_HEATED_BED))
#define PREPARE_CASE_LANG 		(PREPARE_CASE_COOL + 1)
#define PREPARE_CASE_TOTAL 		PREPARE_CASE_LANG

#define CONTROL_CASE_TEMP 		1
#define CONTROL_CASE_MOVE   	2
#define CONTROL_CASE_MIXER  	3
#define CONTROL_CASE_CONFIG  	4
#define CONTROL_CASE_BLTOUCH 	(CONTROL_CASE_CONFIG + ENABLED(BLTOUCH))
#define CONTROL_CASE_SAVE  		(CONTROL_CASE_BLTOUCH + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_LOAD  		(CONTROL_CASE_SAVE + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_RESET 		(CONTROL_CASE_LOAD + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_INFO  		(CONTROL_CASE_RESET + 1)
#define CONTROL_CASE_TOTAL 		CONTROL_CASE_RESET

#define TUNE_CASE_SPEED 		1
#define TUNE_CASE_TEMP 			(TUNE_CASE_SPEED + ENABLED(HAS_HOTEND))
#define TUNE_CASE_BED  			(TUNE_CASE_TEMP + ENABLED(HAS_HEATED_BED))
#define TUNE_CASE_FAN  			(TUNE_CASE_BED + ENABLED(HAS_FAN))
#define TUNE_CASE_ZOFF 			(TUNE_CASE_FAN + ENABLED(BABYSTEPPING))
#define TUNE_CASE_MIXER 		(TUNE_CASE_ZOFF + 1)
#define TUNE_CASE_CONFIG 		(TUNE_CASE_MIXER + 1)
#define TUNE_CASE_TOTAL 		TUNE_CASE_CONFIG

#define TEMP_CASE_TEMP 			(0 + ENABLED(HAS_HOTEND))
#define TEMP_CASE_BED  			(TEMP_CASE_TEMP + ENABLED(HAS_HEATED_BED))
#define TEMP_CASE_FAN  			(TEMP_CASE_BED + ENABLED(HAS_FAN))
#define TEMP_CASE_PLA  			(TEMP_CASE_FAN + ENABLED(HAS_HOTEND))
#define TEMP_CASE_ABS  			(TEMP_CASE_PLA + ENABLED(HAS_HOTEND))
#define TEMP_CASE_TOTAL 		TEMP_CASE_ABS

#define PREHEAT_CASE_TEMP 		(0 + ENABLED(HAS_HOTEND))
#define PREHEAT_CASE_BED  		(PREHEAT_CASE_TEMP + ENABLED(HAS_HEATED_BED))
#define PREHEAT_CASE_FAN  		(PREHEAT_CASE_BED + ENABLED(HAS_FAN))
#define PREHEAT_CASE_SAVE 		(PREHEAT_CASE_FAN + ENABLED(EEPROM_SETTINGS))
#define PREHEAT_CASE_TOTAL 		PREHEAT_CASE_SAVE

#define BLTOUCH_CASE_RESET   	1
#define BLTOUCH_CASE_TEST  		2
#define BLTOUCH_CASE_STOW   	3
#define BLTOUCH_CASE_DEPLOY    	4
#define BLTOUCH_CASE_SW    		5
#define BLTOUCH_CASE_TOTAL  	BLTOUCH_CASE_SW

#define LANGUAGE_CASE_EN   		1
#define LANGUAGE_CASE_SP  		2
#define LANGUAGE_CASE_RU   		3
#define LANGUAGE_CASE_FR    	4
#define LANGUAGE_CASE_PO    	5
#define LANGUAGE_CASE_ZH  		6
#define LANGUAGE_CASE_TOTAL 	LANGUAGE_CASE_PO

#define MIXER_CASE_MANUAL   	1
#define MIXER_CASE_AUTO  		2
#define MIXER_CASE_RANDOM   	3
#define MIXER_CASE_VTOOL    	4
#define MIXER_CASE_TOTAL  		MIXER_CASE_VTOOL

#define CONFIG_CASE_RETRACT   	(ENABLED(FWRETRACT))
#define CONFIG_CASE_FILAMENT  	(CONFIG_CASE_RETRACT + ENABLED(FILAMENT_RUNOUT_SENSOR))
#define CONFIG_CASE_POWERLOSS   (CONFIG_CASE_FILAMENT + ENABLED(POWER_LOSS_RECOVERY))
#define CONFIG_CASE_SHUTDOWN    (CONFIG_CASE_POWERLOSS + ENABLED(OPTION_AUTOPOWEROFF))
#define CONFIG_TUNE_CASE_TOTAL  CONFIG_CASE_SHUTDOWN
#define CONFIG_CASE_WIFI        (CONFIG_CASE_SHUTDOWN + ENABLED(OPTION_WIFI_MODULE))
#define CONFIG_CASE_REPRINT     (CONFIG_CASE_WIFI + ENABLED(OPTION_REPEAT_PRINTING))
#define	CONFIG_CASE_COATING			(CONFIG_CASE_REPRINT + ENABLED(OPTION_BED_COATING))
#define CONFIG_CASE_LEVELING    (CONFIG_CASE_COATING + BOTH(ABL_GRID,AUTO_UPDATA_PROBE_Z_OFFSET))
#define CONFIG_CASE_ACTIVELEVEL (CONFIG_CASE_LEVELING + ENABLED(ABL_GRID))
#define CONFIG_CASE_RGB         (CONFIG_CASE_ACTIVELEVEL + ENABLED(RGB_LED))
#define CONFIG_CASE_M92         (CONFIG_CASE_RGB + ENABLED(DEBUG_GCODE_M92))
#define CONFIG_CASE_TOTAL  		CONFIG_CASE_M92

#define RETRACT_CASE_AUTO   	1
#define RETRACT_CASE_RETRACT_MM 2
#define RETRACT_CASE_RETRACT_V  3
#define RETRACT_CASE_RECOVER_MM 4
#define RETRACT_CASE_RECOVER_V  5
#define RETRACT_CASE_TOTAL  	RETRACT_CASE_RECOVER_V

#define REPRINT_CASE_ENABLED   	1
#define REPRINT_CASE_TIMES 			2
#define REPRINT_CASE_LENGHT  		3
#define REPRINT_CASE_RESET  		4
#define REPRINT_CASE_FORWARD  	5
#define REPRINT_CASE_BACK  			6
#define REPRINT_CASE_TOTAL  		REPRINT_CASE_BACK

#if ENABLED(MIXING_EXTRUDER)
	#define MANUAL_CASE_EXTRUDER1   1
	#define MANUAL_CASE_EXTRUDER2  	2
  	#if(MIXING_STEPPERS > 2) 		
		#define MANUAL_CASE_EXTRUDER3   3
	#endif
	#if(MIXING_STEPPERS > 3) 		
		#define MANUAL_CASE_EXTRUDER4  	4
	#endif
	#define MANUAL_CASE_OK  		(MIXING_STEPPERS+1)
	#define MANUAL_CASE_TOTAL       MANUAL_CASE_OK
#endif

#define AUTO_CASE_ZPOS_START  	1
#define AUTO_CASE_ZPOS_END  	2
#define AUTO_CASE_VTOOL_START  	3
#define AUTO_CASE_VTOOL_END  	4
#define AUTO_CASE_TOTAL       	AUTO_CASE_VTOOL_END

#define RANDOM_CASE_ZPOS_START  1
#define RANDOM_CASE_ZPOS_END  	2
#define RANDOM_CASE_HEIGHT 		3
#define RANDOM_CASE_EXTRUDERS  	4
#define RANDOM_CASE_TOTAL       RANDOM_CASE_EXTRUDERS


#define AXISMOVE_CASE_MOVEX   	1
#define AXISMOVE_CASE_MOVEY  	2
#define AXISMOVE_CASE_MOVEZ   	3
#if HAS_HOTEND
  #define AXISMOVE_CASE_EX1  		4
	#if(E_STEPPERS > 1) 
  #define AXISMOVE_CASE_EX2  		(E_STEPPERS+1)
	#endif
  #if(E_STEPPERS > 2) 
	#define AXISMOVE_CASE_EX3  		(E_STEPPERS+2)	
  #endif
  #if(E_STEPPERS > 3)
	#define AXISMOVE_CASE_EX4  		(E_STEPPERS+3)
  #endif
	#if ENABLED(MIXING_EXTRUDER)
	#define AXISMOVE_CASE_EXALL  	(E_STEPPERS+4)
	#endif
  #define AXISMOVE_CASE_TOTAL     (E_STEPPERS + 3 + ENABLED(MIXING_EXTRUDER))
#endif

#define LEVELING_CASE_POINT1   					1
#define LEVELING_CASE_POINT2  					2
#define LEVELING_CASE_POINT3   					3
#define LEVELING_CASE_POINT4    				4
#define LEVELING_CASE_CATCHOFFSET   		(LEVELING_CASE_POINT4 + ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET))
#define LEVELING_CASE_SAVE    					(LEVELING_CASE_CATCHOFFSET + ENABLED(LCD_BED_LEVELING))
//#define LEVELING_CASE_TOTAL  		LEVELING_CASE_SAVE

#define HOME_CASE_ALL   			1
#define HOME_CASE_X  				2
#define HOME_CASE_Y   				3
#define HOME_CASE_Z    				4
#define HOME_CASE_TOTAL  			HOME_CASE_Z

#define INFO_CASE_VERSION    		1
#define INFO_CASE_FIRMWARE   		2
#define INFO_CASE_WEBSITE    		3
#define INFO_CASE_MODEL      		4
#define INFO_CASE_BOARD      		5
#define INFO_CASE_EXTRUDER_NUM   	6
#define INFO_CASE_EXTRUDER_MODEL    7
#define INFO_CASE_DUALZ_DRIVE      (INFO_CASE_EXTRUDER_MODEL + ENABLED(OPTION_DUALZ_DRIVE)) 
#define INFO_CASE_DUALZ_ENDSTOP    (INFO_CASE_DUALZ_DRIVE + ENABLED(OPTION_Z2_ENDSTOP))
#define INFO_CASE_BAUDRATE   	   (INFO_CASE_DUALZ_ENDSTOP + 1)
#define INFO_CASE_PROTOCOL   	   (INFO_CASE_BAUDRATE + 1)
#define INFO_CASE_PSU   	       (INFO_CASE_PROTOCOL + 1)
#define INFO_CASE_DATE   		   (INFO_CASE_PSU + 1)
#define INFO_CASE_THERMISTOR   	   (INFO_CASE_DATE + 1)
#define INFO_CASE_BED   	       (INFO_CASE_THERMISTOR + 1)
#define INFO_CASE_HOT   	       (INFO_CASE_BED + 1)
#define INFO_CASE_TOTAL  	 	    INFO_CASE_HOT
/*
//
// Draw Menus
//
inline void draw_move_en(const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 69, 61, 102, 71, LBLX, line); 			// "Move"
}
*/
inline void DWIN_Frame_TitleCopy(uint8_t id, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) { DWIN_Frame_AreaCopy(id, x1, y1, x2, y2, 14, 8); }

#if ENABLED(OPTION_REPEAT_PRINTING) 
inline void Item_Reprint_Enabled(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Repeat Print"));
  Draw_Menu_Line(row,ICON_Cursor);
	DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), ReprintManager.enabled?F("ON"):F("OFF"));
}

inline void Item_Reprint_Times(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Reprint Times"));
  Draw_Menu_Line(row,ICON_Cursor);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 216, MBASE(row), ReprintManager.Reprint_times);
}

inline void Item_Reprint_Lenght(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Forward Lenght"));
  Draw_Menu_Line(row,ICON_Cursor);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 216, MBASE(row), ReprintManager.Forward_lenght);
}

inline void Item_Reprint_Reset(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Reset"));
  Draw_Menu_Line(row,ICON_Cursor);
}

inline void Item_Reprint_Forward(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Forward Move"));
  Draw_Menu_Line(row,ICON_Cursor);
}

inline void Item_Reprint_Back(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Back Move"));
  Draw_Menu_Line(row,ICON_Cursor);
}
#endif

#if ENABLED(FWRETRACT) 
inline void Item_Config_Retract(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Auto Retract"));
  Draw_Menu_Line(row,ICON_Cursor);
  Draw_More_Icon(row);
}
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
inline void Item_Config_Filament(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Runout Sensor:"));
  Draw_Menu_Line(row,ICON_Cursor);
  if(runout.enabled)DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("ON"));
  else DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("OFF"));
}
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
inline void Item_Config_Powerloss(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Powerloss:"));
  Draw_Menu_Line(row,ICON_Cursor);
  if(recovery.enabled)DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("ON"));
  else DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("OFF"));
}
#endif

#if ENABLED(OPTION_AUTOPOWEROFF)
inline void Item_Config_Shutdown(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Auto Shutdown:"));
  Draw_Menu_Line(row,ICON_Cursor);
  if(HMI_flag.auto_shutdown) DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("ON"));
  else DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("OFF"));
}
#endif

#if ENABLED(OPTION_WIFI_MODULE)
inline void Item_Config_Wifi(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Wifi:"));
  Draw_Menu_Line(row,ICON_Cursor);
  if(WiFi_Enabled) 
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("ON"));
  else 
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("OFF"));
}
#endif

#if ENABLED(OPTION_REPEAT_PRINTING)
inline void Item_Config_Reprint(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Re-print:"));
  Draw_Menu_Line(row,ICON_Cursor);
  Draw_More_Icon(row);
}
#endif

#if ENABLED(OPTION_BED_COATING)
inline void Item_Config_bedcoating(const uint8_t row) {  
  HMI_ValueStruct.coating_thickness = (int16_t)(coating_thickness * 100);
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Coating Thickness:"));
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 2, 216, MBASE(row), HMI_ValueStruct.coating_thickness);
  Draw_Menu_Line(row,ICON_Cursor);
}
#endif

#if ENABLED(ABL_GRID)
#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
inline void Item_Config_Leveling(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Auto Leveling:"));
  if(HMI_flag.Auto_Leveling_Menu_Fg)
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("ON"));
  else 
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("OFF"));
  Draw_Menu_Line(row,ICON_Cursor);
}
#endif
inline void Item_Config_ActiveLevel(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("Active Autolevel:"));
  if(planner.leveling_active)
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("ON"));
  else 
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(row), F("OFF"));
  Draw_Menu_Line(row,ICON_Cursor);
}
#endif

#if HAS_COLOR_LEDS
inline void Item_Config_RGB(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("RGB Color:"));
  Draw_Menu_Line(row,ICON_Cursor);
  switch(HMI_flag.RGB_LED_Counter) {
  	case RED_ON:
  		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(row), F("RED"));
    break;
	case GREEN_ON:
  		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(row), F("GREEN"));
    break;
	case BLUE_ON:
  		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(row), F("BLUE"));
    break;
	case YELLOW_ON:
  		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(row), F("YELLOW"));
    break;
	case CYAN_ON:
  		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(row), F("CYAN"));
    break;
	case PURPLE_ON:
  		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(row), F("PURPLE"));
    break;
	case WHITE_ON:
  		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(row), F("WHITE"));
    break;
	default:
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(row), F("BLACK"));
		break;
  }
}
#endif

#if ENABLED(DEBUG_GCODE_M92)
inline void Item_Config_M92(const uint8_t row) {
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), F("M92"));
  Draw_Menu_Line(row,ICON_Cursor);
}
#endif


inline void Item_Info_Version(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Version:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Version:")+1)*MENU_CHR_W, MBASE(row), (char*)FIRMWARE_VERSION);
}

inline void Item_Info_Firmware(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Firmware:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Firmware:")+1)*MENU_CHR_W, MBASE(row), (char*)SHORT_BUILD_VERSION);
}

inline void Item_Info_Website(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Website:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Website:")+1)*MENU_CHR_W, MBASE(row), (char*)WEBSITE_URL);
}

inline void Item_Info_Model(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Model:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Model:")+1)*MENU_CHR_W, MBASE(row), (char*)CUSTOM_MACHINE_NAME);
}

inline void Item_Info_Board(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Board:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Board:")+1)*MENU_CHR_W, MBASE(row), (char*)BOARD_INFO_NAME);
}

inline void Item_Info_Extruder_Num(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Extruder Num:"));
  #if ENABLED(MIXING_EXTRUDER)  
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Extruder Num:")+1)*MENU_CHR_W, MBASE(row),(char*)STRINGIFY(MIXING_STEPPERS));
  #else
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Extruder Num:")+1)*MENU_CHR_W, MBASE(row),(char*)STRINGIFY(EXTRUDERS));
  #endif
}

#if ENABLED(OPTION_BGM)
#define EXTRUDER_MODEL				"BGM"
#else
#define EXTRUDER_MODEL				"Titan"
#endif
inline void Item_Info_Extruder_Model(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Extruder Model:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Extruder Model:")+1)*MENU_CHR_W, MBASE(row), (char*)EXTRUDER_MODEL);
}


#if ENABLED(OPTION_DUALZ_DRIVE)
#define Z_Drive						"DUAL Z"
inline void Item_Info_DualZ_Drive(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Z Drive:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Z Drive:")+1)*MENU_CHR_W, MBASE(row), (char*)Z_Drive);
}
#endif

#if ENABLED(OPTION_Z2_ENDSTOP)
#define Z_Endstop					"DUAL Endstop"
inline void Item_Info_DualZ_Endstop(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Z Endstop:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Z Endstop:")+1)*MENU_CHR_W, MBASE(row), (char*)Z_Endstop);
}
#endif

inline void Item_Info_Baudrate(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Baudrate:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Baudrate:")+1)*MENU_CHR_W, MBASE(row), (char*)STRINGIFY(BAUDRATE));
}

inline void Item_Info_Protocol(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Protocol:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Protocol:")+1)*MENU_CHR_W, MBASE(row), (char*)PROTOCOL_VERSION);
}

inline void Item_Info_Psu(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("PSU:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("PSU:")+1)*MENU_CHR_W, MBASE(row), (char*)PSU_NAME);
}

inline void Item_Info_Date(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Date:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Date:")+1)*MENU_CHR_W, MBASE(row), (char*)STRING_DISTRIBUTION_DATE);
}

inline void Item_Info_Thermistor(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Thermistor:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Thermistor:")+1)*MENU_CHR_W, MBASE(row), F("100k with 4.7k pull-up"));
  //DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Thermistor:")+1)*MENU_CHR_W, MBASE(row), (char*)STRINGIFY(THERMISTOR_HEATER_0));
}

inline void Item_Info_Bed(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Hot bed:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Hot bed:"))*MENU_CHR_W, MBASE(row), F("MINTEMP:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("MINTEMP:") + strlen("Hot bed:"))*MENU_CHR_W, MBASE(row), (char*)STRINGIFY(BED_MINTEMP));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 175, MBASE(row), F("MAXTEMP:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 175 + (strlen("MAXTEMP:"))*MENU_CHR_W, MBASE(row), (char*)STRINGIFY(BED_MAXTEMP));
}

inline void Item_Info_Hot(const uint8_t row) {
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, MBASE(row), F("Hot end:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Hot end:"))*MENU_CHR_W, MBASE(row), F("MINTEMP:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("MINTEMP:")+strlen("Hot end:"))*MENU_CHR_W, MBASE(row), (char*)STRINGIFY(HEATER_0_MINTEMP));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 175, MBASE(row), F("MAXTEMP:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 175 + (strlen("MAXTEMP:"))*MENU_CHR_W, MBASE(row), (char*)STRINGIFY(HEATER_0_MAXTEMP));
}

#if ENABLED(ABL_GRID)
#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
inline void Item_Leveling_Auto(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Catch, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Z_Offset, Menu_Coordinate,LBLX+Catch_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_Leveling_Save);
}
#endif
inline void Item_Leveling_Save(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Auto, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Level, Menu_Coordinate,LBLX+Auto_X_Coordinate[HMI_flag.language], MBASE(row));
  if(planner.leveling_active)
		DWIN_Draw_String(false, true, font8x16, Color_Green, Color_Bg_Black, 186, MBASE(row), F("Actived"));
  else
  	DWIN_Draw_String(false, true, font8x16, Color_Bg_Red, Color_Bg_Black, 170, MBASE(row), F("Unactived"));
  Draw_Menu_Line(row, ICON_Leveling_Save);
}
#endif

inline void Item_Leveling_Point1(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Point, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_1, Menu_Coordinate,LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_Leveling_Point1);
}

inline void Item_Leveling_Point2(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Point, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_2, Menu_Coordinate,LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_Leveling_Point2);
}

inline void Item_Leveling_Point3(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Point, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_3, Menu_Coordinate,LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_Leveling_Point3);
}

inline void Item_Leveling_Point4(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Point, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_4, Menu_Coordinate,LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_Leveling_Point4);
}

inline void Item_Control_Temp(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Temperature, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Temperature);
  Draw_More_Icon(row);
}

inline void Item_Control_Motion(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Motion, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Motion);
  Draw_More_Icon(row);
}

inline void Item_Control_Mixer(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Mixer, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Mixer);
  Draw_More_Icon(row);
}

inline void Item_Control_Config(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Config, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Contact);
  Draw_More_Icon(row);
}

#if ENABLED(BLTOUCH)
inline void Item_Control_BLtouch(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Bltouch, Menu_Coordinate,LBLX, MBASE(row));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Settings, Menu_Coordinate,LBLX+Load_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_BLTouch);
}
#endif

inline void Item_Control_Save(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Store, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Settings, Menu_Coordinate,LBLX+Store_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_WriteEEPROM);
}

inline void Item_Control_Load(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Load, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Settings, Menu_Coordinate,LBLX+Load_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_ReadEEPROM);
}

inline void Item_Control_Reset(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Reset, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_ResumeEEPROM);
}

inline void Item_Control_Info(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Info, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Info);
  Draw_More_Icon(row);
}

inline void Item_Language_ZH(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_ZH, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_CH);
}

inline void Item_Language_EN(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_EN, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_EN);
}

inline void Item_Tune_Mixer(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Mixer, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Mixer);
  Draw_More_Icon(row);
}

inline void Item_Tune_Config(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Config, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Contact);
  Draw_More_Icon(row);
}


inline void Item_Tune_Speed(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Speed, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Setspeed);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(row), feedrate_percentage);
}

inline void Item_Tune_Temperture(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Temp, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_HotendTemp);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(row), thermalManager.temp_hotend[0].target);
}


inline void Item_Axis_MoveX(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_X, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_MoveX);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(row), current_position.x * MINUNITMULT);
}

inline void Item_Axis_MoveY(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Y, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_MoveY);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(row), current_position.y * MINUNITMULT);
}

inline void Item_Axis_MoveZ(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Z, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_MoveZ);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(row), current_position.z * MINUNITMULT);
}
#if HAS_HOTEND
inline void Item_Axis_MoveEX1(const uint8_t row) {
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_1, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_Extruder1);
	DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(row), HMI_ValueStruct.Move_E1_scale);
}

#if (E_STEPPERS > 1)
inline void Item_Axis_MoveEX2(const uint8_t row) {
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_2, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_Extruder2);
	DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(row), HMI_ValueStruct.Move_E2_scale);
}
#endif

#if (E_STEPPERS > 2)
inline void Item_Axis_MoveEX3(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_3, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_Extruder3);
	DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(row), HMI_ValueStruct.Move_E3_scale);
}
#endif
#if (E_STEPPERS > 3)
inline void Item_Axis_MoveEX4(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_4, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_Extruder4);
	DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(row), HMI_ValueStruct.Move_E4_scale);
}
#endif
#if ENABLED(MIXING_EXTRUDER)
inline void Item_Axis_MoveEXAll(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_All, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_Language);
	DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(row), HMI_ValueStruct.Move_EAll_scale);
}
#endif
#endif


inline void Item_Prepare_Move(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Move, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Axis);
  Draw_More_Icon(row);
}

inline void Item_Prepare_Disable(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Disable_Steppers, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_CloseMotor);
}

inline void Item_Prepare_Home(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Auto_Home, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Homing);
  Draw_More_Icon(row);
}

inline void Item_Prepare_Leveling(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Bed, Menu_Coordinate,LBLX, MBASE(row));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Auto, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Leveling, Menu_Coordinate,LBLX+Auto_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_Leveling0);
  Draw_More_Icon(row);
}

inline void Item_Prepare_Powerdown(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Power_Outage, Menu_Coordinate,LBLX, MBASE(row));
  if(HMI_flag.language == 0)
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Power_Off, Menu_Coordinate,LBLX+56, MBASE(row));
  Draw_Menu_Line(row, ICON_POWERDOWN);
}



#if BOTH(BABYSTEPPING, PREPARE_CASE_ZOFF)
inline void Item_Prepare_Offset(const uint8_t row) {
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Z_Offset, Menu_Coordinate,LBLX, MBASE(row));
	HMI_ValueStruct.zoffset_value = (int16_t)(babyz_offset * 100);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 2, 202, MBASE(PREPARE_CASE_ZOFF + MROWS - index_prepare), HMI_ValueStruct.zoffset_value);
  Draw_Menu_Line(row, ICON_SetHome);
}
#endif

#if HAS_HOTEND
  inline void Item_Prepare_PLA(const uint8_t row) {
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Preheat, Menu_Coordinate,LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_PLA, Menu_Coordinate,LBLX+Preheat_X_Coordinate[HMI_flag.language], MBASE(row));
    Draw_Menu_Line(row, ICON_PLAPreheat);
  }

  inline void Item_Prepare_ABS(const uint8_t row) {
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Preheat, Menu_Coordinate,LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_ABS, Menu_Coordinate,LBLX+Preheat_X_Coordinate[HMI_flag.language], MBASE(row));
    Draw_Menu_Line(row, ICON_ABSPreheat);
  }
#endif

#if HAS_PREHEAT
  inline void Item_Prepare_Cool(const uint8_t row) {
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Cooldown, Menu_Coordinate,LBLX, MBASE(row));
    Draw_Menu_Line(row, ICON_Cool);
  }
#endif

inline void Item_Prepare_Lang(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Language, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Language);
  Draw_More_Icon(row);
}

inline void Draw_Prepare_Menu() {
  Clear_Main_Window();
  Clear_Bottom_Area();

  const int16_t pscroll = MROWS - index_prepare; // Scrolled-up lines
  #define PSCROL(L) (pscroll + (L))
  #define PVISI(L)  WITHIN(PSCROL(L), 0, MROWS)
  
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Prepare, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  
  if (PVISI(0)) Draw_Back_First(select_prepare.now == 0);                         // < Back  
  if (PVISI(PREPARE_CASE_MOVE)) Item_Prepare_Move(PSCROL(PREPARE_CASE_MOVE));     // Move >
  if (PVISI(PREPARE_CASE_DISA)) Item_Prepare_Disable(PSCROL(PREPARE_CASE_DISA));  // Disable Stepper
  if (PVISI(PREPARE_CASE_HOME)) Item_Prepare_Home(PSCROL(PREPARE_CASE_HOME));     // Auto Home
  if (PVISI(PREPARE_CASE_LEVELING)) Item_Prepare_Leveling(PSCROL(PREPARE_CASE_LEVELING));     // Leveling
  if (PVISI(PREPARE_CASE_POWERDOWN)) Item_Prepare_Powerdown(PSCROL(PREPARE_CASE_POWERDOWN));     // Powerdown
  #if BOTH(BABYSTEPPING, PREPARE_CASE_ZOFF)
    if (PVISI(PREPARE_CASE_ZOFF)) Item_Prepare_Offset(PSCROL(PREPARE_CASE_ZOFF)); // Edit Z-Offset / Babystep / Set Home Offset
  #endif
  #if HAS_HOTEND
    if (PVISI(PREPARE_CASE_PLA)) Item_Prepare_PLA(PSCROL(PREPARE_CASE_PLA));      // Preheat PLA
    if (PVISI(PREPARE_CASE_ABS)) Item_Prepare_ABS(PSCROL(PREPARE_CASE_ABS));      // Preheat ABS
  #endif
  #if HAS_PREHEAT
    if (PVISI(PREPARE_CASE_COOL)) Item_Prepare_Cool(PSCROL(PREPARE_CASE_COOL));   // Cooldown
  #endif
  if (PVISI(PREPARE_CASE_LANG)) Item_Prepare_Lang(PSCROL(PREPARE_CASE_LANG));     // Language CN/EN

  if (select_prepare.now) Draw_Menu_Cursor(PSCROL(select_prepare.now));
}

inline void Draw_Control_Menu() {
  Clear_Main_Window();
  Clear_Bottom_Area();

  #if CONTROL_CASE_TOTAL >= 6
    const int16_t cscroll = MROWS - index_control; // Scrolled-up lines
    #define CCSCROL(L) (cscroll + (L))
  #else
    #define CCSCROL(L) (L)
  #endif
  #define CCLINE(L)  MBASE(CCSCROL(L))
  #define CCVISI(L)  WITHIN(CCSCROL(L), 0, MROWS)
  
  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Control], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Control, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  if (CCVISI(0)) Draw_Back_First(select_control.now == 0);                         // < Back

  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Temperature, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_TEMP));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Motion, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_MOVE));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Mixer, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_MIXER));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Config, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_CONFIG));
  #if ENABLED(BLTOUCH)
	  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Bltouch, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_BLTOUCH));
	//#if ENABLED(EEPROM_SETTINGS)
  	  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Store, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_SAVE));
	//#endif
  #else
    #if ENABLED(EEPROM_SETTINGS)
  	  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Store, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_SAVE));
	  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Settings, Menu_Coordinate,LBLX+Store_X_Coordinate[HMI_flag.language], CCLINE(CONTROL_CASE_SAVE));
	#endif
  #endif
  
  if (select_control.now && CCVISI(select_control.now))
    Draw_Menu_Cursor(CCSCROL(select_control.now));

  Draw_Menu_Line(CCSCROL(1),ICON_Temperature);
  Draw_More_Icon(CCSCROL(1));

  Draw_Menu_Line(CCSCROL(2),ICON_Motion);
  Draw_More_Icon(CCSCROL(2));

  Draw_Menu_Line(CCSCROL(3),ICON_Mixer);
  Draw_More_Icon(CCSCROL(3));

  Draw_Menu_Line(CCSCROL(4),ICON_Contact);
  Draw_More_Icon(CCSCROL(4));

  #if ENABLED(BLTOUCH)
  	Draw_Menu_Line(CCSCROL(5),ICON_BLTouch);
    Draw_More_Icon(CCSCROL(5));
    //#if ENABLED(EEPROM_SETTINGS)
  		//Draw_Menu_Line(CCSCROL(5),ICON_WriteEEPROM);
	//#endif
  #else
    #if ENABLED(EEPROM_SETTINGS)
    	Draw_Menu_Line(CCSCROL(5),ICON_WriteEEPROM);
  		//Draw_Menu_Line(CCSCROL(5),ICON_ReadEEPROM);
	#endif
  #endif
}

inline void Draw_Tune_Menu() {
  Clear_Main_Window();
  Clear_Bottom_Area();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Tune], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Tune, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Speed, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_SPEED));
  #if HAS_HOTEND
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Hotend, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_TEMP));
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Temp, Menu_Coordinate,LBLX+Hotend_X_Coordinate[HMI_flag.language], MBASE(TUNE_CASE_TEMP));
  #endif
  #if HAS_HEATED_BED
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Bed, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_BED));
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Temp, Menu_Coordinate,LBLX+Bed_X_Coordinate[HMI_flag.language], MBASE(TUNE_CASE_BED));
  #endif
  #if HAS_FAN
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Fan_Speed, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_FAN));
  #endif
  #if ENABLED(BABYSTEPPING)
  	//DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Probe, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_ZOFF));
    //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Z_Offset, Menu_Coordinate,LBLX+Probe_X_Coordinate[HMI_flag.language], MBASE(TUNE_CASE_ZOFF));
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Z_Offset, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_ZOFF));
  #endif
  
  Draw_Back_First(select_tune.now == 0);
  if (select_tune.now) Draw_Menu_Cursor(select_tune.now);

  Draw_Menu_Line(TUNE_CASE_SPEED, ICON_Speed);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_SPEED), feedrate_percentage);

  #if HAS_HOTEND
    Draw_Menu_Line(TUNE_CASE_TEMP, ICON_HotendTemp);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_TEMP), thermalManager.temp_hotend[0].target);
  #endif
  
  #if HAS_HEATED_BED
    Draw_Menu_Line(TUNE_CASE_BED, ICON_BedTemp);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_BED), thermalManager.temp_bed.target);
  #endif
  
  #if HAS_FAN
    Draw_Menu_Line(TUNE_CASE_FAN, ICON_FanSpeed);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_FAN), thermalManager.fan_speed[0]);
  #endif
  
  #if ENABLED(BABYSTEPPING)
    Draw_Menu_Line(TUNE_CASE_ZOFF, ICON_Zoffset);
    DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 2, 202, MBASE(TUNE_CASE_ZOFF), HMI_ValueStruct.zoffset_value);
  #endif
}

inline void Draw_Motion_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Motion], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Motion, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Motion_Menu_Feedrate, Menu_Coordinate,LBLX, MBASE(MOTION_CASE_RATE));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Motion_Menu_Acc, Menu_Coordinate,LBLX, MBASE(MOTION_CASE_ACCEL));
  #if HAS_CLASSIC_JERK
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Motion_Menu_Jerk, Menu_Coordinate,LBLX, MBASE(MOTION_CASE_JERK));
  #endif 
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Motion_Menu_Steps, Menu_Coordinate,LBLX, MBASE(MOTION_CASE_STEPS));
  
  Draw_Back_First(select_motion.now == 0);
  if (select_motion.now) Draw_Menu_Cursor(select_motion.now);

  uint8_t i = 0;
  #define _MOTION_ICON(N) Draw_Menu_Line(++i, ICON_MaxSpeed + (N) - 1)
  _MOTION_ICON(MOTION_CASE_RATE); Draw_More_Icon(i);
  _MOTION_ICON(MOTION_CASE_ACCEL); Draw_More_Icon(i);
  #if HAS_CLASSIC_JERK
    _MOTION_ICON(MOTION_CASE_JERK); Draw_More_Icon(i);
  #endif
  _MOTION_ICON(MOTION_CASE_STEPS); Draw_More_Icon(i);
}

//
// Draw Leveling Windows
//
inline void Draw_Leveling_Menu() {
  Clear_Main_Window();
	
#if ENABLED(ABL_GRID)
	if(HMI_flag.Auto_Leveling_Menu_Fg){
		HMI_flag.Leveling_Case_Total = 5 + ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET);
	}
	else 
		HMI_flag.Leveling_Case_Total = 4;
	HMI_flag.need_home_flag = true;
#endif
  const int16_t lscroll = MROWS - index_leveling; // Scrolled-up lines
  #define LSCROL(L) (lscroll + (L))
  #define LVISI(L)  WITHIN(LSCROL(L), 0, MROWS)

  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Leveling, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);

  if (LVISI(0)) Draw_Back_First(select_leveling.now == 0);                         		  // < Back  
  if (LVISI(LEVELING_CASE_POINT1)) Item_Leveling_Point1(LSCROL(LEVELING_CASE_POINT1));     // point1
  if (LVISI(LEVELING_CASE_POINT2)) Item_Leveling_Point2(LSCROL(LEVELING_CASE_POINT2));     // point2
  if (LVISI(LEVELING_CASE_POINT3)) Item_Leveling_Point3(LSCROL(LEVELING_CASE_POINT3));     // point3
  if (LVISI(LEVELING_CASE_POINT4)) Item_Leveling_Point4(LSCROL(LEVELING_CASE_POINT4));     // point4

  #if ENABLED(ABL_GRID)
  	if(HMI_flag.Auto_Leveling_Menu_Fg){
			#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
			if (LVISI(LEVELING_CASE_CATCHOFFSET)) Item_Leveling_Auto(LSCROL(LEVELING_CASE_CATCHOFFSET));     // auto
			#endif
			if (LVISI(LEVELING_CASE_SAVE)) Item_Leveling_Save(LSCROL(LEVELING_CASE_SAVE));     // save
  	}
  #endif
  
  if (select_leveling.now) Draw_Menu_Cursor(LSCROL(select_leveling.now));

  #if ENABLED(ABL_GRID)
  	uint8_t i,j=ICON_Leveling_Point1;
  	for(i=LEVELING_CASE_POINT1; i<HMI_flag.Leveling_Case_Total; i++,j++) {
  		Draw_Menu_Line(i,j);
  	}
  #else
    uint8_t i,j=ICON_Leveling_Point1;
  	for(i=LEVELING_CASE_POINT1; i<HMI_flag.Leveling_Case_Total+LEVELING_CASE_POINT1; i++,j++) {
  		Draw_Menu_Line(i,j);
  	}
  #endif
}

//
// Draw Home Windows
//
inline void Draw_Home_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Home], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Home, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Home, Menu_Coordinate,LBLX, MBASE(HOME_CASE_ALL));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_All, Menu_Coordinate,LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_ALL));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Home, Menu_Coordinate,LBLX, MBASE(HOME_CASE_X));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_X, Menu_Coordinate,LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_X));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Home, Menu_Coordinate,LBLX, MBASE(HOME_CASE_Y));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Y, Menu_Coordinate,LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_Y));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Home, Menu_Coordinate,LBLX, MBASE(HOME_CASE_Z));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Z, Menu_Coordinate,LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_Z));
  
  Draw_Back_First(select_home.now == 0);
  if (select_home.now) Draw_Menu_Cursor(select_home.now);

  uint8_t i,j=ICON_HOME_ALL;
  for(i=HOME_CASE_ALL; i<HOME_CASE_TOTAL+1; i++,j++) {
  	Draw_Menu_Line(i,j);
  }
}

//
// Draw Bltouch Windows
//
#if ENABLED(BLTOUCH)
inline void Draw_Bltouch_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_BLTouch], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_BLTouch, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, BLTouch_Menu_Reset, Menu_Coordinate,LBLX, MBASE(BLTOUCH_CASE_RESET));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, BLTouch_Menu_Test, Menu_Coordinate,LBLX, MBASE(BLTOUCH_CASE_TEST));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, BLTouch_Menu_Stow, Menu_Coordinate,LBLX, MBASE(BLTOUCH_CASE_STOW));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, BLTouch_Menu_Deploy, Menu_Coordinate,LBLX, MBASE(BLTOUCH_CASE_DEPLOY));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, BLTouch_Menu_Mode, Menu_Coordinate,LBLX, MBASE(BLTOUCH_CASE_SW));

  Draw_Back_First(select_bltouch.now == 0);
  if (select_bltouch.now) Draw_Menu_Cursor(select_bltouch.now);

  uint8_t i,j=ICON_BLTOUCH_RESET;
  for(i=BLTOUCH_CASE_RESET; i<BLTOUCH_CASE_TOTAL+1; i++,j++) {
  	Draw_Menu_Line(i,j);
  }
}
#endif

//
// Draw Language Windows
//
inline void Draw_Language_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Mixer], 14, 7);
 
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Language, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_EN, Menu_Coordinate,LBLX, MBASE(LANGUAGE_CASE_EN));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_SP, Menu_Coordinate,LBLX, MBASE(LANGUAGE_CASE_SP));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_RU, Menu_Coordinate,LBLX, MBASE(LANGUAGE_CASE_RU));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_FR, Menu_Coordinate,LBLX, MBASE(LANGUAGE_CASE_FR));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_PO, Menu_Coordinate,LBLX, MBASE(LANGUAGE_CASE_PO));

  Draw_Back_First(select_language.now == 0);
  if (select_language.now) Draw_Menu_Cursor(select_language.now);

  uint8_t i,j=ICON_EN;
  for(i=LANGUAGE_CASE_EN; i<LANGUAGE_CASE_TOTAL+1; i++,j++) {
  	Draw_Menu_Line(i,j);
  }
}

//
// Draw Babystep Windows
//
inline void Draw_Babystep_Menu() {
	Clear_Main_Window();
	DWIN_Draw_String(false, false, font14x28, Color_White, Color_Bg_Black, 10, 160, F("Babysteps:"));
	DWIN_Draw_Signed_Float(font14x28, Color_White, Color_Bg_Black, 2, 2,170, 160, HMI_ValueStruct.zoffset_value);
}

//
// Draw Config Windows
//
inline void Draw_Config_Menu() {
  Clear_Main_Window();
  Clear_Bottom_Area();
  //uint8_t id = 0;

  const int16_t coscroll = MROWS - index_config; // Scrolled-up lines
  #define COSCROL(L) (coscroll + (L))
  #define COVISI(L)  WITHIN(COSCROL(L), 0, MROWS)

  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Blue, 14, 7, F("Configure"));
  Draw_Back_First(select_config.now == 0);

  if (COVISI(0)) Draw_Back_First(select_config.now == 0);                         			// < Back  

  #if ENABLED(FWRETRACT) 
  if (COVISI(CONFIG_CASE_RETRACT)) 	Item_Config_Retract(COSCROL(CONFIG_CASE_RETRACT));     	// Retract >
  #endif 
  
  #if ENABLED(FILAMENT_RUNOUT_SENSOR) 
  if (COVISI(CONFIG_CASE_FILAMENT)) Item_Config_Filament(COSCROL(CONFIG_CASE_FILAMENT));    // filament runout
  #endif 
  
  #if ENABLED(POWER_LOSS_RECOVERY) 
  if (COVISI(CONFIG_CASE_POWERLOSS)) Item_Config_Powerloss(COSCROL(CONFIG_CASE_POWERLOSS));   // powerloss
  #endif
  
  #if ENABLED(OPTION_AUTOPOWEROFF) 
  if (COVISI(CONFIG_CASE_SHUTDOWN)) Item_Config_Shutdown(COSCROL(CONFIG_CASE_SHUTDOWN));   	  // shutdown
  #endif
	
  if(!IS_SD_PRINTING() && !IS_SD_PAUSED()){
	  #if ENABLED(OPTION_WIFI_MODULE) 
	  if (COVISI(CONFIG_CASE_WIFI)) 	Item_Config_Wifi(COSCROL(CONFIG_CASE_WIFI));   	  			// WIFI
	  #endif

	  #if ENABLED(OPTION_REPEAT_PRINTING) 
	  if (COVISI(CONFIG_CASE_REPRINT)) 	Item_Config_Reprint(COSCROL(CONFIG_CASE_REPRINT));   	  	// repeat print
	  #endif

	  #if ENABLED(OPTION_BED_COATING) 
	  if (COVISI(CONFIG_CASE_COATING)) 	Item_Config_bedcoating(COSCROL(CONFIG_CASE_COATING));   	  	// GLASS LEVELING
	  #endif

	  #if ENABLED(ABL_GRID) 
		
	  if (COVISI(CONFIG_CASE_LEVELING)) Item_Config_Leveling(COSCROL(CONFIG_CASE_LEVELING));   	  			// auto LEVELING
	  if (COVISI(CONFIG_CASE_ACTIVELEVEL)) Item_Config_ActiveLevel(COSCROL(CONFIG_CASE_ACTIVELEVEL));  // auto LEVELING
	  #endif
	  
	  #if HAS_COLOR_LEDS 
	  if (COVISI(CONFIG_CASE_RGB)) 	Item_Config_RGB(COSCROL(CONFIG_CASE_RGB));   	  			// rgb
	  #endif

	  #if ENABLED(DEBUG_GCODE_M92)
	  if (COVISI(CONFIG_CASE_M92)) 	Item_Config_M92(COSCROL(CONFIG_CASE_M92));   	  			// M92
	  #endif
  }
  if (select_config.now) Draw_Menu_Cursor(COSCROL(select_config.now));
}

//
// Draw reprint Windows
//
#if ENABLED(OPTION_REPEAT_PRINTING)
inline void Draw_Reprint_Menu() {
  Clear_Main_Window();

  const int16_t roscroll = MROWS - index_reprint; // Scrolled-up lines
  #define ROSCROL(L) (roscroll + (L))
  #define ROVISI(L)  WITHIN(ROSCROL(L), 0, MROWS)

  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Blue, 14, 7, F("Re-print"));
  Draw_Back_First(select_reprint.now == 0);

  if (ROVISI(0)) Draw_Back_First(select_reprint.now == 0);                         				// < Back  
  if (ROVISI(REPRINT_CASE_ENABLED)) Item_Reprint_Enabled(ROSCROL(REPRINT_CASE_ENABLED));     	// ENABLED
  if (ROVISI(REPRINT_CASE_TIMES)) Item_Reprint_Times(ROSCROL(REPRINT_CASE_TIMES));    			// repeat print times
  if (ROVISI(REPRINT_CASE_LENGHT)) Item_Reprint_Lenght(ROSCROL(REPRINT_CASE_LENGHT));  		 	// forward move lenght
  if (ROVISI(REPRINT_CASE_RESET)) Item_Reprint_Reset(ROSCROL(REPRINT_CASE_RESET));  		 	// reset
  if (ROVISI(REPRINT_CASE_FORWARD)) Item_Reprint_Forward(ROSCROL(REPRINT_CASE_FORWARD));  		// FORWARD
  if (ROVISI(REPRINT_CASE_BACK)) Item_Reprint_Back(ROSCROL(REPRINT_CASE_BACK));  		 	    // BACK
  if (select_reprint.now) Draw_Menu_Cursor(ROSCROL(select_reprint.now));
}
#endif

//
// Draw Retract Windows
//
#if ENABLED(FWRETRACT) 
inline void Draw_Retract_Menu() {
  Clear_Main_Window();
 
  //DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Config, Menu_Coordinate,14, 7);
  //DWIN_JPG_CacheToN(1,HMI_flag.language+1);

  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Blue, 14, 7, F("Retract"));

  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(1), F("Auto-Retract:"));
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(2), F("Retract MM:"));
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(3), F("Retract V:"));
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(4), F("UnRetract MM:"));
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(5), F("UnRetract V:"));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Mix, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_MANUAL));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Gradient, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_AUTO));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Random, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_RANDOM));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Current, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_VTOOL));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Vtool, Menu_Coordinate,LBLX+Current_X_Coordinate[HMI_flag.language], MBASE(MIXER_CASE_VTOOL));

  Draw_Back_First(select_retract.now == 0);
  if (select_retract.now) Draw_Menu_Cursor(select_retract.now);

  Draw_Menu_Line(1,ICON_Cursor);
  if(fwretract.autoretract_enabled) DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(1), F("ON"));
  else DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(1), F("OFF"));

  Draw_Menu_Line(2,ICON_Cursor);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 2, 216, MBASE(2), fwretract.settings.retract_length*100);
  
  Draw_Menu_Line(3,ICON_Cursor);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 2, 216, MBASE(3), fwretract.settings.retract_feedrate_mm_s*100);
  
  Draw_Menu_Line(4,ICON_Cursor);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 2, 216, MBASE(4), fwretract.settings.retract_recover_extra*100);
  
  Draw_Menu_Line(5,ICON_Cursor);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 2, 216, MBASE(5), fwretract.settings.retract_recover_feedrate_mm_s*100);
  //uint8_t i,j=ICON_Mixer_Manual;
  //for(i=MIXER_CASE_MANUAL; i<MIXER_CASE_VTOOL; i++,j++) {
  	//Draw_Menu_Line(i,j);
	//Draw_More_Icon(i);
  //}
  
  //Draw_Menu_Line(i++,ICON_S_VTOOL);
  //DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i-1), mixer.selected_vtool);
}
#endif

//
// Draw Mixer Windows
//
inline void Draw_Mixer_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Mixer], 14, 7);
 
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Mixer, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Mix, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_MANUAL));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Gradient, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_AUTO));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Random, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_RANDOM));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Current, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_VTOOL));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Vtool, Menu_Coordinate,LBLX+Current_X_Coordinate[HMI_flag.language], MBASE(MIXER_CASE_VTOOL));

  Draw_Back_First(select_mixer.now == 0);
  if (select_mixer.now) Draw_Menu_Cursor(select_mixer.now);

  uint8_t i,j=ICON_Mixer_Manual;
  for(i=MIXER_CASE_MANUAL; i<MIXER_CASE_VTOOL; i++,j++) {
  	Draw_Menu_Line(i,j);
	Draw_More_Icon(i);
  }
  
  //Gradient
  if(mixer.gradient.enabled)
	DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 195, MBASE(MIXER_CASE_AUTO), F("ON"));
  else
  	DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 185, MBASE(MIXER_CASE_AUTO), F("OFF"));
  
  if(mixer.random_mix.enabled)
	DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 195, MBASE(MIXER_CASE_RANDOM), F("ON"));
  else
  	DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 185, MBASE(MIXER_CASE_RANDOM), F("OFF"));
  
  Draw_Menu_Line(i++,ICON_S_VTOOL);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i-1), mixer.selected_vtool);
}

//
// Draw Mixer_Manual Windows
//
inline void Draw_Mixer_Manual_Menu() {
  Clear_Main_Window();

  #if MANUAL_CASE_TOTAL >= 6
    const int16_t mscroll = MROWS - index_manual; // Scrolled-up lines
    #define MCSCROL(L) (mscroll + (L))
  #else
    #define MCSCROL(L) (L)
  #endif
  #define MCLINE(L)  MBASE(MCSCROL(L))
  #define MCVISI(L)  WITHIN(MCSCROL(L), 0, MROWS)

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_MIX], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_MIX, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  if (MCVISI(0)) Draw_Back_First(select_manual.now == 0);                         // < Back

  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_1, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER1));
  #if ENABLED (MIXING_EXTRUDER)
  		#if(MIXING_STEPPERS == 4)
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER2));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_2, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER2));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER3));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_3, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER3));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER4));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_4, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER4));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Comit, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_OK));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_vtool, Menu_Coordinate,LBLX+Comit_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_OK));
			DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 176, MCLINE(MANUAL_CASE_OK), F("---->"));
		#elif(MIXING_STEPPERS == 3)
    		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER2));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_2, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER2));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER3));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_3, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER3));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Comit, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_OK));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_vtool, Menu_Coordinate,LBLX+Comit_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_OK));
			DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 176, MCLINE(MANUAL_CASE_OK), F("---->"));
		#elif(MIXING_STEPPERS == 2)
    		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER2));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_2, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER2));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Comit, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_OK));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_vtool, Menu_Coordinate,LBLX+Comit_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_OK));
			DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 176, MCLINE(MANUAL_CASE_OK), F("---->"));
		#endif
  #endif
  
  Draw_Back_First(select_manual.now == 0);
  if (select_manual.now) Draw_Menu_Cursor(select_manual.now);

  uint8_t i,j=ICON_Extruder1;
  for(i=MANUAL_CASE_EXTRUDER1; i<MANUAL_CASE_TOTAL; i++,j++) {
  	Draw_Menu_Line(i,j);
  	//DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), MixerCfg.Manual_Percent[mixer.selected_vtool][i-1]);
	DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), mixer.mix[i-1]);
  }
  Draw_Menu_Line(i++,ICON_C_VTOOL);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i-1), mixer.selected_vtool);
}

//
// Draw Mixer_Auto Windows
//
inline void Draw_Mixer_Auto_Menu() {
  Clear_Main_Window();

  #if AUTO_CASE_TOTAL >= 6
    const int16_t ascroll = MROWS - index_auto; // Scrolled-up lines
    #define ACSCROL(L) (ascroll + (L))
  #else
    #define ACSCROL(L) (L)
  #endif
  #define ACLINE(L)  MBASE(ACSCROL(L))
  #define ACVISI(L)  WITHIN(ACSCROL(L), 0, MROWS)

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_GRADIENT], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_GRADIENT, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  if (ACVISI(0)) Draw_Back_First(select_auto.now == 0);                         // < Back

  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Start, Menu_Coordinate,LBLX, ACLINE(AUTO_CASE_ZPOS_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Z, Menu_Coordinate,LBLX+Start_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_ZPOS_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_End, Menu_Coordinate,LBLX, ACLINE(AUTO_CASE_ZPOS_END));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Z, Menu_Coordinate,LBLX+End_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_ZPOS_END));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Start, Menu_Coordinate,LBLX, ACLINE(AUTO_CASE_VTOOL_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Vtool, Menu_Coordinate,LBLX+Start_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_VTOOL_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_End, Menu_Coordinate,LBLX, ACLINE(AUTO_CASE_VTOOL_END));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Vtool, Menu_Coordinate,LBLX+End_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_VTOOL_END));
  
  Draw_Back_First(select_auto.now == 0);
  if (select_auto.now) Draw_Menu_Cursor(select_auto.now);

  Draw_Menu_Line(AUTO_CASE_ZPOS_START,ICON_Zpos_Rise);
  HMI_ValueStruct.Auto_Zstart_scale = mixer.gradient.start_z*MINUNITMULT;
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(1), HMI_ValueStruct.Auto_Zstart_scale);

  Draw_Menu_Line(AUTO_CASE_ZPOS_END,ICON_Zpos_Drop);
  HMI_ValueStruct.Auto_Zend_scale = mixer.gradient.end_z*MINUNITMULT;
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(2), HMI_ValueStruct.Auto_Zend_scale);

  Draw_Menu_Line(AUTO_CASE_VTOOL_START,ICON_VTool_Rise);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(3), mixer.gradient.start_vtool);

  Draw_Menu_Line(AUTO_CASE_VTOOL_END,ICON_VTool_Drop);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(4), mixer.gradient.end_vtool);
  /*
  uint8_t i,j=ICON_Extruder1;
  for(i=AUTO_CASE_VTOOL_START; i<AUTO_CASE_TOTAL; i++,j++) {
  	Draw_Menu_Line(i,j);
  	DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), MixerCfg.Auto_Percent[mixer.selected_vtool][i-1]);
  }
  */
  //Draw_Menu_Line(i++,ICON_S_VTOOL);
  //DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i-1), mixer.selected_vtool);
}


//
// Draw Mixer_Random Windows
//
inline void Draw_Mixer_Random_Menu() {
  Clear_Main_Window();

  #if RANDOM_CASE_TOTAL >= 6
    const int16_t rscroll = MROWS - index_random; // Scrolled-up lines
    #define RCSCROL(L) (rscroll + (L))
  #else
    #define RCSCROL(L) (L)
  #endif
  #define RCLINE(L)  MBASE(RCSCROL(L))
  #define RCVISI(L)  WITHIN(RCSCROL(L), 0, MROWS)

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_RANDOM], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_RANDOM, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  if (RCVISI(0)) Draw_Back_First(select_random.now == 0); // < Back

  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Random_Menu_Start, Menu_Coordinate,LBLX, RCLINE(RANDOM_CASE_ZPOS_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Random_Menu_Z, Menu_Coordinate,LBLX+Start_X_Coordinate[HMI_flag.language], RCLINE(RANDOM_CASE_ZPOS_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Random_Menu_End, Menu_Coordinate,LBLX, RCLINE(RANDOM_CASE_ZPOS_END));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Random_Menu_Z, Menu_Coordinate,LBLX+End_X_Coordinate[HMI_flag.language], RCLINE(RANDOM_CASE_ZPOS_END));

  Draw_Back_First(select_random.now == 0);
  if (select_random.now) Draw_Menu_Cursor(select_random.now);

  Draw_Menu_Line(RANDOM_CASE_ZPOS_START,ICON_Zpos_Rise);
  HMI_ValueStruct.Random_Zstart_scale = mixer.random_mix.start_z*MINUNITMULT;
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
  Draw_Menu_Line(RANDOM_CASE_ZPOS_END,ICON_Zpos_Drop);
  HMI_ValueStruct.Random_Zend_scale = mixer.random_mix.end_z*MINUNITMULT;
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);

  //Height
  Draw_Menu_Line(RANDOM_CASE_HEIGHT,ICON_Cursor);
  HMI_ValueStruct.Random_Height = mixer.random_mix.height*MINUNITMULT;
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(RANDOM_CASE_HEIGHT), F("Height:"));
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(RANDOM_CASE_HEIGHT), HMI_ValueStruct.Random_Height);
  
  //Extruders
  Draw_Menu_Line(RANDOM_CASE_EXTRUDERS,ICON_Cursor);
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(RANDOM_CASE_EXTRUDERS), F("Extruders:"));
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(RANDOM_CASE_EXTRUDERS), mixer.random_mix.extruders);
}


//
// Draw Popup Windows
//
#if HAS_HOTEND || HAS_HEATED_BED
void DWIN_Popup_Temperature(const char *msg) {
  Clear_Popup_Area();
  Draw_Popup_Bkgd_105();
	DWIN_ICON_Show(ICON, ICON_TempTooHigh, 102, 165);
	DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - strlen(msg)*8)/2, 300, (char*)msg);  
	#if 1
	for(uint8_t i=0; i<20; i++){
		buzzer.tone(200, 2000);
		buzzer.tone(200, 0);
	}
	#endif
}
#endif

inline void Draw_Popup_Bkgd_60() {
  DWIN_Draw_Rectangle(1, Color_Bg_Window, 14, 60, 258, 330);
}

inline void Draw_Popup_Bkgd_Wifi() {
  DWIN_Draw_Rectangle(1, Color_Bg_Blue, 14, 60, 258, 90);
}

inline void Draw_Popup_Bkgd_Reprint() {
  DWIN_Draw_Rectangle(1, Color_Bg_Blue, 14, 60, 258, 90);
}


#if ENABLED(OPTION_REPEAT_PRINTING)
void Popup_Window_BTempCooldown() {
	Clear_Main_Window();
  Draw_Popup_Bkgd_60();
	Draw_Popup_Bkgd_Reprint();
	Draw_Reprint_Title("Remain Times:");
	DWIN_Draw_Rectangle(1, Color_Bg_Blue, 144, 65, 258, 85);
	DWIN_Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Blue, 4, 144, 65, ReprintManager.Reprint_times);
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Blue, 14, 7, F("Re-print"));
	DWIN_Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, 61, 200, F("Wait for Hotbed"));
	DWIN_Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, 86, 235, F("cool down!"));
	DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 280);
}

void Popup_Window_FMoveStart() {
	Clear_Main_Window();
    Draw_Popup_Bkgd_60();
	Draw_Popup_Bkgd_Reprint();
	Draw_Reprint_Title("Remain Times:");
	DWIN_Draw_Rectangle(1, Color_Bg_Blue, 144, 65, 258, 85);
	DWIN_Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Blue, 4, 144, 65, ReprintManager.Reprint_times);
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Blue, 14, 7, F("Re-print"));
	DWIN_Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, 41, 200, F("Forward Move Start!"));
	DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 280);
}

void Popup_Window_BMoveStart() {
	Clear_Main_Window();
    Draw_Popup_Bkgd_60();
	Draw_Popup_Bkgd_Reprint();
	Draw_Reprint_Title("Remain Times:");
	DWIN_Draw_Rectangle(1, Color_Bg_Blue, 144, 65, 258, 85);
	DWIN_Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Blue, 4, 144, 65, ReprintManager.Reprint_times);
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Blue, 14, 7, F("Re-print"));
	DWIN_Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, 56, 200, F("Back Move Start!"));
	DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 280);
}

void Popup_Window_BMoveStop() {
	Clear_Main_Window();
  Draw_Popup_Bkgd_60();
	Draw_Popup_Bkgd_Reprint();
	Draw_Reprint_Title("Remain Times:");
	DWIN_Draw_Rectangle(1, Color_Bg_Blue, 144, 65, 258, 85);
	DWIN_Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Blue, 4, 144, 65, ReprintManager.Reprint_times);
  DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Blue, 14, 7, F("Re-print"));
	DWIN_Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, 61, 200, F("Back Move Stop!"));
	DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 280);
}
#endif

#if HAS_HOTEND
void Popup_Window_ETempTooLow() {
    Clear_Main_Window();
    Draw_Popup_Bkgd_60();
    DWIN_ICON_Show(ICON, ICON_TempTooLow, 102, 105);
	
		DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, 64, 235, F("Nozzle is too cold"));
    DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 280);
}
#endif

void Popup_Window_Resume() {
  Clear_Popup_Area();
  Draw_Popup_Bkgd_105();
  
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 14) / 2, 115, F("Continue Print"));
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 22) / 2, 192, F("It looks like the last"));
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 22) / 2, 212, F("file was interrupted."));
  DWIN_ICON_Show(ICON, ICON_Cancel_E,    26, 307);
  DWIN_ICON_Show(ICON, ICON_Continue_E, 146, 307);
}

void Popup_Window_HomeAll(const bool parking=false) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing XYZ"));
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
}

void Popup_Window_HomeX(const bool parking=false) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing X"));
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
}

void Popup_Window_HomeY(const bool parking=false) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing Y"));
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
}

void Popup_Window_HomeZ(const bool parking=false) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing Z"));
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
}

#ifdef LCD_BED_LEVELING
void Popup_Window_Leveling0() {
    Clear_Main_Window();
    Draw_Popup_Bkgd_60();
    
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 13) / 2, 230, F("Bed Leveling"));
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
		DWIN_ICON_Show(ICON, ICON_AutoLeveling, 101, 105);
}

#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
void Popup_Window_CatchOffset() {
    Clear_Main_Window();
    Draw_Popup_Bkgd_60();
    
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 230, F("Catching Probe Z Offset"));
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
		DWIN_ICON_Show(ICON, ICON_AutoLeveling, 101, 105);
}
#endif

void Popup_Remove_Glass() {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  
  DWIN_Draw_String(false, true, font8x16, Color_Bg_Red, Color_Bg_Window, (272 - 8 * 13) / 2, 180, F("!!Attention!!"));
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 212, F("Please remove the glass"));
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 15) / 2, 232, F("before catching"));
	DWIN_ICON_Show(ICON, ICON_AutoLeveling, 101, 105);
  DWIN_ICON_Show(ICON, ICON_Confirm_E, 96, 280);
}
#endif


#if HAS_ONESTEP_LEVELING
  void Popup_Window_Leveling() {
    Clear_Main_Window();
    Draw_Popup_Bkgd_60();
    DWIN_ICON_Show(ICON, ICON_AutoLeveling, 101, 105);
    if (HMI_IsChinese()) {
      DWIN_Frame_AreaCopy(1, 0, 371, 100, 386, 84, 240);
      DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
    }
    else {
      DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 13) / 2, 230, F("Bed Leveling"));
      DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
    }
  }
#endif


void Draw_Select_Highlight(const bool sel) {
  HMI_flag.select_flag = sel;
  const uint16_t c1 = sel ? Select_Color : Color_Bg_Window,
                 c2 = sel ? Color_Bg_Window : Select_Color;
  DWIN_Draw_Rectangle(0, c1, 25, 279, 126, 318);
  DWIN_Draw_Rectangle(0, c1, 24, 278, 127, 319);
  DWIN_Draw_Rectangle(0, c2, 145, 279, 246, 318);
  DWIN_Draw_Rectangle(0, c2, 144, 278, 247, 319);
}

void Popup_window_PauseOrStop() {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  
  if (select_print.now == 1) DWIN_Draw_String(false, true, font12x24, Popup_Text_Color, Color_Bg_Window, (272 - 12 * 12) / 2, 150, F("Pause Print?"));
  else if (select_print.now == 2) DWIN_Draw_String(false, true, font12x24, Popup_Text_Color, Color_Bg_Window, (272 - 12 * 11) / 2, 150, F("Stop Print?"));
  DWIN_ICON_Show(ICON, ICON_Confirm_E, 26, 280);
  DWIN_ICON_Show(ICON, ICON_Cancel_E, 146, 280);
  Draw_Select_Highlight(true);
}

void Popup_window_Wifi_Connect() {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  Draw_Title(F("WIFI"));
  //DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 10) / 2, 211, F("WiFi Connect..."));
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 10) / 2 - 24, 240, F("Connection"));
}

void Popup_window_Wifi_Disconnect() {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  Draw_Title(F("WIFI"));
  
  //DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 14) / 2, 211, F("WiFi On Failed"));
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 11) / 2, 240, F("No connect!"));
}

void Popup_window_Powerdown() {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  Draw_Title(GET_TEXT_F(MSG_DWIN_POWERDOWN_TITLE));
  DWIN_ICON_Show(ICON, ICON_POWER_DOWN, 86, 95);
  DWIN_ICON_Show(ICON, ICON_NO_0, 26, 228);
  DWIN_ICON_Show(ICON, ICON_YES_1, 146, 228);
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 17) / 2, 290, GET_TEXT_F(MSG_DWIN_POWERDOWN));
}

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
void DRAW_Filament_Runout_Mode(char mode){
	switch (mode){
		case DWIN_PAUSE_MODE_PAUSE_PRINT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_PAUSE));
			break;
		case DWIN_PAUSE_MODE_CHANGE_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_CHANGE));
			break;
		case DWIN_PAUSE_MODE_LOAD_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_LOAD));
			break;
		case DWIN_PAUSE_MODE_UNLOAD_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_UNLOAD));
			break;
		case DWIN_PAUSE_MODE_INSERT_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_INSERT));
			break;
	    case DWIN_PAUSE_MODE_PURGE_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_PURGE));
			break;
	    case DWIN_PAUSE_MODE_OPTION_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_OPTION));
			break;
		case DWIN_PAUSE_MODE_RESUME_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_RESUME));
			break;
		default:break;
	}
}

void Popup_window_Filament_Runout_Start(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_FIL_OPTION, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 15) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_START1));
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 9) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_START2));
}

void Popup_window_Filament_Runout_Heating(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_FIL_OPTION, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_HEATING1));
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 17) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_HEATING2));
}

void Popup_window_Filament_Runout_Insert(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DRAW_Filament_Runout_Mode(mode);
  DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 168);
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 15) / 2, 211, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_INSERT1));
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 16) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_INSERT2));
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 12) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_INSERT3));
}

void Popup_window_Filament_Runout_Unload(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_FIL_OPTION, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_UNLOAD1));
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 16) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_UNLOAD2));
}

void Popup_window_Filament_Runout_Load(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_FIL_OPTION, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_LOAD1));
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 14) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_LOAD2));
}

void Popup_window_Filament_Runout_Purge(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_FIL_OPTION, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_PURGE1));
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 15) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_PURGE2));
}

void Popup_window_Filament_Runout_Option(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DRAW_Filament_Runout_Mode(mode);
  DWIN_ICON_Show(ICON, ICON_YES_0, 26, 168);
  DWIN_ICON_Show(ICON, ICON_NO_1, 146, 168);
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 18) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_OPTION));
}

void Popup_window_Filament_Runout_Resume(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_Leveling_0, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 17) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_RESUME));
}
#endif


void Draw_Printing_Screen() {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Tune, Menu_Coordinate,Tune_X_Coordinate[HMI_flag.language], 325);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Pause, Menu_Coordinate,Pause_X_Coordinate[HMI_flag.language], 325);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Stop, Menu_Coordinate,Stop_X_Coordinate[HMI_flag.language], 325);
}

void Draw_Print_ProgressBar() {
  DWIN_ICON_Show(ICON, ICON_Bar, 15, 63);
  DWIN_Draw_Rectangle(1, BarFill_Color, 15 + Percentrecord * 240 / 100, 63, 256, 83);
  DWIN_Draw_IntValue(false, true, 0, font8x16, Color_Bg_Red, BarFill_Color, 2, 117, 65, Percentrecord);
  DWIN_Draw_String(false, false, font8x16, Color_Bg_Red, BarFill_Color, 133, 65, F("%"));
}

void Draw_Print_ProgressElapsed() {
  duration_t elapsed = print_job_timer.duration(); // print timer
  DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 2, 42, 212, elapsed.value / 3600);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 58, 212, F(":"));
  DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 2, 66, 212, (elapsed.value % 3600) / 60);
}

#if HAS_SUICIDE
void Draw_Powerdown_Machine() {		
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 115, 325, 157, 353);
  if(HMI_flag.putdown_close_timer_rg >= 100)
  	DWIN_Draw_IntValue(true, true, 1, font14x28, Color_White, Color_Bg_Black, 3, 115, 325, HMI_flag.putdown_close_timer_rg);// Remaining time
  else if((HMI_flag.putdown_close_timer_rg < 100)&&(HMI_flag.putdown_close_timer_rg >= 10))
  	DWIN_Draw_IntValue(true, true, 1, font14x28, Color_White, Color_Bg_Black, 2, 122, 325, HMI_flag.putdown_close_timer_rg);// Remaining time
  else
  	DWIN_Draw_IntValue(true, true, 1, font14x28, Color_White, Color_Bg_Black, 1, 129, 325, HMI_flag.putdown_close_timer_rg);// Remaining time
}

void Draw_Freedown_Machine() {		
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 230, 7, 256, 23);
  if(HMI_flag.free_close_timer_rg >= 100)
  	DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 3, 230, 7, HMI_flag.free_close_timer_rg);// Remaining time
  else if((HMI_flag.free_close_timer_rg < 100)&&(HMI_flag.free_close_timer_rg >= 10))
  	DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 2, 234, 7, HMI_flag.free_close_timer_rg);// Remaining time
  else
  	DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 1, 238, 7, HMI_flag.free_close_timer_rg);// Remaining time
}
#endif

void Draw_Print_ProgressRemain() {
  DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 2, 176, 212, remain_time / 3600);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 192, 212, F(":"));
  DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 2, 200, 212, (remain_time % 3600) / 60);
}

void Draw_Print_ProgressZvaule() {
  DWIN_Draw_Signed_Float(DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_Zoffset_inum, State_text_Zoffset_fnum, State_text_Zoffset_X, State_text_Zoffset_Y, 100*current_position.z);
}

void Draw_Print_ProgressExtruder() {
  #if(MIXING_STEPPERS == 4)
  	DWIN_ICON_Show(ICON, ICON_Extruder1_P, 10, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder2_P, 62, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder3_P, 114, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder4_P, 166, 98);
  	DWIN_ICON_Show(ICON, ICON_VTool_P, 	218, 98);
  #elif(MIXING_STEPPERS == 3)
  	DWIN_ICON_Show(ICON, ICON_Extruder1_P, 21, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder2_P, 84, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder3_P, 147, 98);
  	DWIN_ICON_Show(ICON, ICON_VTool_P, 	210, 98);
  #elif(MIXING_STEPPERS == 2)
  	DWIN_ICON_Show(ICON, ICON_Extruder1_P, 36, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder2_P, 114, 98);
  	DWIN_ICON_Show(ICON, ICON_VTool_P, 	192, 98);
  #else
    DWIN_ICON_Show(ICON, ICON_Extruder1_P, 63, 98);
    DWIN_ICON_Show(ICON, ICON_VTool_P, 	168, 98);
  #endif
}

void Draw_Print_ProgressModel(){
    DWIN_Draw_Rectangle(1, Color_Bg_Black, 10, 455, 250, 471);
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 10, 455, F("Model:"));
	/*
	if(MixerCfg.Mixer_Mode_Rg == 0) {
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 58, 455, F("Vtool"));
		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 2, 107, 455, mixer.selected_vtool);
	}
	if(MixerCfg.Mixer_Mode_Rg == 1) {
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 58, 455, F("Gradient"));
		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 2, 130, 455, mixer.gradient.start_vtool);
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 146, 455, F("--->"));
		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 2, 178, 455, mixer.gradient.end_vtool);
	}
	if(MixerCfg.Mixer_Mode_Rg == 2) {
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 58, 455, F("Random"));
	}
    */
	if(mixer.gradient.enabled) {
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 58, 455, F("Gradient"));
		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 2, 130, 455, mixer.gradient.start_vtool);
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 146, 455, F("--->"));
		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 2, 178, 455, mixer.gradient.end_vtool);
	}
	else if(mixer.random_mix.enabled) {
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 58, 455, F("Random"));
	}
	else{
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 58, 455, F("Vtool"));
		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 2, 107, 455, mixer.selected_vtool);
	}
}

void Goto_PrintProcess() {
  checkkey = PrintProcess;
	babyz_offset = 0.0;
	last_babyz_offset = 0.0;
	prevouis_babyz_offset = 0.0;
	HMI_ValueStruct.zoffset_value = 0;
  Clear_Main_Window();
  Clear_Bottom_Area();
  Draw_Printing_Screen();
 
  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Print], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Printing, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  ICON_Tune();
  if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
  ICON_Stop();
  
  // Copy into filebuf string before entry
  char * const name = card.longest_filename();
  const int8_t npos = _MAX(0U, DWIN_WIDTH - strlen(name) * MENU_CHR_W) / 2;
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, npos, 40, name);

  DWIN_ICON_Show(ICON, ICON_PrintTime, 17, 193);
  DWIN_ICON_Show(ICON, ICON_RemainTime, 150, 191);
  DWIN_ICON_Show(ICON, ICON_PRINT_TIME, 42, 193);
  DWIN_ICON_Show(ICON, ICON_REMAIN_TIME, 176, 193);
  
  Draw_Print_ProgressBar();
  Draw_Print_ProgressElapsed();
  Draw_Print_ProgressRemain();
  Draw_Print_ProgressZvaule();
  Draw_Print_ProgressExtruder();
  mixer.selected_vtool = MixerCfg.Vtool_Backup;
  updata_mixer_from_vtool();
  Refresh_Percent_display();
  Draw_Print_ProgressModel();
}

void Goto_MainMenu() {
  checkkey = MainMenu;

  //Clear_Main_Window();
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 0, DWIN_WIDTH, DWIN_HEIGHT);
 
  //DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Main, Menu_Coordinate,14, 7);
  //DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_ICON_Show(ICON, ICON_LOGO, Logo_offset_X, Logo_offset_Y);

  ICON_Print();
  ICON_Prepare();
  ICON_Control();
  TERN(HAS_ONESTEP_LEVELING, ICON_Leveling, ICON_StartInfo)(select_page.now == 3);
  Draw_Status_Area(true);
  mixer.selected_vtool = MixerCfg.Vtool_Backup;
  //Draw_Print_ProgressModel();
  #if ENABLED(OPTION_AUTOPOWEROFF)
  	if(HMI_flag.auto_shutdown) Draw_Freedown_Machine();
  #endif

	//Show WiFi ICON
  #if ENABLED(OPTION_WIFI_MODULE)
	//Clear_Bottom_Area();
	if(HMI_flag.wifi_Handshake_ok && WiFi_Enabled){
		DWIN_ICON_Show(ICON, ICON_WIFI, 0, 0);
		//DWIN_Draw_String(false, true, font10x20, Color_White, Color_Bg_Black, 12, 454, F("WiFi link printing..."));
	}
  #endif
}

inline ENCODER_DiffState get_encoder_state() {
  static millis_t Encoder_ms = 0;
  const millis_t ms = millis();
  if (PENDING(ms, Encoder_ms)) return ENCODER_DIFF_NO;
  const ENCODER_DiffState state = Encoder_ReceiveAnalyze();
  if (state != ENCODER_DIFF_NO) Encoder_ms = ms + ENCODER_WAIT;
  return state;
}

#if ENABLED(FWRETRACT) 
char Retract_Buf[50]={0};
void HMI_Retract_MM() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Retract_MM_scale)) {
      checkkey = Retract;
      EncoderRate.enabled = false;
	  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RETRACT_MM), HMI_ValueStruct.Retract_MM_scale);
	  ZERO(Retract_Buf);
	  sprintf_P(Retract_Buf,PSTR("M207 S%.2f"),(fwretract.settings.retract_length));
	  queue.inject_P(Retract_Buf);
	  settings.save();
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Retract_MM_scale, 0);
    NOMORE(HMI_ValueStruct.Retract_MM_scale, 100*100);
	fwretract.settings.retract_length = HMI_ValueStruct.Retract_MM_scale/100;
	DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RETRACT_MM), HMI_ValueStruct.Retract_MM_scale);
	DWIN_UpdateLCD();
  }
}

void HMI_Retract_V() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Retract_V_scale)) {
      checkkey = Retract;
      EncoderRate.enabled = false;
	  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RETRACT_V), HMI_ValueStruct.Retract_V_scale);
	  ZERO(Retract_Buf);
	  sprintf_P(Retract_Buf,PSTR("M207 F%.2f"),(MMS_TO_MMM(fwretract.settings.retract_feedrate_mm_s)));
	  queue.inject_P(Retract_Buf);
	  settings.save();
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Retract_V_scale, 0);
    NOMORE(HMI_ValueStruct.Retract_V_scale, 100*100);
	fwretract.settings.retract_feedrate_mm_s = HMI_ValueStruct.Retract_V_scale/100;
	DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RETRACT_V), HMI_ValueStruct.Retract_V_scale);
	DWIN_UpdateLCD();
  }
}

void HMI_UnRetract_MM() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.unRetract_MM_scale)) {
      checkkey = Retract;
      EncoderRate.enabled = false;
	  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RECOVER_MM), HMI_ValueStruct.unRetract_MM_scale);
	  ZERO(Retract_Buf);
	  sprintf_P(Retract_Buf,PSTR("M208 S%.2f"),(fwretract.settings.retract_recover_extra));
	  queue.inject_P(Retract_Buf);
	  settings.save();
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.unRetract_MM_scale, 0);
    NOMORE(HMI_ValueStruct.unRetract_MM_scale, 100*100);
	fwretract.settings.retract_recover_extra = HMI_ValueStruct.unRetract_MM_scale/100;
	DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RECOVER_MM), HMI_ValueStruct.unRetract_MM_scale);
	DWIN_UpdateLCD();
  }
}

void HMI_UnRetract_V() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.unRetract_V_scale)) {
      checkkey = Retract;
      EncoderRate.enabled = false;
		  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RECOVER_V), HMI_ValueStruct.unRetract_V_scale);
		  ZERO(Retract_Buf);
		  sprintf_P(Retract_Buf,PSTR("M208 F%.2f"),(MMS_TO_MMM(fwretract.settings.retract_recover_feedrate_mm_s)));
		  queue.inject_P(Retract_Buf);
		  settings.save();
      DWIN_UpdateLCD();
      return;
  }
  NOLESS(HMI_ValueStruct.unRetract_V_scale, 0);
  NOMORE(HMI_ValueStruct.unRetract_V_scale, 100*100);
	fwretract.settings.retract_recover_feedrate_mm_s = HMI_ValueStruct.unRetract_V_scale/100;
	DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RECOVER_V), HMI_ValueStruct.unRetract_V_scale);
	DWIN_UpdateLCD();
  }
}
#endif

void HMI_Move_X() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_X_scale)) {
      checkkey = AxisMove;
      EncoderRate.enabled = false;
		  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
		  if (!planner.is_full()) {
        // Wait for planner moves to finish!
        planner.synchronize();
        planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
      }
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Move_X_scale, (X_MIN_POS) * MINUNITMULT);
    NOMORE(HMI_ValueStruct.Move_X_scale, (X_MAX_POS) * MINUNITMULT);
    current_position.x = HMI_ValueStruct.Move_X_scale / 10;
		DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
		DWIN_UpdateLCD();
  }
}

void HMI_Move_Y() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_Y_scale)) {
      checkkey = AxisMove;
      EncoderRate.enabled = false;
		  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
		  if (!planner.is_full()) {
        // Wait for planner moves to finish!
        planner.synchronize();
        planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
      }
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Move_Y_scale, (Y_MIN_POS) * MINUNITMULT);
    NOMORE(HMI_ValueStruct.Move_Y_scale, (Y_MAX_POS) * MINUNITMULT);
    current_position.y = HMI_ValueStruct.Move_Y_scale / 10;
		DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
		DWIN_UpdateLCD();
  }
}

void HMI_Move_Z() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_Z_scale)) {
	    checkkey = AxisMove;
	    EncoderRate.enabled = false;
		  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
		  if (!planner.is_full()) {
        // Wait for planner moves to finish!
        planner.synchronize();
        planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
      }
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Move_Z_scale, Z_MIN_POS * MINUNITMULT);
    NOMORE(HMI_ValueStruct.Move_Z_scale, Z_MAX_POS * MINUNITMULT);
    current_position.z = HMI_ValueStruct.Move_Z_scale / 10;
		DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
		DWIN_UpdateLCD();
  }
}

void HMI_Adjust_Ext_Percent(uint8_t Extruder_Number) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1])) {
      checkkey = Mix_Manual;
      EncoderRate.enabled = false;
	  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(Extruder_Number), MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1]);
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1], 0);
    NOMORE(MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1], 100);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(Extruder_Number), MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1]);
	DWIN_UpdateLCD();
  }
}

/*
void HMI_Adjust_Ext1_Percent() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, MixerCfg.Manual_Percent[mixer.selected_vtool][0])) {
      checkkey = Mix_Manual;
      EncoderRate.enabled = false;
	  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(MixerCfg.Manual_Percent[mixer.selected_vtool][0], 0);
    NOMORE(MixerCfg.Manual_Percent[mixer.selected_vtool][0], 100);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
	DWIN_UpdateLCD();
  }
}
*/

void HMI_Adjust_Auto_Zpos_Start() {
	static float last_Z_scale = 0;
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();

	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Auto_Zstart_scale)) {
			checkkey = Mix_Auto;
			EncoderRate.enabled = false;
			last_Z_scale = HMI_ValueStruct.Auto_Zstart_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_START), HMI_ValueStruct.Auto_Zstart_scale);
			DWIN_UpdateLCD();
			return;
	   }
		
	   if ((HMI_ValueStruct.Auto_Zstart_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Auto_Zstart_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
	   else if ((last_Z_scale - HMI_ValueStruct.Auto_Zstart_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Auto_Zstart_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
	     NOLESS(HMI_ValueStruct.Auto_Zstart_scale, 0);		 
		 DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_START), HMI_ValueStruct.Auto_Zstart_scale);
		 mixer.gradient.start_z = HMI_ValueStruct.Auto_Zstart_scale / 10;
		 mixer.refresh_gradient();
		 DWIN_UpdateLCD();
	}
}

void HMI_Adjust_Auto_Zpos_End() {
	static float last_Z_scale = 0;
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	
		if (encoder_diffState != ENCODER_DIFF_NO) {
			if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Auto_Zend_scale)) {
				checkkey = Mix_Auto;
				EncoderRate.enabled = false;
				last_Z_scale = HMI_ValueStruct.Auto_Zend_scale;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_END), HMI_ValueStruct.Auto_Zend_scale);
				DWIN_UpdateLCD();
				return;
		   }
			
		   if ((HMI_ValueStruct.Auto_Zend_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
				HMI_ValueStruct.Auto_Zend_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
		   else if ((last_Z_scale - HMI_ValueStruct.Auto_Zend_scale) > (Z_MAX_POS) * MINUNITMULT)
				HMI_ValueStruct.Auto_Zend_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
		     NOLESS(HMI_ValueStruct.Auto_Zend_scale, 0);			 
			 DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_END), HMI_ValueStruct.Auto_Zend_scale);
			 mixer.gradient.end_z = HMI_ValueStruct.Auto_Zend_scale / 10;
			 mixer.refresh_gradient();
			 DWIN_UpdateLCD();
		}
}

void HMI_Adjust_Auto_VTool_Start() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, mixer.gradient.start_vtool)) {
      checkkey = Mix_Auto;
      EncoderRate.enabled = false;
	  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_START), mixer.gradient.start_vtool);
	  mixer.refresh_gradient();
	  DWIN_UpdateLCD();
      return;
    }
    NOLESS(mixer.gradient.start_vtool, 0);
    NOMORE(mixer.gradient.start_vtool, MIXING_VIRTUAL_TOOLS-1);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_START), mixer.gradient.start_vtool);
	DWIN_UpdateLCD();
  }
}

void HMI_Adjust_Auto_VTool_End() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, mixer.gradient.end_vtool)) {
      checkkey = Mix_Auto;
      EncoderRate.enabled = false;
	  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_END), mixer.gradient.end_vtool);
	  mixer.refresh_gradient();
	  DWIN_UpdateLCD();
      return;
    }
    NOLESS(mixer.gradient.end_vtool, 0);
    NOMORE(mixer.gradient.end_vtool, MIXING_VIRTUAL_TOOLS-1);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_END), mixer.gradient.end_vtool);
	DWIN_UpdateLCD();
  }
}

void HMI_Adjust_Random_Zpos_Start() {
	static float last_Z_scale = 0;
	
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Random_Zstart_scale)) {
			checkkey = Mix_Random;
			EncoderRate.enabled = false;
			last_Z_scale = HMI_ValueStruct.Random_Zstart_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
			DWIN_UpdateLCD();
			return;
	   }		
	   if ((HMI_ValueStruct.Random_Zstart_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Random_Zstart_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
	   else if ((last_Z_scale - HMI_ValueStruct.Random_Zstart_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Random_Zstart_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
	     NOLESS(HMI_ValueStruct.Random_Zstart_scale, 0);
		 mixer.random_mix.start_z = HMI_ValueStruct.Random_Zstart_scale / 10;
		 mixer.refresh_random_mix();
		 DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
		 DWIN_UpdateLCD();
	}
}

void HMI_Adjust_Random_Zpos_End() {
	static float last_Z_scale = 0;
	
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if (encoder_diffState != ENCODER_DIFF_NO) {
			if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Random_Zend_scale)) {
				checkkey = Mix_Random;
				EncoderRate.enabled = false;
				last_Z_scale = HMI_ValueStruct.Random_Zend_scale;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);
				DWIN_UpdateLCD();
				return;
		   }
			
		   if ((HMI_ValueStruct.Random_Zend_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
				HMI_ValueStruct.Random_Zend_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
		   else if ((last_Z_scale - HMI_ValueStruct.Random_Zend_scale) > (Z_MAX_POS) * MINUNITMULT)
				HMI_ValueStruct.Random_Zend_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
		     NOLESS(HMI_ValueStruct.Random_Zend_scale, 0);
			 mixer.random_mix.end_z = HMI_ValueStruct.Random_Zend_scale / 10;
			 mixer.refresh_random_mix();
			 DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);
			 DWIN_UpdateLCD();
		}
}

void HMI_Adjust_Random_Height() {
	static float last_Height = 0;
	
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if (encoder_diffState != ENCODER_DIFF_NO) {
			if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Random_Height)) {
				checkkey = Mix_Random;
				EncoderRate.enabled = false;
				last_Height = HMI_ValueStruct.Random_Height;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_HEIGHT), HMI_ValueStruct.Random_Height);
				DWIN_UpdateLCD();
				return;
		   }
			
		   if ((HMI_ValueStruct.Random_Height - last_Height) > (100) * MINUNITMULT)
				HMI_ValueStruct.Random_Height = last_Height + (100) * MINUNITMULT;
		   else if ((last_Height - HMI_ValueStruct.Random_Height) > (Z_MAX_POS) * MINUNITMULT)
				HMI_ValueStruct.Random_Height = last_Height - (Z_MAX_POS) * MINUNITMULT;
		     NOLESS(HMI_ValueStruct.Random_Height, 0);
			 mixer.random_mix.height = HMI_ValueStruct.Random_Height / 10;
			 DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_HEIGHT), HMI_ValueStruct.Random_Height);
			 DWIN_UpdateLCD();
		}
}


void HMI_Adjust_Random_Extruders() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, mixer.random_mix.extruders)) {
      checkkey = Mix_Random;
      EncoderRate.enabled = false;
	  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + RANDOM_CASE_EXTRUDERS), mixer.random_mix.extruders);
      #if ENABLED(POWER_LOSS_RECOVERY)
	  recovery.save(true);
	  #endif
	  DWIN_UpdateLCD();
      return;
    }
    NOLESS(mixer.random_mix.extruders, 0);
    NOMORE(mixer.random_mix.extruders, MIXING_STEPPERS);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + RANDOM_CASE_EXTRUDERS), mixer.random_mix.extruders);
	DWIN_UpdateLCD();
  }
}

#if ENABLED(OPTION_BED_COATING)
void HMI_Adjust_Coating_Thickness() {
	static float last_G = 0;
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();  
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.coating_thickness)) {
			checkkey = Config;
			EncoderRate.enabled = false;
			last_G = HMI_ValueStruct.coating_thickness;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 2, 216, MBASE(MROWS -index_config + CONFIG_CASE_COATING), HMI_ValueStruct.coating_thickness);			
			DWIN_UpdateLCD();
			coating_thickness = (float)HMI_ValueStruct.coating_thickness/100;
	    settings.save();
			return;
	  }		
		if ((HMI_ValueStruct.coating_thickness - last_G) > 1000)
			HMI_ValueStruct.coating_thickness = last_G + 1000;
		else if ((last_G - HMI_ValueStruct.coating_thickness) > 1000)
			HMI_ValueStruct.coating_thickness = last_G - 1000;
		NOLESS(HMI_ValueStruct.coating_thickness, -100);
		NOMORE(HMI_ValueStruct.coating_thickness, 1000);		
		DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 2, 2, 216, MBASE(MROWS -index_config + CONFIG_CASE_COATING), HMI_ValueStruct.coating_thickness);
		DWIN_UpdateLCD();
		
	}
}

#endif

#if ENABLED(OPTION_REPEAT_PRINTING)
char Reprint_Buf[50] = {0}; 
void HMI_Reprint_Times() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, ReprintManager.Reprint_times)) {
      checkkey = Re_print;
      EncoderRate.enabled = false;
		  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 216, MBASE(MROWS -index_reprint + REPRINT_CASE_TIMES), ReprintManager.Reprint_times);
		  ZERO(Reprint_Buf);
		  sprintf_P(Reprint_Buf,PSTR("M180 T%4d"),ReprintManager.Reprint_times);
		  queue.inject_P(Reprint_Buf);
		  DWIN_UpdateLCD();
      return;
    }
    NOLESS(ReprintManager.Reprint_times, 0);
    NOMORE(ReprintManager.Reprint_times, REPEAT_PRINTING_MAX_TIMES);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 4, 216, MBASE(MROWS -index_reprint + REPRINT_CASE_TIMES), ReprintManager.Reprint_times);
		DWIN_UpdateLCD();
  }
}

void HMI_Forward_Lenght() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, ReprintManager.Forward_lenght)) {
      checkkey = Re_print;
      EncoderRate.enabled = false;
		  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 216, MBASE(MROWS -index_reprint + REPRINT_CASE_LENGHT), ReprintManager.Forward_lenght);
		  ZERO(Reprint_Buf);
		  sprintf_P(Reprint_Buf,PSTR("M180 L%4d"),ReprintManager.Forward_lenght);
		  queue.inject_P(Reprint_Buf);
		  DWIN_UpdateLCD();
      return;
    }
    NOLESS(ReprintManager.Forward_lenght, 0);
    NOMORE(ReprintManager.Forward_lenght, FORWARD_PRINTING_MAX_LENGHT);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 4, 216, MBASE(MROWS -index_reprint + REPRINT_CASE_LENGHT), ReprintManager.Forward_lenght);
		DWIN_UpdateLCD();
  }
}
#endif

#if HAS_HOTEND
char E_Buf[50] = {0};
  	
  void HMI_Move_E1() {
    static float last_E1_scale = 0;
	
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_E1_scale)) {
        checkkey = AxisMove;
        EncoderRate.enabled = false;
        last_E1_scale = HMI_ValueStruct.Move_E1_scale;
		DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX1), HMI_ValueStruct.Move_E1_scale);
		if (!planner.is_full()) {
		  planner.synchronize(); // Wait for planner moves to finish!
          //planner.buffer_line(current_position, MMM_TO_MMS(FEEDRATE_E), active_extruder);
          ZERO(E_Buf);
		  if((HMI_flag.last_E1_Coordinate - current_position.e) <= 0)
        	sprintf_P(E_Buf, PSTR("T0\nG92 E0\nG1 E%.2f F100"),(current_position.e - HMI_flag.last_E1_Coordinate));
		  else
			sprintf_P(E_Buf, PSTR("T0\nG92 E0\nG1 E-%.2f F100"),(HMI_flag.last_E1_Coordinate - current_position.e));

		  #ifdef DEBUG_DWIN_LCD
		  SERIAL_ECHOLNPGM("HMI_Move_E1>>>>	");
		  SERIAL_ECHOLNPAIR("current_position.e=",current_position.e);
		  SERIAL_ECHOLNPAIR("HMI_flag.last_E_Coordinate=",HMI_flag.last_E1_Coordinate);
		  SERIAL_ECHOLNPAIR("HMI_ValueStruct.Move_E1_scale=",HMI_ValueStruct.Move_E1_scale);
		  SERIAL_ECHOLN(E_Buf);
		  #endif

		  queue.inject_P(E_Buf);
		  HMI_flag.last_E1_Coordinate = current_position.e;
        }
        DWIN_UpdateLCD();
        return;
      }
      if ((HMI_ValueStruct.Move_E1_scale - last_E1_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        HMI_ValueStruct.Move_E1_scale = last_E1_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      else if ((last_E1_scale - HMI_ValueStruct.Move_E1_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        HMI_ValueStruct.Move_E1_scale = last_E1_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      current_position.e = HMI_ValueStruct.Move_E1_scale / MINUNITMULT;
      DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX1), HMI_ValueStruct.Move_E1_scale);
      DWIN_UpdateLCD();
    }
  }
  
#if(E_STEPPERS > 1)
 	void HMI_Move_E2() {
    	static float last_E2_scale = 0;
		
    	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    	if (encoder_diffState != ENCODER_DIFF_NO) {
      	if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_E2_scale)) {
        	checkkey = AxisMove;
        	EncoderRate.enabled = false;
        	last_E2_scale = HMI_ValueStruct.Move_E2_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX2), HMI_ValueStruct.Move_E2_scale);
        	if (!planner.is_full()) {
			planner.synchronize(); // Wait for planner moves to finish!
		  	//planner.buffer_line(current_position, MMM_TO_MMS(FEEDRATE_E), active_extruder);
		  	ZERO(E_Buf);
		    if((HMI_flag.last_E2_Coordinate - current_position.e) <= 0)
		    	sprintf_P(E_Buf, PSTR("T1\nG92 E0\nG1 E%.2f F100"),(current_position.e - HMI_flag.last_E2_Coordinate));
			else
			  	sprintf_P(E_Buf, PSTR("T1\nG92 E0\nG1 E-%.2f F100"),(HMI_flag.last_E2_Coordinate - current_position.e));
			#ifdef DEBUG_DWIN_LCD
			SERIAL_ECHOLNPGM("HMI_Move_E2>>>>	");
			SERIAL_ECHOLNPAIR("current_position.e=",current_position.e);
		  	SERIAL_ECHOLNPAIR("HMI_flag.last_E_Coordinate=",HMI_flag.last_E2_Coordinate);
		  	SERIAL_ECHOLNPAIR("HMI_ValueStruct.Move_E2_scale=",HMI_ValueStruct.Move_E2_scale);
			SERIAL_ECHOLN(E_Buf);
			#endif
		    queue.inject_P(E_Buf);
		    HMI_flag.last_E2_Coordinate = current_position.e;
        	}
        	DWIN_UpdateLCD();
        	return;
      	}
      	if ((HMI_ValueStruct.Move_E2_scale - last_E2_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E2_scale = last_E2_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      	else if ((last_E2_scale - HMI_ValueStruct.Move_E2_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E2_scale = last_E2_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
  	    current_position.e = HMI_ValueStruct.Move_E2_scale / MINUNITMULT;
      	DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX2), HMI_ValueStruct.Move_E2_scale);
      	DWIN_UpdateLCD();
    	}
  	}
#endif
 
#if(E_STEPPERS > 2)
 	void HMI_Move_E3() {
   	static float last_E3_scale = 0;
    	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    	if (encoder_diffState != ENCODER_DIFF_NO) {
      	if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_E3_scale)) {
        	checkkey = AxisMove;
        	EncoderRate.enabled = false;
        	last_E3_scale = HMI_ValueStruct.Move_E3_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX3), HMI_ValueStruct.Move_E3_scale);
        	if (!planner.is_full()) {
          	planner.synchronize(); // Wait for planner moves to finish!
          	//planner.buffer_line(current_position, MMM_TO_MMS(FEEDRATE_E), active_extruder);
          	ZERO(E_Buf);
		  if((HMI_flag.last_E3_Coordinate - current_position.e) <= 0)
      		sprintf_P(E_Buf, PSTR("T2\nG92 E0\nG1 E%.2f F100"),(current_position.e - HMI_flag.last_E3_Coordinate));
  			else
  				sprintf_P(E_Buf, PSTR("T2\nG92 E0\nG1 E-%.2f F100"),(HMI_flag.last_E3_Coordinate - current_position.e));
		    #ifdef DEBUG_DWIN_LCD
			SERIAL_ECHOLNPGM("HMI_Move_E3>>>>	");
			SERIAL_ECHOLNPAIR("current_position.e=",current_position.e);
		  	SERIAL_ECHOLNPAIR("HMI_flag.last_E_Coordinate=",HMI_flag.last_E3_Coordinate);
		  	SERIAL_ECHOLNPAIR("HMI_ValueStruct.Move_E3_scale=",HMI_ValueStruct.Move_E3_scale);
			SERIAL_ECHOLN(E_Buf);
		    #endif
		    queue.inject_P(E_Buf);
		    HMI_flag.last_E3_Coordinate = current_position.e;
        	}
        	DWIN_UpdateLCD();
        	return;
      	}
      	if ((HMI_ValueStruct.Move_E3_scale - last_E3_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E3_scale = last_E3_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      	else if ((last_E3_scale - HMI_ValueStruct.Move_E3_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E3_scale = last_E3_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
  	    current_position.e = HMI_ValueStruct.Move_E3_scale / MINUNITMULT;
      	DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX3), HMI_ValueStruct.Move_E3_scale);
      	DWIN_UpdateLCD();
    	}
  	}
    #endif
 
#if(E_STEPPERS > 3)
 	void HMI_Move_E4() {
    	static float last_E4_scale = 0;
    	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    	if (encoder_diffState != ENCODER_DIFF_NO) {
      	if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_E4_scale)) {
        	checkkey = AxisMove;
        	EncoderRate.enabled = false;
        	last_E4_scale = HMI_ValueStruct.Move_E4_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX4), HMI_ValueStruct.Move_E4_scale);
        	if (!planner.is_full()) {
            planner.synchronize(); // Wait for planner moves to finish!
          	//planner.buffer_line(current_position, MMM_TO_MMS(FEEDRATE_E), active_extruder);
			ZERO(E_Buf);
		  if((HMI_flag.last_E4_Coordinate - current_position.e) <= 0)
	      	sprintf_P(E_Buf, PSTR("T3\nG92 E0\nG1 E%.2f F100"),(current_position.e - HMI_flag.last_E4_Coordinate));
			  else
			  	sprintf_P(E_Buf, PSTR("T3\nG92 E0\nG1 E-%.2f F100"),(HMI_flag.last_E4_Coordinate - current_position.e));				
			#ifdef DEBUG_DWIN_LCD
			SERIAL_ECHOLNPGM("HMI_Move_E4>>>> ");
			SERIAL_ECHOLNPAIR("current_position.e=",current_position.e);
			SERIAL_ECHOLNPAIR("HMI_flag.last_E_Coordinate=",HMI_flag.last_E4_Coordinate);
			SERIAL_ECHOLNPAIR("HMI_ValueStruct.Move_E4_scale=",HMI_ValueStruct.Move_E4_scale);
			SERIAL_ECHOLN(E_Buf);
			#endif
		  queue.inject_P(E_Buf);
		  HMI_flag.last_E4_Coordinate = current_position.e;
        	}
        	DWIN_UpdateLCD();
        	return;
      	}
      	if ((HMI_ValueStruct.Move_E4_scale - last_E4_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E4_scale = last_E4_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      	else if ((last_E4_scale - HMI_ValueStruct.Move_E4_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E4_scale = last_E4_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
  	    current_position.e = HMI_ValueStruct.Move_E4_scale / MINUNITMULT;
      	DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX4), HMI_ValueStruct.Move_E4_scale);
      	DWIN_UpdateLCD();
    	}
  	}
	#endif
#if ENABLED(MIXING_EXTRUDER)
void HMI_Move_AllExtr() {
	static float last_EALL_scale = 0;
	uint8_t extruders = MIXING_STEPPERS;
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
  	if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_EAll_scale)) {
    	checkkey = AxisMove;
    	EncoderRate.enabled = false;
    	last_EALL_scale = HMI_ValueStruct.Move_EAll_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EXALL), HMI_ValueStruct.Move_EAll_scale);
    	if (!planner.is_full()) {
        planner.synchronize(); // Wait for planner moves to finish!
      	//planner.buffer_line(current_position, MMM_TO_MMS(60), active_extruder);
				ZERO(E_Buf);
			  if((HMI_flag.last_EALL_Coordinate - current_position.e) <= 0)
	      	sprintf_P(E_Buf, PSTR("T%d\nG92 E0\nG1 E%.2f F%d"),extruders, (current_position.e - HMI_flag.last_EALL_Coordinate), 100);
			  else
			  	sprintf_P(E_Buf, PSTR("T%d\nG92 E0\nG1 E-%.2f F%d"),extruders, (HMI_flag.last_EALL_Coordinate - current_position.e), 100);				
			#ifdef DEBUG_DWIN_LCD
			SERIAL_ECHOLNPGM("HMI_Move_AllExtr>>>> ");
			SERIAL_ECHOLNPAIR("current_position.e=",current_position.e);
			SERIAL_ECHOLNPAIR("HMI_flag.last_E_Coordinate=",HMI_flag.last_EALL_Coordinate);
			SERIAL_ECHOLNPAIR("HMI_ValueStruct.Move_EAll_scale=",HMI_ValueStruct.Move_EAll_scale);
			SERIAL_ECHOLN(E_Buf);
			#endif
			queue.inject_P(E_Buf);
			HMI_flag.last_EALL_Coordinate = current_position.e;
    	}
    	DWIN_UpdateLCD();
    	return;
  	}
  	if ((HMI_ValueStruct.Move_EAll_scale - last_EALL_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
    	HMI_ValueStruct.Move_EAll_scale = last_EALL_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
  	else if ((last_EALL_scale - HMI_ValueStruct.Move_EAll_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
    	HMI_ValueStruct.Move_EAll_scale = last_EALL_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
  	current_position.e = HMI_ValueStruct.Move_EAll_scale / MINUNITMULT;
  	DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EXALL), HMI_ValueStruct.Move_EAll_scale);
  	DWIN_UpdateLCD();
	}
}
#endif
#endif

#if ENABLED(BABYSTEPPING)
bool printer_busy() { return planner.movesplanned() || printingIsActive(); }
static void Apply_ZOffset(){	
	if((ENABLED(BABYSTEP_WITHOUT_HOMING) || all_axes_known()) && (ENABLED(BABYSTEP_ALWAYS_AVAILABLE) || printer_busy())){				
		babyz_offset = (float)HMI_ValueStruct.zoffset_value / 100;
		if(last_babyz_offset != babyz_offset){
      babystep.add_mm(Z_AXIS, babyz_offset - last_babyz_offset);			
			last_babyz_offset = babyz_offset;
		}
	}		
}

#if ENABLED(SAVE_BABYZ_OFFSET)
static void Save_ZOffset(void){	
	if((ENABLED(BABYSTEP_WITHOUT_HOMING) || all_axes_known()) && (ENABLED(BABYSTEP_ALWAYS_AVAILABLE) || printer_busy())){
			coating_thickness += (babyz_offset - prevouis_babyz_offset);
			prevouis_babyz_offset = babyz_offset;
			settings.save();
		}
}
#endif

void HMI_Pop_Babystep() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.zoffset_value)) {
			EncoderRate.enabled = false;
			checkkey = Tune;
			Draw_Tune_Menu();			
			#if ENABLED(SAVE_BABYZ_OFFSET)
			Save_ZOffset();
			#endif			
    	return;
		}
		NOLESS(HMI_ValueStruct.zoffset_value, (Z_PROBE_OFFSET_RANGE_MIN) * 100);
  	NOMORE(HMI_ValueStruct.zoffset_value, (Z_PROBE_OFFSET_RANGE_MAX) * 100);  	
		Apply_ZOffset();
    DWIN_Draw_Signed_Float(font14x28, Color_White, Color_Bg_Black, 2, 2, 170, 160, HMI_ValueStruct.zoffset_value);
    DWIN_UpdateLCD();
	}
}

void HMI_Zoffset() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    uint8_t zoff_line;
		
		#if BOTH(BABYSTEPPING,PREPARE_CASE_ZOFF)
    switch (HMI_ValueStruct.show_mode) {
      case -4: zoff_line = PREPARE_CASE_ZOFF + MROWS - index_prepare; break;
      default: zoff_line = TUNE_CASE_ZOFF + MROWS - index_tune;
    }
		#else
		zoff_line = TUNE_CASE_ZOFF + MROWS - index_tune;
		#endif
  
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.zoffset_value)) {
      EncoderRate.enabled = false;
			#if BOTH(BABYSTEPPING,PREPARE_CASE_ZOFF)
      if (HMI_ValueStruct.show_mode == -4) {
        checkkey = Prepare;
        DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 2, 202, MBASE(zoff_line), HMI_ValueStruct.zoffset_value);
      }
      else 
			#endif
			{
        checkkey = Tune;
        DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 2, 202, MBASE(zoff_line), HMI_ValueStruct.zoffset_value);
      }
      DWIN_UpdateLCD();			
			
		#if ENABLED(SAVE_BABYZ_OFFSET)
			Save_ZOffset();
		#endif
			
      return;
    }
    NOLESS(HMI_ValueStruct.zoffset_value, (Z_PROBE_OFFSET_RANGE_MIN) * 100);
    NOMORE(HMI_ValueStruct.zoffset_value, (Z_PROBE_OFFSET_RANGE_MAX) * 100);
		Apply_ZOffset();
    DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 2, 2, 202, MBASE(zoff_line), HMI_ValueStruct.zoffset_value);
    DWIN_UpdateLCD();
  }
}

#endif // ENABLED(BABYSTEPPING)

#if HAS_HOTEND
  void HMI_ETemp() {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      uint8_t temp_line;
      switch (HMI_ValueStruct.show_mode) {
        case -1: temp_line = TEMP_CASE_TEMP; break;
        case -2: temp_line = PREHEAT_CASE_TEMP; break;
        case -3: temp_line = PREHEAT_CASE_TEMP; break;
        default: temp_line = TUNE_CASE_TEMP + MROWS - index_tune;
      }
      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.E_Temp)) {
        EncoderRate.enabled = false;
        if (HMI_ValueStruct.show_mode == -1) { // temperature
          checkkey = TemperatureID;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(temp_line), HMI_ValueStruct.E_Temp);
        }
        else if (HMI_ValueStruct.show_mode == -2) {
          checkkey = PLAPreheat;
          ui.material_preset[0].hotend_temp = HMI_ValueStruct.E_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(temp_line), ui.material_preset[0].hotend_temp);
          return;
        }
        else if (HMI_ValueStruct.show_mode == -3) {
          checkkey = ABSPreheat;
          ui.material_preset[1].hotend_temp = HMI_ValueStruct.E_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(temp_line), ui.material_preset[1].hotend_temp);
          return;
        }
        else { // tune
          checkkey = Tune;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(temp_line), HMI_ValueStruct.E_Temp);
        }
        thermalManager.setTargetHotend(HMI_ValueStruct.E_Temp, 0);
        return;
      }
      // E_Temp limit
      NOMORE(HMI_ValueStruct.E_Temp, MAX_E_TEMP);
      NOLESS(HMI_ValueStruct.E_Temp, MIN_E_TEMP);
      // E_Temp value
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(temp_line), HMI_ValueStruct.E_Temp);
    }
  }
#endif // HAS_HOTEND

#if HAS_HEATED_BED
  void HMI_BedTemp() {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      uint8_t bed_line;
      switch (HMI_ValueStruct.show_mode) {
        case -1: bed_line = TEMP_CASE_BED; break;
        case -2: bed_line = PREHEAT_CASE_BED; break;
        case -3: bed_line = PREHEAT_CASE_BED; break;
        default: bed_line = TUNE_CASE_BED + MROWS - index_tune;
      }
      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Bed_Temp)) {
        EncoderRate.enabled = false;
        if (HMI_ValueStruct.show_mode == -1) {
          checkkey = TemperatureID;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(bed_line), HMI_ValueStruct.Bed_Temp);
        }
        else if (HMI_ValueStruct.show_mode == -2) {
          checkkey = PLAPreheat;
          ui.material_preset[0].bed_temp = HMI_ValueStruct.Bed_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(bed_line), ui.material_preset[0].bed_temp);
          return;
        }
        else if (HMI_ValueStruct.show_mode == -3) {
          checkkey = ABSPreheat;
          ui.material_preset[1].bed_temp = HMI_ValueStruct.Bed_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(bed_line), ui.material_preset[1].bed_temp);
          return;
        }
        else {
          checkkey = Tune;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(bed_line), HMI_ValueStruct.Bed_Temp);
        }
        thermalManager.setTargetBed(HMI_ValueStruct.Bed_Temp);
        return;
      }
      // Bed_Temp limit
      NOMORE(HMI_ValueStruct.Bed_Temp, BED_MAX_TARGET);
      NOLESS(HMI_ValueStruct.Bed_Temp, MIN_BED_TEMP);
      // Bed_Temp value
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(bed_line), HMI_ValueStruct.Bed_Temp);
    }
  }
#endif // HAS_HEATED_BED

#if HAS_PREHEAT
  void HMI_FanSpeed() {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      uint8_t fan_line;
      switch (HMI_ValueStruct.show_mode) {
        case -1: fan_line = TEMP_CASE_FAN; break;
        case -2: fan_line = PREHEAT_CASE_FAN; break;
        case -3: fan_line = PREHEAT_CASE_FAN; break;
        default: fan_line = TUNE_CASE_FAN + MROWS - index_tune;
      }

      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Fan_speed)) {
        EncoderRate.enabled = false;
        if (HMI_ValueStruct.show_mode == -1) {
          checkkey = TemperatureID;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(fan_line), HMI_ValueStruct.Fan_speed);
        }
        else if (HMI_ValueStruct.show_mode == -2) {
          checkkey = PLAPreheat;
          ui.material_preset[0].fan_speed = HMI_ValueStruct.Fan_speed;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(fan_line), ui.material_preset[0].fan_speed);
          return;
        }
        else if (HMI_ValueStruct.show_mode == -3) {
          checkkey = ABSPreheat;
          ui.material_preset[1].fan_speed = HMI_ValueStruct.Fan_speed;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(fan_line), ui.material_preset[1].fan_speed);
          return;
        }
        else {
          checkkey = Tune;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(fan_line), HMI_ValueStruct.Fan_speed);
        }
        thermalManager.set_fan_speed(0, HMI_ValueStruct.Fan_speed);
        return;
      }
      // Fan_speed limit
      NOMORE(HMI_ValueStruct.Fan_speed, FANON);
      NOLESS(HMI_ValueStruct.Fan_speed, FANOFF);
      // Fan_speed value
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(fan_line), HMI_ValueStruct.Fan_speed);
    }
  }
#endif // HAS_PREHEAT

void HMI_PrintSpeed() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.print_speed)) {
      checkkey = Tune;
      EncoderRate.enabled = false;
      feedrate_percentage = HMI_ValueStruct.print_speed;
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(select_tune.now + MROWS - index_tune), HMI_ValueStruct.print_speed);
      return;
    }
    // print_speed limit
    NOMORE(HMI_ValueStruct.print_speed, MAX_PRINT_SPEED);
    NOLESS(HMI_ValueStruct.print_speed, MIN_PRINT_SPEED);
    // print_speed value
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(select_tune.now + MROWS - index_tune), HMI_ValueStruct.print_speed);
  }
}

void HMI_MaxFeedspeedXYZE() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Feedspeed)) {
      checkkey = MaxSpeed;
      EncoderRate.enabled = false;
      if (WITHIN(HMI_flag.feedspeed_axis, X_AXIS, E_AXIS))
        planner.set_max_feedrate(HMI_flag.feedspeed_axis, HMI_ValueStruct.Max_Feedspeed);
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(select_speed.now), HMI_ValueStruct.Max_Feedspeed);
      return;
    }
    // MaxFeedspeed limit
    if (WITHIN(HMI_flag.feedspeed_axis, X_AXIS, E_AXIS))
      NOMORE(HMI_ValueStruct.Max_Feedspeed, default_max_feedrate[HMI_flag.feedspeed_axis] * 4);
    if (HMI_ValueStruct.Max_Feedspeed < MIN_MAXFEEDSPEED) HMI_ValueStruct.Max_Feedspeed = MIN_MAXFEEDSPEED;
    // MaxFeedspeed value
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 4, 210, MBASE(select_speed.now), HMI_ValueStruct.Max_Feedspeed);
  }
}

void HMI_MaxAccelerationXYZE() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Acceleration)) {
      checkkey = MaxAcceleration;
      EncoderRate.enabled = false;
      if (HMI_flag.acc_axis == X_AXIS) planner.set_max_acceleration(X_AXIS, HMI_ValueStruct.Max_Acceleration);
      else if (HMI_flag.acc_axis == Y_AXIS) planner.set_max_acceleration(Y_AXIS, HMI_ValueStruct.Max_Acceleration);
      else if (HMI_flag.acc_axis == Z_AXIS) planner.set_max_acceleration(Z_AXIS, HMI_ValueStruct.Max_Acceleration);
      #if HAS_HOTEND
        else if (HMI_flag.acc_axis == E_AXIS) planner.set_max_acceleration(E_AXIS, HMI_ValueStruct.Max_Acceleration);
      #endif
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 5, 210, MBASE(select_acc.now), HMI_ValueStruct.Max_Acceleration);
      return;
    }
    // MaxAcceleration limit
    if (WITHIN(HMI_flag.acc_axis, X_AXIS, E_AXIS))
      NOMORE(HMI_ValueStruct.Max_Acceleration, default_max_acceleration[HMI_flag.acc_axis] * 4);
    if (HMI_ValueStruct.Max_Acceleration < MIN_MAXACCELERATION) HMI_ValueStruct.Max_Acceleration = MIN_MAXACCELERATION;
    // MaxAcceleration value
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 5, 210, MBASE(select_acc.now), HMI_ValueStruct.Max_Acceleration);
  }
}

#if HAS_CLASSIC_JERK
  void HMI_MaxJerkXYZE() {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Jerk)) {
        checkkey = MaxJerk;
        EncoderRate.enabled = false;
        if (WITHIN(HMI_flag.jerk_axis, X_AXIS, E_AXIS))
          planner.set_max_jerk(HMI_flag.jerk_axis, HMI_ValueStruct.Max_Jerk / 10);
        DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 1, 210, MBASE(select_jerk.now), HMI_ValueStruct.Max_Jerk);
        return;
      }
      // MaxJerk limit
      if (WITHIN(HMI_flag.jerk_axis, X_AXIS, E_AXIS))
        NOMORE(HMI_ValueStruct.Max_Jerk, default_max_jerk[HMI_flag.jerk_axis] * 4 * MINUNITMULT);
      NOLESS(HMI_ValueStruct.Max_Jerk, (MIN_MAXJERK) * MINUNITMULT);
      // MaxJerk value
      DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 2, 1, 210, MBASE(select_jerk.now), HMI_ValueStruct.Max_Jerk);
    }
  }
#endif // HAS_CLASSIC_JERK

void HMI_StepXYZE() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Step)) {
      checkkey = Step;
      EncoderRate.enabled = false;
      if (WITHIN(HMI_flag.step_axis, X_AXIS, E_AXIS))
        planner.settings.axis_steps_per_mm[HMI_flag.step_axis] = HMI_ValueStruct.Max_Step / 10;
      DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 4, 1, 210, MBASE(select_step.now), HMI_ValueStruct.Max_Step);
      return;
    }
    // Step limit
    if (WITHIN(HMI_flag.step_axis, X_AXIS, E_AXIS))
      NOMORE(HMI_ValueStruct.Max_Step, default_axis_steps_per_unit[HMI_flag.step_axis] * 4 * MINUNITMULT);
    NOLESS(HMI_ValueStruct.Max_Step, MIN_STEP);
    // Step value
    DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 4, 1, 210, MBASE(select_step.now), HMI_ValueStruct.Max_Step);
  }
}

void X_Start_Coordinate_Calculation(uint8_t Extruder_number,uint16_t Coordinate){
	uint8_t j = 0;

	MIXER_STEPPER_LOOP(i){
		if(Coordinate >= 100) MixerDis.Extruder_X_Coordinate[i] = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*i, MixerDis.Extruder_Int_Number[i] = 3;
		if((Coordinate < 100)&&(Coordinate >= 10)) MixerDis.Extruder_X_Coordinate[i] = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+7+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*i, MixerDis.Extruder_Int_Number[i] = 2;
		if(Coordinate < 10) MixerDis.Extruder_X_Coordinate[i] = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+14+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*i, MixerDis.Extruder_Int_Number[i] = 1;
		j = i;
    }
	j++;
	if(Extruder_number == MIXING_STEPPERS){
		if((Coordinate < 100)&&(Coordinate >= 10)) MixerDis.VTool_X_Coordinate = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+7+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*j, MixerDis.VTool_Int_Number = 2;
		if(Coordinate < 10) MixerDis.VTool_X_Coordinate = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+14+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*j, MixerDis.VTool_Int_Number = 1;
	}
}

void Refresh_Percent_display(){
	DWIN_Draw_Rectangle(1, Color_Bg_Black, MixerDis.Area_X_Start, MixerDis.Area_Y_Start, MixerDis.Area_X_End, MixerDis.Area_Y_End);
	MIXER_STEPPER_LOOP(i){
		X_Start_Coordinate_Calculation(i,mixer.mix[i]);
	    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.Extruder_Int_Number[i], MixerDis.Extruder_X_Coordinate[i], MixerDis.Y_Coordinate, mixer.mix[i]);
		/*
		if(MixerCfg.Mixer_Mode_Rg == 0){
			X_Start_Coordinate_Calculation(MIXING_STEPPERS,MixerCfg.Vtool_Backup);
			DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_Int_Number, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, MixerCfg.Vtool_Backup);
		}
		else if(MixerCfg.Mixer_Mode_Rg == 1){
			X_Start_Coordinate_Calculation(MIXING_STEPPERS,10);
			DWIN_Draw_String(false, false, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, F("Gr"));
		}
		else if(MixerCfg.Mixer_Mode_Rg == 2){
			X_Start_Coordinate_Calculation(MIXING_STEPPERS,10);
			DWIN_Draw_String(false, false, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, F("Rd"));
		}
		*/
		if(mixer.gradient.enabled){
			X_Start_Coordinate_Calculation(MIXING_STEPPERS,10);
			DWIN_Draw_String(false, false, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, F("Gr"));
		}
		else if(mixer.random_mix.enabled){
			X_Start_Coordinate_Calculation(MIXING_STEPPERS,10);
			DWIN_Draw_String(false, false, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, F("Rd"));
		}
		else{
			//X_Start_Coordinate_Calculation(MIXING_STEPPERS,MixerCfg.Vtool_Backup);
			//DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_Int_Number, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, MixerCfg.Vtool_Backup);
            X_Start_Coordinate_Calculation(MIXING_STEPPERS,mixer.selected_vtool);
			DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_Int_Number, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, mixer.selected_vtool);
		}
	}
}

void update_variable() {
  #if HAS_HOTEND
    static float last_temp_hotend_target = 0, last_temp_hotend_current = 0;
  #endif
  #if HAS_HEATED_BED
    static float last_temp_bed_target = 0, last_temp_bed_current = 0;
  #endif
  #if HAS_FAN
    static uint8_t last_fan_speed = 0;
  #endif
	static float last_z_pos = 0.0;
  #if ENABLED(MIXING_EXTRUDER)
  	static uint8_t last_mixer_percent[MIXING_STEPPERS] = {0};
    static uint8_t last_vtool = 0;
	bool bupdata = false;
  #endif

  /* Tune page temperature update */
  if (checkkey == Tune) {
    #if HAS_HOTEND
      if (last_temp_hotend_target != thermalManager.temp_hotend[0].target)
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_TEMP + MROWS - index_tune), thermalManager.temp_hotend[0].target);
    #endif
    #if HAS_HEATED_BED
      if (last_temp_bed_target != thermalManager.temp_bed.target)
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_BED + MROWS - index_tune), thermalManager.temp_bed.target);
    #endif
    #if HAS_FAN
      if (last_fan_speed != thermalManager.fan_speed[0]) {
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_FAN + MROWS - index_tune), thermalManager.fan_speed[0]);
        last_fan_speed = thermalManager.fan_speed[0];
      }
    #endif
  }

  /* Temperature page temperature update */
  if (checkkey == TemperatureID) {
    #if HAS_HOTEND
      if (last_temp_hotend_target != thermalManager.temp_hotend[0].target)
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TEMP_CASE_TEMP), thermalManager.temp_hotend[0].target);
    #endif
    #if HAS_HEATED_BED
      if (last_temp_bed_target != thermalManager.temp_bed.target)
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TEMP_CASE_BED), thermalManager.temp_bed.target);
    #endif
    #if HAS_FAN
      if (last_fan_speed != thermalManager.fan_speed[0]) {
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TEMP_CASE_FAN), thermalManager.fan_speed[0]);
        last_fan_speed = thermalManager.fan_speed[0];
      }
    #endif
  }

  /* Bottom temperature update */
  #if HAS_HOTEND
    if (last_temp_hotend_current != thermalManager.temp_hotend[0].celsius) {
      DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_extruder_num, State_text_extruder_X, State_text_extruder_Y, thermalManager.temp_hotend[0].celsius);
      last_temp_hotend_current = thermalManager.temp_hotend[0].celsius;
    }
    if (last_temp_hotend_target != thermalManager.temp_hotend[0].target) {
      DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_extruder_num, State_text_extruder_X + (State_text_extruder_num + 1) * STAT_CHR_W, State_text_extruder_Y, thermalManager.temp_hotend[0].target);
      last_temp_hotend_target = thermalManager.temp_hotend[0].target;
    }
  #endif
  
  #if HAS_HEATED_BED
    if (last_temp_bed_current != thermalManager.temp_bed.celsius) {
      DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_bed_num, State_text_bed_X, State_text_bed_Y, thermalManager.temp_bed.celsius);
      last_temp_bed_current = thermalManager.temp_bed.celsius;
    }
    if (last_temp_bed_target != thermalManager.temp_bed.target) {
      DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_bed_num, State_text_bed_X + (State_text_bed_num + 1) * STAT_CHR_W, State_text_bed_Y, thermalManager.temp_bed.target);
      last_temp_bed_target = thermalManager.temp_bed.target;
    }
  #endif
  
  static uint16_t last_speed = 0;
  if (last_speed != feedrate_percentage) {
    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_speed_num, State_text_speed_X, State_text_speed_Y, feedrate_percentage);
    last_speed = feedrate_percentage;
  }

	//updata Z position
	if((checkkey == PrintProcess) || (checkkey == MainMenu) || (checkkey == Leveling0) || (checkkey == Last_Leveling) || (checkkey == Last_Level_CatchOffset)|| (checkkey == Last_Level_CatchPop)){
		if(all_axes_known()){
			if (last_z_pos != current_position.z)
				DWIN_Draw_Signed_Float(DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_Zoffset_inum, State_text_Zoffset_fnum, State_text_Zoffset_X, State_text_Zoffset_Y, 100*current_position.z);
			last_z_pos = current_position.z;
		}
		else
			DWIN_Draw_String(false, true, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_Zoffset_X,State_text_Zoffset_Y, F("  ???  "));		
	}	

  #if ENABLED(MIXING_EXTRUDER)
  if((checkkey == PrintProcess)&&(!HMI_flag.filament_runout_star)) {
		//mixing rate changed?
		MIXER_STEPPER_LOOP(i){
			if(last_mixer_percent[i] != mixer.mix[i]){
				bupdata = true;
				break;
			}
		}
		//vool changed?
		if(last_vtool != MixerCfg.Vtool_Backup){
			bupdata = true;
			last_vtool = mixer.selected_vtool = MixerCfg.Vtool_Backup;
			updata_mixer_from_vtool();
			//Draw_Print_ProgressModel();
		}
	
		if(bupdata || HMI_flag.refersh_mix_flag){		
			MIXER_STEPPER_LOOP(i) last_mixer_percent[i] = mixer.mix[i];
    	Refresh_Percent_display();
			Draw_Print_ProgressModel();
			HMI_flag.refersh_mix_flag = false;
		}
 }	
 #endif
}

/**
 * Read and cache the working directory.
 *
 * TODO: New code can follow the pattern of menu_media.cpp
 * and rely on Marlin caching for performance. No need to
 * cache files here.
 */

#ifndef strcasecmp_P
  #define strcasecmp_P(a, b) strcasecmp((a), (b))
#endif

inline void make_name_without_ext(char *dst, char *src, size_t maxlen=MENU_CHAR_LIMIT) {
  char * const name = card.longest_filename();
  size_t pos        = strlen(name); // index of ending nul

  // For files, remove the extension
  // which may be .gcode, .gco, or .g
  if (!card.flag.filenameIsDir)
    while (pos && src[pos] != '.') pos--; // find last '.' (stop at 0)

  size_t len = pos;   // nul or '.'
  if (len > maxlen) { // Keep the name short
    pos        = len = maxlen; // move nul down
    dst[--pos] = '.'; // insert dots
    dst[--pos] = '.';
    dst[--pos] = '.';
  }

  dst[len] = '\0';    // end it

  // Copy down to 0
  while (pos--) dst[pos] = src[pos];
}

inline void HMI_SDCardInit() {	
	if (card.flag.workDirIsRoot) {
    #if !PIN_EXISTS(SD_DETECT)
      card.mount();
    #endif
  }
	card.cdroot();
}

void MarlinUI::refresh() { /* Nothing to see here */ }

#define ICON_Folder ICON_More

#if ENABLED(SCROLL_LONG_FILENAMES)

  char shift_name[LONG_FILENAME_LENGTH + 1];
  int8_t shift_amt; // = 0
  millis_t shift_ms; // = 0

  // Init the shift name based on the highlighted item
  inline void Init_Shift_Name() {
    const bool is_subdir = !card.flag.workDirIsRoot;
    const int8_t filenum = select_file.now - 1 - is_subdir; // Skip "Back" and ".."
    const uint16_t fileCnt = card.get_num_Files();
    if (WITHIN(filenum, 0, fileCnt - 1)) {
      card.getfilename_sorted(SD_ORDER(filenum, fileCnt));
      char * const name = card.longest_filename();
      make_name_without_ext(shift_name, name, 100);
    }
  }

  inline void Init_SDItem_Shift() {
    shift_amt = 0;
    shift_ms  = select_file.now > 0 && strlen(shift_name) > MENU_CHAR_LIMIT
           ? millis() + 750UL : 0;
  }
#endif

/**
 * Display an SD item, adding a CDUP for subfolders.
 */
inline void Draw_SDItem(const uint16_t item, int16_t row=-1) {
  if (row < 0) row = item + 1 + MROWS - index_file;
  const bool is_subdir = !card.flag.workDirIsRoot;
  if (is_subdir && item == 0) {
    Draw_Menu_Line(row, ICON_Folder, "..");
    return;
  }

  card.getfilename_sorted(item - is_subdir);
  char * const name = card.longest_filename();

  #if ENABLED(SCROLL_LONG_FILENAMES)
    // Init the current selected name
    // This is used during scroll drawing
    if (item == select_file.now - 1) {
      make_name_without_ext(shift_name, name, 100);
      Init_SDItem_Shift();
    }
  #endif

  // Draw the file/folder with name aligned left
  char str[strlen(name) + 1];
  make_name_without_ext(str, name);
  Draw_Menu_Line(row, card.flag.filenameIsDir ? ICON_Folder : ICON_File, str);
}

#if ENABLED(SCROLL_LONG_FILENAMES)
  inline void Draw_SDItem_Shifted(int8_t &shift) {
    // Limit to the number of chars past the cutoff
    const size_t len = strlen(shift_name);
    NOMORE(shift, _MAX(len - MENU_CHAR_LIMIT, 0U));

    // Shorten to the available space
    const size_t lastchar = _MIN((signed)len, shift + MENU_CHAR_LIMIT);

    const char c = shift_name[lastchar];
    shift_name[lastchar] = '\0';

    const uint8_t row = select_file.now + MROWS - index_file; // skip "Back" and scroll
    Erase_Menu_Text(row);
    Draw_Menu_Line(row, 0, &shift_name[shift]);

    shift_name[lastchar] = c;
  }
#endif

// Redraw the first set of SD Files
inline void Redraw_SD_List() {
  select_file.reset();
  index_file = MROWS;

  Clear_Menu_Area(); // Leave title bar unchanged

  Draw_Back_First();

  //card.mount();
  
  if (card.isMounted()) {
  //if (card.flag.mounted) {
    // As many files as will fit
    LOOP_L_N(i, _MIN(nr_sd_menu_items(), MROWS))
      Draw_SDItem(i, i+1);

    TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());
  }
  else {
    DWIN_Draw_Rectangle(1, Color_Bg_Red, 10, MBASE(3) - 10, DWIN_WIDTH - 10, MBASE(4));
    DWIN_Draw_String(false, false, font16x32, Color_Yellow, Color_Bg_Red, ((DWIN_WIDTH) - 8 * 16) / 2, MBASE(3), F("No Media"));
  }
}

bool DWIN_lcd_sd_status = false;

inline void SDCard_Up() {
  card.cdup();
  Redraw_SD_List();
  DWIN_lcd_sd_status = true; // On next DWIN_Update
}

inline void SDCard_Folder(char * const dirname) {
  card.cd(dirname);
  Redraw_SD_List();
  DWIN_lcd_sd_status = true; // On next DWIN_Update
}

//
// Watch for media mount / unmount
//
void HMI_SDCardUpdate() {
  if (DWIN_lcd_sd_status != card.isMounted()) {
    DWIN_lcd_sd_status = card.isMounted();
    // SERIAL_ECHOLNPAIR("HMI_SDCardUpdate: ", int(DWIN_lcd_sd_status));
    if (DWIN_lcd_sd_status) {
      if (checkkey == SelectFile)
        Redraw_SD_List();
    }
    else {
      // clean file icon
      if (checkkey == SelectFile) {
        Redraw_SD_List();
      }
      else if (checkkey == PrintProcess || checkkey == Tune || printingIsActive()) {
      //if (checkkey == PrintProcess || checkkey == Tune || printingIsActive()) {
       /// TODO: Move card removed abort handling
        //       to CardReader::manage_media.
        card.flag.abort_sd_printing = true;
        wait_for_heatup = wait_for_user = false;
        dwin_abort_flag = true; // Reset feedrate, return to Home
      }
    }
    DWIN_UpdateLCD();
  }
}

//
// The status area is always on-screen, except during
// full-screen modal dialogs. (TODO: Keep alive during dialogs)
//
void Draw_Status_Area(const bool with_update) {
  // Clear the bottom area of the screen
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, STATUS_Y_START, DWIN_WIDTH, STATUS_Y_END);
  //
  // Status Area
  //
  #if HAS_HOTEND
    DWIN_ICON_Show(ICON, ICON_HotendTemp, State_icon_extruder_X, State_icon_extruder_Y);
    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_extruder_num, State_text_extruder_X, State_text_extruder_Y, thermalManager.temp_hotend[0].celsius);
    DWIN_Draw_String(false, false, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_string_extruder_X, State_string_extruder_Y, F("/"));
    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_extruder_num, State_text_extruder_X + (State_text_extruder_num + 1) * STAT_CHR_W, State_text_extruder_Y, thermalManager.temp_hotend[0].target);
  #endif
  #if HOTENDS > 1
    // DWIN_ICON_Show(ICON,ICON_HotendTemp, 13, 381);
  #endif

  #if HAS_HEATED_BED
    DWIN_ICON_Show(ICON, ICON_BedTemp, State_icon_bed_X, State_icon_bed_Y);
    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_bed_num, State_text_bed_X, State_text_bed_Y, thermalManager.temp_bed.celsius);
    DWIN_Draw_String(false, false, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_string_bed_X, State_string_bed_Y, F("/"));
    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_bed_num, State_text_bed_X + (State_text_extruder_num + 1) * STAT_CHR_W, State_text_bed_Y, thermalManager.temp_bed.target);
  #endif

  DWIN_ICON_Show(ICON, ICON_Speed, State_icon_speed_X, State_icon_speed_Y);
  DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_speed_num, State_text_speed_X, State_text_speed_Y, feedrate_percentage);
  DWIN_Draw_String(false, false, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_string_speed_X, State_string_speed_Y, F("%"));
  
	//
  DWIN_ICON_Show(ICON, ICON_Zoffset, State_icon_Zoffset_X, State_icon_Zoffset_Y);
  if(all_axes_known())
    DWIN_Draw_Signed_Float(DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_Zoffset_inum, State_text_Zoffset_fnum, State_text_Zoffset_X, State_text_Zoffset_Y, current_position.z * 100);
	else
		DWIN_Draw_String(false, true, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_Zoffset_X,State_text_Zoffset_Y, F("  ???  "));	


  if (with_update) {
    DWIN_UpdateLCD();
    delay(5);
  }
}

void Draw_Mixer_Status_Area(const bool with_update) {
  //
  // Mixer Status Area
  //
  #if ENABLED(MIXING_EXTRUDER)
   	mixer.selected_vtool = MixerCfg.Vtool_Backup;
  	updata_mixer_from_vtool();
  	Refresh_Percent_display();
  #endif

  if (with_update) {
    DWIN_UpdateLCD();
    delay(5);
  }
}

void HMI_StartFrame(const bool with_update) {
  Goto_MainMenu();
  Draw_Status_Area(with_update);
  mixer.selected_vtool = MixerCfg.Vtool_Backup;
  //Draw_Print_ProgressModel();
}

inline void Draw_Info_Menu() {
  Clear_Main_Window();

  #if INFO_CASE_TOTAL >= 6
    const int16_t iscroll = MROWS - index_info; // Scrolled-up lines
    #define ICSCROL(L) (iscroll + (L))
  #else
    #define ICSCROL(L) (L)
  #endif
  #define ICLINE(L)  MBASE(ICSCROL(L))
  #define ICVISI(L)  WITHIN(ICSCROL(L), 0, MROWS)
 
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_StarInfo, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  if (ICVISI(0)) Draw_Back_First(select_info.now == 0);                         						// < Back

  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, ICLINE(INFO_CASE_VERSION), F("Version:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Version:")+1)*MENU_CHR_W, ICLINE(INFO_CASE_VERSION), (char*)FIRMWARE_VERSION);    	
  DWIN_Draw_Line(Line_Color, 16, ICLINE(INFO_CASE_VERSION) + 33, 256, ICLINE(INFO_CASE_VERSION) + 34);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, ICLINE(INFO_CASE_FIRMWARE), F("Firmware:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Firmware:")+1)*MENU_CHR_W, ICLINE(INFO_CASE_FIRMWARE), (char*)SHORT_BUILD_VERSION);  
  DWIN_Draw_Line(Line_Color, 16, ICLINE(INFO_CASE_FIRMWARE) + 33, 256, ICLINE(INFO_CASE_FIRMWARE) + 34);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, ICLINE(INFO_CASE_WEBSITE), F("Website:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Website:")+1)*MENU_CHR_W, ICLINE(INFO_CASE_WEBSITE), (char*)WEBSITE_URL);  
  DWIN_Draw_Line(Line_Color, 16, ICLINE(INFO_CASE_WEBSITE) + 33, 256, ICLINE(INFO_CASE_WEBSITE) + 34);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, ICLINE(INFO_CASE_MODEL), F("Model:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Model:")+1)*MENU_CHR_W, ICLINE(INFO_CASE_MODEL), (char*)CUSTOM_MACHINE_NAME);  
  DWIN_Draw_Line(Line_Color, 16, ICLINE(INFO_CASE_MODEL) + 33, 256, ICLINE(INFO_CASE_MODEL) + 34);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO, ICLINE(INFO_CASE_BOARD), F("Board:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX_INFO + (strlen("Board:")+1)*MENU_CHR_W, ICLINE(INFO_CASE_BOARD), (char*)BOARD_INFO_NAME);  
  DWIN_Draw_Line(Line_Color, 16, ICLINE(INFO_CASE_BOARD) + 33, 256, ICLINE(INFO_CASE_BOARD) + 34);
  if (select_info.now && ICVISI(select_info.now))
    Draw_Menu_Cursor(ICSCROL(select_info.now)); 
   
  /*
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 96, F("Version:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 96, (char*)FIRMWARE_VERSION);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 138, F("Firmware:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 138, (char*)SHORT_BUILD_VERSION);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 180, F("Website:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 180, (char*)WEBSITE_URL);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 222, F("Model:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 222, (char*)CUSTOM_MACHINE_NAME);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 264, F("Board:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 264, (char*)BOARD_INFO_NAME);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 306, F("Date:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 306, (char*)STRING_DISTRIBUTION_DATE);
  Draw_Back_First();
  LOOP_L_N(i, 6) {
    DWIN_Draw_Line(Line_Color, 0, 83 + (i+1) * 42, 256, 84 + (i+1) * 42);
  }
  */
}

inline void Draw_Print_File_Menu() {
  Clear_Title_Bar();
  Clear_Bottom_Area();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Print_File], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Print, Menu_Coordinate,14, 7);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_File, Menu_Coordinate,14+Print_File_X_Coordinate[HMI_flag.language], 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  Redraw_SD_List();
}

/* Main Process */
void HMI_MainMenu() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_page.inc(4)) {
      switch (select_page.now) {
        case 0: ICON_Print(); break;
        case 1: ICON_Print(); ICON_Prepare(); break;
        case 2: ICON_Prepare(); ICON_Control(); break;
        case 3: ICON_Control(); TERN(HAS_ONESTEP_LEVELING, ICON_Leveling, ICON_StartInfo)(1); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_page.dec()) {
      switch (select_page.now) {
        case 0: ICON_Print(); ICON_Prepare(); break;
        case 1: ICON_Prepare(); ICON_Control(); break;
        case 2: ICON_Control(); TERN(HAS_ONESTEP_LEVELING, ICON_Leveling, ICON_StartInfo)(0); break;
        case 3: TERN(HAS_ONESTEP_LEVELING, ICON_Leveling, ICON_StartInfo)(1); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_page.now) {
      case 0: // Print File
        	checkkey = SelectFile;
			index_file = MROWS;
			HMI_SDCardInit();
        	Draw_Print_File_Menu();
        break;

      case 1: // Prepare
        	checkkey = Prepare;
        	select_prepare.reset();
        	index_prepare = MROWS;
        	Draw_Prepare_Menu();
        break;

      case 2: // Control
        checkkey = Control;
        select_control.reset();
        index_control = MROWS;
        Draw_Control_Menu();
        break;

      case 3: // Leveling or Info
        #if HAS_ONESTEP_LEVELING
          checkkey = Leveling;
          HMI_Leveling();
        #else
          checkkey = Info;
		  		select_info.reset();
          index_info = MROWS;
          Draw_Info_Menu();
        #endif
        break;
    }
  }
  DWIN_UpdateLCD();
}

// Select (and Print) File
void HMI_SelectFile() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();

  const uint16_t hasUpDir = !card.flag.workDirIsRoot;

  if (encoder_diffState == ENCODER_DIFF_NO) {
    #if ENABLED(SCROLL_LONG_FILENAMES)
      if (shift_ms && select_file.now >= 1 + hasUpDir) {
        // Scroll selected filename every second
        const millis_t ms = millis();
        if (ELAPSED(ms, shift_ms)) {
          const bool was_reset = shift_amt < 0;
          shift_ms = ms + 375UL + was_reset * 250UL;  // ms per character
          int8_t shift_new = shift_amt + 1;           // Try to shift by...
          Draw_SDItem_Shifted(shift_new);             // Draw the item
          if (!was_reset && shift_new == 0)           // Was it limited to 0?
            shift_ms = 0;                             // No scrolling needed
          else if (shift_new == shift_amt)            // Scroll reached the end
            shift_new = -1;                           // Reset
          shift_amt = shift_new;                      // Set new scroll
        }
      }
    #endif
    return;
  }

  // First pause is long. Easy.
  // On reset, long pause must be after 0.

  const uint16_t fullCnt = nr_sd_menu_items();

  if (encoder_diffState == ENCODER_DIFF_CW && fullCnt) {
    if (select_file.inc(1 + fullCnt)) {
      const uint8_t itemnum = select_file.now - 1;              // -1 for "Back"
      if (TERN0(SCROLL_LONG_FILENAMES, shift_ms)) {             // If line was shifted
        Erase_Menu_Text(itemnum + MROWS - index_file);          // Erase and
        Draw_SDItem(itemnum - 1);                               // redraw
      }
	  
      if (select_file.now > MROWS && select_file.now > index_file) { // Cursor past the bottom
        index_file = select_file.now;                           // New bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_SDItem(itemnum, MROWS);                            // Draw and init the shift name
      }
      else {
        Move_Highlight(1, select_file.now + MROWS - index_file); // Just move highlight
        TERN_(SCROLL_LONG_FILENAMES, Init_Shift_Name());         // ...and init the shift name
      }
      TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW && fullCnt) {
    if (select_file.dec()) {
      const uint8_t itemnum = select_file.now - 1;              // -1 for "Back"
      if (TERN0(SCROLL_LONG_FILENAMES, shift_ms)) {             // If line was shifted
        Erase_Menu_Text(select_file.now + 1 + MROWS - index_file); // Erase and
        Draw_SDItem(itemnum + 1);                               // redraw
      }
	  
      if (select_file.now < index_file - MROWS) {               // Cursor past the top
        index_file--;                                           // New bottom line
        Scroll_Menu(DWIN_SCROLL_DOWN);
        if (index_file == MROWS) {
          Draw_Back_First();
          TERN_(SCROLL_LONG_FILENAMES, shift_ms = 0);
        }
        else {
          Draw_SDItem(itemnum, 0);                              // Draw the item (and init shift name)
        }
      }
      else {
        Move_Highlight(-1, select_file.now + MROWS - index_file); // Just move highlight
        TERN_(SCROLL_LONG_FILENAMES, Init_Shift_Name());        // ...and init the shift name
      }
      TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());        // Reset left. Init timer.
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (select_file.now == 0) { // Back
      select_page.set(0);
      Goto_MainMenu();
    }
    else if (hasUpDir && select_file.now == 1) { // CD-Up
      SDCard_Up();
      goto HMI_SelectFileExit;
    }
    else {
      const uint16_t filenum = select_file.now - 1 - hasUpDir;
      card.getfilename_sorted(SD_ORDER(filenum, card.get_num_Files()));

      // Enter that folder!
      if (card.flag.filenameIsDir) {
        SDCard_Folder(card.filename);
        goto HMI_SelectFileExit;
      }

      // Reset highlight for next entry
      select_print.reset();
      select_file.reset();

      // Start choice and print SD file
      HMI_flag.heat_flag = true;     
      HMI_ValueStruct.show_mode = 0;
			HMI_flag.print_finish = false;

      card.openAndPrintFile(card.filename);

      #if FAN_COUNT > 0
        // All fans on for Ender 3 v2 ?
        // The slicer should manage this for us.
        // for (uint8_t i = 0; i < FAN_COUNT; i++)
        //  thermalManager.fan_speed[i] = FANON;
      #endif

      Goto_PrintProcess();
    }
  }
HMI_SelectFileExit:
  DWIN_UpdateLCD();
}

/* Printing */
void HMI_Printing() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (HMI_flag.done_confirm_flag) {
    if (encoder_diffState == ENCODER_DIFF_ENTER) {
		  #if HAS_SUICIDE
				//reload timer
      	HMI_flag.putdown_close_machine = 0;
		  	HMI_flag.putdown_close_timer_rg = POWERDOWN_MACHINE_TIMER;
		  #endif
				HMI_flag.done_confirm_flag = false;
		  #if ENABLED(OPTION_REPEAT_PRINTING)
			  ReprintManager.Is_Reprint_Print = false;
			  ReprintManager.reprt_state = REPRINT_INIT;
		  #endif	      
				dwin_abort_flag = true; // Reset feedrate, return to Home
    }
    return;
  }
  
  if(HMI_flag.filament_runout_star) return;
  
  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_print.inc(3)) {
      switch (select_print.now) {
        case 0: ICON_Tune(); break;
        case 1:
          ICON_Tune();
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          break;
        case 2:
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          ICON_Stop();
          break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_print.dec()) {
      switch (select_print.now) {
        case 0:
          ICON_Tune();
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          break;
        case 1:
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          ICON_Stop();
          break;
        case 2: ICON_Stop(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_print.now) {
      case 0: // Tune
        checkkey = Tune;
				HMI_flag.Is_Mixer_Print = 1;
        HMI_ValueStruct.show_mode = 0;
        select_tune.reset();
        index_tune = MROWS;
        Draw_Tune_Menu();
				#if ENABLED(BABYSTEPPING)
				Babysteps_timer_first = millis();
				#endif
				
        break;
      case 1: // Pause
        if (HMI_flag.pause_flag) {
          ICON_Pause();

          char cmd[40];
          cmd[0] = '\0';

          #if ENABLED(PAUSE_HEAT)
            #if HAS_HEATED_BED
              if (tempbed) sprintf_P(cmd, PSTR("M190 S%i\n"), tempbed);
            #endif
            #if HAS_HOTEND
              if (temphot) sprintf_P(&cmd[strlen(cmd)], PSTR("M109 S%i\n"), temphot);
            #endif
          #endif

          strcat_P(cmd, PSTR("M24"));
          queue.inject(cmd);
        }
        else {
          HMI_flag.select_flag = true;
          checkkey = Print_window;
          Popup_window_PauseOrStop();
        }
        break;

      case 2: // Stop
        HMI_flag.select_flag = true;
        checkkey = Print_window;
        Popup_window_PauseOrStop();
        break;

      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Filament Runout Option window */
void HMI_Filament_Runout_Option() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_option.inc(2)) ICON_YESorNO(select_option.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_option.dec()) ICON_YESorNO(select_option.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_option.now) {
      case 0: // say yes
        FIL.Puge_More_Yes = 1;
		FIL.Puge_More_No = 0;
        break;
      case 1: // say no
        FIL.Puge_More_Yes = 1;
        FIL.Puge_More_No = 1;
	    checkkey = PrintProcess;
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Powerdown window */
void HMI_Powerdown() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_powerdown.inc(2)) ICON_YESorNO_Powerdown(select_powerdown.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_powerdown.dec()) ICON_YESorNO_Powerdown(select_powerdown.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_powerdown.now) {
      case 0: 
        checkkey = Prepare;
        select_prepare.set(PREPARE_CASE_POWERDOWN);
        index_prepare = MROWS;
        Draw_Prepare_Menu();
        break;
      case 1: 
        queue.inject_P(PSTR("M81"));
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Pause and Stop window */
void HMI_PauseOrStop() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW)
    Draw_Select_Highlight(false);
  else if (encoder_diffState == ENCODER_DIFF_CCW)
    Draw_Select_Highlight(true);
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (select_print.now == 1) { // pause window
      if (HMI_flag.select_flag) {
        HMI_flag.pause_action = true;
        ICON_Continue();
        #if ENABLED(POWER_LOSS_RECOVERY)
          if (recovery.enabled){
		  	recovery.save(true);
          }
        #endif
        queue.inject_P(PSTR("M25"));
      }
      else {
        // cancel pause
      }
      Goto_PrintProcess();
    }
    else if (select_print.now == 2) { // stop window
      if (HMI_flag.select_flag) {        
        planner.synchronize();													// Wait for planner moves to finish!
        checkkey = Back_Main;
        wait_for_heatup = wait_for_user = false;       // Stop waiting for heating/user
        card.flag.abort_sd_printing = true;            // Let the main loop handle SD abort
        dwin_abort_flag = true;                        // Reset feedrate, return to Home
        Percentrecord = 0;
				remain_time = 0;
        #ifdef ACTION_ON_CANCEL
          host_action_cancel();
        #endif
        Popup_Window_HomeAll(true);
      }
      else
        Goto_PrintProcess(); // cancel stop
    }
  }
  DWIN_UpdateLCD();
}

inline void Draw_Move_Menu() {
  Clear_Main_Window();

   #if AXISMOVE_CASE_TOTAL >= 6
    const int16_t scroll = MROWS - index_axismove; // Scrolled-up lines
    #define CSCROL(L) (scroll + (L))
  #else
    #define CSCROL(L) (L)
  #endif
  #define CLINE(L)  MBASE(CSCROL(L))
  #define CVISI(L)  WITHIN(CSCROL(L), 0, MROWS)

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Move], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Move, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);

  if (CVISI(0)) Draw_Back_First(select_axis.now == 0);                         // < Back
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, CLINE(AXISMOVE_CASE_MOVEX));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_X, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], CLINE(AXISMOVE_CASE_MOVEX));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, CLINE(AXISMOVE_CASE_MOVEY));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Y, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], CLINE(AXISMOVE_CASE_MOVEY));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, CLINE(AXISMOVE_CASE_MOVEZ));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Z, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], CLINE(AXISMOVE_CASE_MOVEZ));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, CLINE(AXISMOVE_CASE_EX1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_1, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], CLINE(AXISMOVE_CASE_EX1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, CLINE(AXISMOVE_CASE_EX2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_2, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], CLINE(AXISMOVE_CASE_EX2));
  
  if (select_axis.now && CVISI(select_axis.now))
    Draw_Menu_Cursor(CSCROL(select_axis.now));

  Draw_Menu_Line(1,ICON_MoveX);
  Draw_Menu_Line(2,ICON_MoveY);
  Draw_Menu_Line(3,ICON_MoveZ);
  Draw_Menu_Line(4,ICON_Extruder1);
  Draw_Menu_Line(5,ICON_Extruder2);

  HMI_ValueStruct.Move_E1_scale = HMI_ValueStruct.Move_E2_scale = HMI_ValueStruct.Move_E3_scale = HMI_ValueStruct.Move_E4_scale = HMI_ValueStruct.Move_EAll_scale= 0;
  HMI_flag.last_E1_Coordinate = HMI_flag.last_E2_Coordinate = HMI_flag.last_E3_Coordinate = HMI_flag.last_E4_Coordinate = HMI_flag.last_EALL_Coordinate = 0;
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, CLINE(AXISMOVE_CASE_EX1), HMI_ValueStruct.Move_E1_scale);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, CLINE(AXISMOVE_CASE_EX2), HMI_ValueStruct.Move_E2_scale);
}


/*
inline void Draw_Move_Menu() {
  Clear_Main_Window();

  if (HMI_IsChinese()) {
    DWIN_Frame_TitleCopy(1, 192, 1, 233, 14); // "Move"
    DWIN_Frame_AreaCopy(1, 58, 118, 106, 132, LBLX, MBASE(1));
    DWIN_Frame_AreaCopy(1, 109, 118, 157, 132, LBLX, MBASE(2));
    DWIN_Frame_AreaCopy(1, 160, 118, 209, 132, LBLX, MBASE(3));
    #if HAS_HOTEND
      DWIN_Frame_AreaCopy(1, 212, 118, 253, 131, LBLX, MBASE(4));
    #endif
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title(GET_TEXT_F(MSG_MOVE_AXIS));
    #else
      DWIN_Frame_TitleCopy(1, 231, 2, 265, 12);                     // "Move"
    #endif
    draw_move_en(MBASE(1)); say_x(36, MBASE(1));                    // "Move X"
    draw_move_en(MBASE(2)); say_y(36, MBASE(2));                    // "Move Y"
    draw_move_en(MBASE(3)); say_z(36, MBASE(3));                    // "Move Z"
    #if HAS_HOTEND
      DWIN_Frame_AreaCopy(1, 123, 192, 176, 202, LBLX, MBASE(4));   // "Extruder"
    #endif
  }

  Draw_Back_First(select_axis.now == 0);
  if (select_axis.now) Draw_Menu_Cursor(select_axis.now);

  // Draw separators and icons
  LOOP_L_N(i, 3 + ENABLED(HAS_HOTEND)) Draw_Menu_Line(i + 1, ICON_MoveX + i);
}
*/
#include "../../../libs/buzzer.h"
void HMI_AudioFeedback(const bool success=true) {
   if (success) {
    buzzer.tone(200, 1000);
    buzzer.tone(10, 0);
    buzzer.tone(200, 3000);
  }
  else
    buzzer.tone(20, 1000);
}

/* Prepare */
void HMI_Prepare() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_prepare.inc(1 + PREPARE_CASE_TOTAL)) {
      if (select_prepare.now > MROWS && select_prepare.now > index_prepare) {
        index_prepare = select_prepare.now;

        // Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_Menu_Icon(MROWS, ICON_Axis + select_prepare.now - 1);

        // Draw "More" icon for sub-menus
        if (index_prepare < 7) Draw_More_Icon(MROWS - index_prepare + 1);
		
	    #if BOTH(BABYSTEPPING,PREPARE_CASE_ZOFF)
			  if (index_prepare == PREPARE_CASE_ZOFF) Item_Prepare_Offset(MROWS);
			#endif
      #if HAS_HOTEND
			  if (index_prepare == PREPARE_CASE_PLA) Item_Prepare_PLA(MROWS);
        if (index_prepare == PREPARE_CASE_ABS) Item_Prepare_ABS(MROWS);
      #endif
      #if HAS_PREHEAT
        if (index_prepare == PREPARE_CASE_COOL) Item_Prepare_Cool(MROWS);
      #endif
        if (index_prepare == PREPARE_CASE_LANG) Item_Prepare_Lang(MROWS);
      }
      else {
        Move_Highlight(1, select_prepare.now + MROWS - index_prepare);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_prepare.dec()) {
      if (select_prepare.now < index_prepare - MROWS) {
        index_prepare--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

        if (index_prepare == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_Axis + select_prepare.now - 1);
        if (index_prepare < 7) Draw_More_Icon(MROWS - index_prepare + 1);
				
				if (index_prepare - MROWS == PREPARE_CASE_HOME) Item_Prepare_Home(0);
				#if HAS_HOTEND
        	 if (index_prepare - MROWS == PREPARE_CASE_MOVE) Item_Prepare_Move(0);
           if (index_prepare - MROWS == PREPARE_CASE_DISA) Item_Prepare_Disable(0);
				#endif
				#if HAS_PREHEAT
		     if (index_prepare - MROWS == PREPARE_CASE_LEVELING) Item_Prepare_Leveling(0);
				#endif
		     if (index_prepare - MROWS == PREPARE_CASE_POWERDOWN) Item_Prepare_Powerdown(0);				
				#if BOTH(BABYSTEPPING,PREPARE_CASE_ZOFF)
         if (index_prepare - MROWS == PREPARE_CASE_ZOFF) Item_Prepare_Offset(0);
				#endif
		     
      }
      else {
        Move_Highlight(-1, select_prepare.now + MROWS - index_prepare);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_prepare.now) {
      case 0: // Back
        select_page.set(1);
        Goto_MainMenu();
        break;
      case PREPARE_CASE_MOVE: // Axis move
        checkkey = AxisMove;
        select_axis.reset();
		    index_axismove = MROWS;
        Draw_Move_Menu();
				
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_MOVEX), current_position.x * MINUNITMULT);
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_MOVEY), current_position.y * MINUNITMULT);
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_MOVEZ), current_position.z * MINUNITMULT);
		#if HAS_HOTEND
	      queue.inject_P(PSTR("G92 E0"));
	      current_position.e = HMI_ValueStruct.Move_E1_scale = 0;
			  current_position.e = HMI_ValueStruct.Move_E2_scale = 0;
			  //current_position.e = HMI_ValueStruct.Move_E3_scale = 0;
			  //current_position.e = HMI_ValueStruct.Move_E4_scale = 0;
	      DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_EX1), 0);
			  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_EX2), 0);
	  #endif	
        break;
		
    case PREPARE_CASE_DISA: // Disable steppers
      queue.inject_P(PSTR("M84"));
      break;
	  
    case PREPARE_CASE_HOME: // Homing
    	checkkey = Home;
      index_home = MROWS;
	  	select_home.reset();
      Draw_Home_Menu();	     
      break;
		
	  case PREPARE_CASE_LEVELING: 		// Leveling
      checkkey = Leveling0;
      index_leveling = MROWS;
  		select_leveling.reset();
      Draw_Leveling_Menu();
      break;

	  case PREPARE_CASE_POWERDOWN: 		// Powerdown
      //queue.inject_P(PSTR("M81"));
      checkkey = Powerdown;
      Popup_window_Powerdown();
      break;
		
  #if BOTH(BABYSTEPPING,PREPARE_CASE_ZOFF)
    case PREPARE_CASE_ZOFF: // Z-offset
	    checkkey = Homeoffset;
      HMI_ValueStruct.show_mode = -4;      
      DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 2, 2, 202, MBASE(PREPARE_CASE_ZOFF + MROWS - index_prepare), HMI_ValueStruct.zoffset_value);
      EncoderRate.enabled = true;
      break;
  #endif
	  
  #if HAS_HOTEND
    case PREPARE_CASE_PLA: // PLA preheat
      thermalManager.setTargetHotend(ui.material_preset[0].hotend_temp, 0);
      thermalManager.setTargetBed(ui.material_preset[0].bed_temp);
      thermalManager.set_fan_speed(0, ui.material_preset[0].fan_speed);
      break;
    case PREPARE_CASE_ABS: // ABS preheat
      thermalManager.setTargetHotend(ui.material_preset[1].hotend_temp, 0);
      thermalManager.setTargetBed(ui.material_preset[1].bed_temp);
      thermalManager.set_fan_speed(0, ui.material_preset[1].fan_speed);
      break;
  #endif
			
  #if HAS_PREHEAT
    case PREPARE_CASE_COOL: // Cool
      TERN_(HAS_FAN, thermalManager.zero_fan_speeds());
      #if HAS_HOTEND || HAS_HEATED_BED
        thermalManager.disable_all_heaters();
      #endif
      break;
  #endif
			
    case PREPARE_CASE_LANG: // Toggle Language
      //HMI_ToggleLanguage();
      //Draw_Prepare_Menu();
      checkkey = Language;
      index_language = MROWS;
	  	select_language.reset();
      Draw_Language_Menu();
      break;
				
    default: break;
    }
  }
  DWIN_UpdateLCD();
}

void Draw_Temperature_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Temperature], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Temperature, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
#if HAS_HOTEND
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Hotend, Menu_Coordinate,LBLX, MBASE(TEMP_CASE_TEMP));
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Temp, Menu_Coordinate,LBLX+Hotend_X_Coordinate[HMI_flag.language], MBASE(TEMP_CASE_TEMP));
#endif
#if HAS_HEATED_BED
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Bed, Menu_Coordinate,LBLX, MBASE(TEMP_CASE_BED));
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Temp, Menu_Coordinate,LBLX+Bed_X_Coordinate[HMI_flag.language], MBASE(TEMP_CASE_BED));
#endif
#if HAS_FAN
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Fan_Speed, Menu_Coordinate,LBLX, MBASE(TEMP_CASE_FAN));
#endif
#if HAS_HOTEND
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_PLA, Menu_Coordinate,LBLX, MBASE(TEMP_CASE_PLA));
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Preheat, Menu_Coordinate,LBLX+36, MBASE(TEMP_CASE_PLA));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_ABS, Menu_Coordinate,LBLX, MBASE(TEMP_CASE_ABS));
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Preheat, Menu_Coordinate,LBLX+36, MBASE(TEMP_CASE_ABS));
#endif
  
  Draw_Back_First(select_temp.now == 0);
  if (select_temp.now) Draw_Menu_Cursor(select_temp.now);

  // Draw icons and lines
  uint8_t i = 0;
  #define _TMENU_ICON(N) Draw_Menu_Line(++i, ICON_SetEndTemp + (N) - 1)
  #if HAS_HOTEND
    _TMENU_ICON(TEMP_CASE_TEMP);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), thermalManager.temp_hotend[0].target);
  #endif
  #if HAS_HEATED_BED
    _TMENU_ICON(TEMP_CASE_BED);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), thermalManager.temp_bed.target);
  #endif
  #if HAS_FAN
    _TMENU_ICON(TEMP_CASE_FAN);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), thermalManager.fan_speed[0]);
  #endif
  #if HAS_HOTEND
    // PLA/ABS items have submenus
    _TMENU_ICON(TEMP_CASE_PLA);
    Draw_More_Icon(i);
    _TMENU_ICON(TEMP_CASE_ABS);
    Draw_More_Icon(i);
  #endif
}

/* Control */
void HMI_Control() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_control.inc(1 + CONTROL_CASE_TOTAL)) {
      if (select_control.now > MROWS && select_control.now > index_control) {
        index_control = select_control.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_Menu_Icon(MROWS, ICON_Temperature + select_control.now - 1);

		#if ENABLED(BLTOUCH)
			if(index_control == CONTROL_CASE_BLTOUCH) Item_Control_BLtouch(MROWS);
		#endif

		#if ENABLED(EEPROM_SETTINGS)
			if(index_control == CONTROL_CASE_SAVE) Item_Control_Save(MROWS);
			if(index_control == CONTROL_CASE_LOAD)	Item_Control_Load(MROWS);
        	if(index_control == CONTROL_CASE_RESET)	Item_Control_Reset(MROWS);
		#endif
		
			if (index_control == CONTROL_CASE_INFO) Item_Control_Info(MROWS);
      }
      else {
        Move_Highlight(1, select_control.now + MROWS - index_control);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_control.dec()) {
      if (select_control.now < index_control - MROWS) {
        index_control--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

        if (index_control == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_Temperature + select_control.now - 1);

		if (index_control - MROWS == CONTROL_CASE_CONFIG ) Item_Control_Config(0);
		if (index_control - MROWS == CONTROL_CASE_MIXER ) Item_Control_Mixer(0);
		if (index_control - MROWS == CONTROL_CASE_MOVE ) Item_Control_Motion(0);
		if (index_control - MROWS == CONTROL_CASE_TEMP ) Item_Control_Temp(0);
      }
      else {
        Move_Highlight(-1, select_control.now + MROWS - index_control);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_control.now) {
      case 0: // Back
        select_page.set(2);
        Goto_MainMenu();
        break;
      case CONTROL_CASE_TEMP: // Temperature
        checkkey = TemperatureID;
        HMI_ValueStruct.show_mode = -1;
        select_temp.reset();
        Draw_Temperature_Menu();
        break;
      case CONTROL_CASE_MOVE: // Motion
        checkkey = Motion;
        select_motion.reset();
        Draw_Motion_Menu();
        break;
	  case CONTROL_CASE_MIXER: // Mixer
        checkkey = DMixer;
				HMI_flag.Is_Mixer_Print = 0;
        select_mixer.reset();
        Draw_Mixer_Menu();
        break;

	  case CONTROL_CASE_CONFIG: // Config
        checkkey = Config;
        select_config.reset();
        Draw_Config_Menu();
        break;
		
	  #if ENABLED(BLTOUCH)
	  	case CONTROL_CASE_BLTOUCH: // Bltouch
        	checkkey = Bltouch;
        	select_bltouch.reset();
        	Draw_Bltouch_Menu();
        	break;
	  #endif
	  
    #if ENABLED(EEPROM_SETTINGS)
      case CONTROL_CASE_SAVE:  // Write EEPROM        
        HMI_AudioFeedback(settings.save());
	      break;
      case CONTROL_CASE_LOAD:  // Read EEPROM
        HMI_AudioFeedback(settings.load());
	      break;
      case CONTROL_CASE_RESET: // Reset EEPROM
        settings.reset();				
        HMI_AudioFeedback(settings.save());      
	  		checkkey = Control;
	      select_control.reset();
        index_control = MROWS;
	  		Draw_Control_Menu();
        break;
    #endif
      case CONTROL_CASE_INFO: // Info
        checkkey = Info;
        Draw_Info_Menu();
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Language */
void HMI_Language() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_language.inc(1 + LANGUAGE_CASE_TOTAL)) {
      if (select_language.now > MROWS && select_language.now > index_language) {
        index_language = select_language.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_Menu_Icon(MROWS, LANGUAGE_CASE_EN + select_language.now - 1);
		
		if(index_language == LANGUAGE_CASE_ZH) Item_Language_ZH(MROWS);
      }
      else {
        Move_Highlight(1, select_language.now + MROWS - index_language);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_language.dec()) {
      if (select_language.now < index_language - MROWS) {
        index_language--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

        if (index_language == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, LANGUAGE_CASE_EN + select_language.now - 1);

		  if(index_language == LANGUAGE_CASE_ZH) Item_Language_EN(0);
      }
      else {
        Move_Highlight(-1, select_language.now + MROWS - index_language);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_language.now) {
	  case 0: 										// Back
        checkkey = Prepare;
        select_prepare.set(PREPARE_CASE_LANG);
        Draw_Prepare_Menu();
        break;

	  case LANGUAGE_CASE_EN: 
	  case LANGUAGE_CASE_SP:
	  case LANGUAGE_CASE_RU:
	  	HMI_flag.Title_Menu_Backup = 7;
      HMI_flag.language = select_language.now - 1;
			DWIN_JPG_CacheToN(1,HMI_flag.language+1);
			HMI_AudioFeedback(settings.save());
			checkkey = Prepare;
      select_prepare.set(PREPARE_CASE_LANG);
      Draw_Prepare_Menu();
      break;		
			
	  case LANGUAGE_CASE_FR: 
	  case LANGUAGE_CASE_PO:
	  	HMI_flag.Title_Menu_Backup = 6;
      HMI_flag.language = select_language.now - 1;
			DWIN_JPG_CacheToN(1,HMI_flag.language+1);
			HMI_AudioFeedback(settings.save());
			checkkey = Prepare;
      select_prepare.set(PREPARE_CASE_LANG);
      Draw_Prepare_Menu();
		break; 	
		
	  case LANGUAGE_CASE_ZH: 
		break;
	  
	  default:break;
    } 
  }
  DWIN_UpdateLCD();
}



#if HAS_ONESTEP_LEVELING
  /* Leveling */
  void HMI_Leveling() {
    Popup_Window_Leveling();
    DWIN_UpdateLCD();
		queue.inject_P(PSTR("G28O\nG29"));
  }

#endif

/* Leveling0 */
char Level_Buf[200]={0};
constexpr uint16_t lfrb[4] = LEVEL_CORNERS_INSET_LFRB;
void HMI_Leveling0() {
	static bool leveling_was_active = false;	
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_leveling.inc(1 + HMI_flag.Leveling_Case_Total)) {
      if (select_leveling.now > MROWS && select_leveling.now > index_leveling) {
        index_leveling = select_leveling.now;

				// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_Menu_Icon(MROWS, ICON_Leveling_Point1 + select_leveling.now - 1);
		
			#if ENABLED(ABL_GRID)
			  #if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
			  if(index_leveling == LEVELING_CASE_CATCHOFFSET) Item_Leveling_Auto(MROWS);
				#endif
				if(index_leveling == LEVELING_CASE_SAVE) Item_Leveling_Save(MROWS);
			#endif
      }
      else {
        Move_Highlight(1, select_leveling.now + MROWS - index_leveling);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_leveling.dec()) {
      if (select_leveling.now < index_leveling - MROWS) {
        index_leveling--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

        if (index_leveling == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_Leveling_Point1 + select_leveling.now - 1);

				#if ENABLED(LCD_BED_LEVELING)
				if(index_leveling - MROWS == LEVELING_CASE_POINT1) Item_Leveling_Point1(0);
				#endif
      }
      else {
        Move_Highlight(-1, select_leveling.now + MROWS - index_leveling);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_leveling.now) {
	  case 0: 										// Back
        checkkey = Prepare;
		    Clear_Bottom_Area();
        select_prepare.set(PREPARE_CASE_LEVELING);
        Draw_Prepare_Menu();
				HMI_flag.need_home_flag = true;
        break;
		
	  case LEVELING_CASE_POINT1: 										
        checkkey = Leveling0;								
	    	Clear_Bottom_Area();			
				leveling_was_active = planner.leveling_active;
				set_bed_leveling_enabled(false);				
				ZERO(Level_Buf);
				if(HMI_flag.need_home_flag)
					sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,lfrb[0],lfrb[1],3000,0,500);
				else
					sprintf_P(Level_Buf,PSTR("G28O\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,lfrb[0],lfrb[1],3000,0,500);
				queue.inject_P(Level_Buf);
				planner.synchronize();
				HMI_flag.need_home_flag = false;
				set_bed_leveling_enabled(leveling_was_active);
		break;
				
	  case LEVELING_CASE_POINT2: 										
        checkkey = Leveling0;				
		    Clear_Bottom_Area();
				
				leveling_was_active = planner.leveling_active;
				set_bed_leveling_enabled(false);
				ZERO(Level_Buf);
				if(HMI_flag.need_home_flag)
					sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-lfrb[2],lfrb[1],3000,0,500);	
				else
					sprintf_P(Level_Buf,PSTR("G28O\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-lfrb[2],lfrb[1],3000,0,500);	
				queue.inject_P(Level_Buf);
				planner.synchronize();
				HMI_flag.need_home_flag = false;
				set_bed_leveling_enabled(leveling_was_active);
        break;
				
	  case LEVELING_CASE_POINT3: 										
        checkkey = Leveling0;				
		    Clear_Bottom_Area();				
				leveling_was_active = planner.leveling_active;
				set_bed_leveling_enabled(false);
				ZERO(Level_Buf);
				if(HMI_flag.need_home_flag)
					sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-lfrb[2],Y_BED_SIZE-lfrb[3],3000,0,500);
				else
					sprintf_P(Level_Buf,PSTR("G28O\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-lfrb[2],Y_BED_SIZE-lfrb[3],3000,0,500);
				queue.inject_P(Level_Buf);
				planner.synchronize();
				HMI_flag.need_home_flag = false;
				set_bed_leveling_enabled(leveling_was_active);
        break;
	  case LEVELING_CASE_POINT4: 										
        checkkey = Leveling0;								
		    Clear_Bottom_Area();
				
				leveling_was_active = planner.leveling_active;
				set_bed_leveling_enabled(false);
				ZERO(Level_Buf);
				if(HMI_flag.need_home_flag)
					sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,lfrb[0],Y_BED_SIZE-lfrb[3],3000,0,500);
				else
					sprintf_P(Level_Buf,PSTR("G28O\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,lfrb[0],Y_BED_SIZE-lfrb[3],3000,0,500);
				queue.inject_P(Level_Buf);
				planner.synchronize();
				HMI_flag.need_home_flag = false;
				set_bed_leveling_enabled(leveling_was_active);
        break;
		
	#if ENABLED(ABL_GRID)
		#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
  	case LEVELING_CASE_CATCHOFFSET:
			#if ENABLED(OPTION_BED_COATING)
				if((checkkey == Last_Level_CatchPop) || (coating_thickness < 2))
			#endif
				{
					checkkey = Last_Level_CatchOffset;					
					set_bed_leveling_enabled(false);
					Clear_Bottom_Area();
					Popup_Window_CatchOffset();
			    DWIN_G29_Show_Messge(G29_CATCH_START);					
					//if(HMI_flag.need_home_flag)
						queue.inject_P(PSTR("G28\nG29 N\n"));		
					//else
					//	queue.inject_P(PSTR("G28O\nG29 N\n"));		
				}
			#if ENABLED(OPTION_BED_COATING)
	   	 	else{
					checkkey = Last_Level_CatchPop;
					Popup_Remove_Glass();
	   	 	}
			#endif
				planner.synchronize();
				HMI_flag.need_home_flag = false;
			break;
		#endif
		
  	case LEVELING_CASE_SAVE: 										
	      checkkey = Last_Leveling;
				set_bed_leveling_enabled(false);
				Clear_Bottom_Area();
				Popup_Window_Leveling0();
				DWIN_G29_Show_Messge(G29_MESH_START);
				if(HMI_flag.need_home_flag)
					queue.inject_P(PSTR("G28\nG29\n"));
				else
					queue.inject_P(PSTR("G28O\nG29\n"));
				planner.synchronize();
				HMI_flag.need_home_flag = false;
			break;
  #endif
		
	  default:break;
    } 
  }
  DWIN_UpdateLCD();
}

/* Home */
void HMI_Home() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_home.inc(1 + HOME_CASE_TOTAL)) {
		Move_Highlight(1, select_home.now);
	    //for(uint8_t i=HOME_CASE_ALL; i<HOME_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
		//DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(select_home.now));
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_home.dec()) {
		Move_Highlight(-1, select_home.now);
		//for(uint8_t i=HOME_CASE_ALL; i<HOME_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
		//if(select_home.now) DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(select_home.now));
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_home.now) {
	  case 0: 										// Back
        checkkey = Prepare;
        select_leveling.set(PREPARE_CASE_HOME);
        Draw_Prepare_Menu();
        break;
	  case HOME_CASE_ALL: 										
        checkkey = Last_Prepare;
        index_home = MROWS;
        queue.inject_P(PSTR("G28")); 	// G28 will set home_flag
        Popup_Window_HomeAll();
				//TERN_(EEPROM_SETTINGS, settings.save());
        break;
	  case HOME_CASE_X: 										
        checkkey = Last_Prepare;
        index_home = MROWS;
				queue.inject_P(TERN(HOME_Y_BEFORE_X, PSTR("G28Y\nG28X"),PSTR("G28X")));
        Popup_Window_HomeX();
        break;
	  case HOME_CASE_Y: 										
        checkkey = Last_Prepare;
        index_home = MROWS;
				queue.inject_P(TERN(HOME_X_BEFORE_Y, PSTR("G28X\nG28Y"),PSTR("G28Y")));
        Popup_Window_HomeY();
        break;
	  case HOME_CASE_Z: 										
        checkkey = Last_Prepare;
        index_home = MROWS;
        queue.inject_P(PSTR("G28 Z0"));
        Popup_Window_HomeZ();
				//TERN_(EEPROM_SETTINGS, settings.save());
        break;
	  default:break;
    } 
  }
  DWIN_UpdateLCD();
}

/* Axis Move */
void HMI_AxisMove() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    // popup window resume
    if (HMI_flag.ETempTooLow_flag) {
      if (encoder_diffState == ENCODER_DIFF_ENTER) {
        HMI_flag.ETempTooLow_flag = false;
        current_position.e = HMI_ValueStruct.Move_E1_scale = HMI_ValueStruct.Move_E2_scale = HMI_ValueStruct.Move_E3_scale = HMI_ValueStruct.Move_E4_scale = HMI_ValueStruct.Move_EAll_scale= 0;
				checkkey = AxisMove;
        select_axis.reset();
	    	index_axismove = MROWS;
				Draw_Move_Menu();
        
        HMI_ValueStruct.Move_X_scale = current_position.x * MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(1), HMI_ValueStruct.Move_X_scale);
				HMI_ValueStruct.Move_Y_scale = current_position.y * MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(2), HMI_ValueStruct.Move_Y_scale);
				HMI_ValueStruct.Move_Z_scale = current_position.z * MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(3), HMI_ValueStruct.Move_Z_scale);
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(4), HMI_ValueStruct.Move_E1_scale);
				DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(5), HMI_ValueStruct.Move_E2_scale);
        DWIN_UpdateLCD();
      }
      return;
    }
  #endif

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
  	if (select_axis.inc(1 + AXISMOVE_CASE_TOTAL)) {
      if (select_axis.now > MROWS && select_axis.now > index_axismove) {
        index_axismove = select_axis.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_Menu_Icon(MROWS, ICON_MoveX + select_axis.now - 1);
	  		#if (AXISMOVE_CASE_EX1 > 5)
				if(index_axismove == AXISMOVE_CASE_EX1)	Item_Axis_MoveEX1(MROWS);
				#endif
				#if (AXISMOVE_CASE_EX2 > 5)
				if(index_axismove == AXISMOVE_CASE_EX2)	Item_Axis_MoveEX2(MROWS);
				#endif				
				#if (AXISMOVE_CASE_EX3 > 5)
				if(index_axismove == AXISMOVE_CASE_EX3)	Item_Axis_MoveEX3(MROWS);
				#endif
				#if (AXISMOVE_CASE_EX4 > 5)
				if (index_axismove == AXISMOVE_CASE_EX4) Item_Axis_MoveEX4(MROWS);
				#endif
				#if (AXISMOVE_CASE_EXALL > 5)
				if (index_axismove == AXISMOVE_CASE_EXALL) Item_Axis_MoveEXAll(MROWS);
				#endif
      }
      else {
        Move_Highlight(1, select_axis.now + MROWS - index_axismove);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
  	if (select_axis.dec()) {
      if (select_axis.now < index_axismove - MROWS) {
        index_axismove--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

		if (index_axismove == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_MoveX + select_axis.now - 1);

		if(index_axismove - MROWS == AXISMOVE_CASE_MOVEX) Item_Axis_MoveX(0);
        if(index_axismove - MROWS  == AXISMOVE_CASE_MOVEY) Item_Axis_MoveY(0);
		if(index_axismove - MROWS  == AXISMOVE_CASE_MOVEZ) Item_Axis_MoveZ(0);
		if(index_axismove - MROWS  == AXISMOVE_CASE_EX1) Item_Axis_MoveEX1(0);
      }
      else {
        Move_Highlight(-1, select_axis.now + MROWS - index_axismove);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_axis.now) {
      case 0: // Back
        checkkey = Prepare;
        select_prepare.set(PREPARE_CASE_MOVE);
        index_prepare = MROWS;
        Draw_Prepare_Menu();
        break;
      case AXISMOVE_CASE_MOVEX: // X axis move
        checkkey = Move_X;
        HMI_ValueStruct.Move_X_scale = current_position.x * MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
		    EncoderRate.enabled = true;
        break;
      case AXISMOVE_CASE_MOVEY: // Y axis move
        checkkey = Move_Y;
        HMI_ValueStruct.Move_Y_scale = current_position.y * MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
		    EncoderRate.enabled = true;
        break;
      case AXISMOVE_CASE_MOVEZ: // Z axis move
        checkkey = Move_Z;
        HMI_ValueStruct.Move_Z_scale = current_position.z * MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
		    EncoderRate.enabled = true;
        break;
		
      #if HAS_HOTEND
       case AXISMOVE_CASE_EX1: // Extruder1
            // window tips
        #ifdef PREVENT_COLD_EXTRUSION
          if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
            HMI_flag.ETempTooLow_flag = true;
            Popup_Window_ETempTooLow();
            DWIN_UpdateLCD();
            return;
          }
        #endif
      checkkey = Extruder1;
			queue.inject_P("G92 E0");
			current_position.e = HMI_flag.last_E1_Coordinate;
			HMI_ValueStruct.Move_E1_scale = current_position.e * MINUNITMULT;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX1), HMI_ValueStruct.Move_E1_scale);
			EncoderRate.enabled = true;
      break;

			#if(E_STEPPERS > 1)
			case AXISMOVE_CASE_EX2: // Extruder2
			// window tips
    	#ifdef PREVENT_COLD_EXTRUSION
			if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
				HMI_flag.ETempTooLow_flag = true;
				Popup_Window_ETempTooLow();
				DWIN_UpdateLCD();
				return;
			}
    	#endif
			checkkey = Extruder2;
			queue.inject_P("G92 E0");
			current_position.e = HMI_flag.last_E2_Coordinate;
			HMI_ValueStruct.Move_E2_scale = current_position.e * MINUNITMULT;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX2), HMI_ValueStruct.Move_E2_scale);
			EncoderRate.enabled = true;
			break;
			#endif
			
			#if(E_STEPPERS > 2)
			case AXISMOVE_CASE_EX3: // Extruder3
			// window tips
    	#ifdef PREVENT_COLD_EXTRUSION
			if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
				HMI_flag.ETempTooLow_flag = true;
				Popup_Window_ETempTooLow();
				DWIN_UpdateLCD();
				return;
			}
    	#endif
			checkkey = Extruder3;
			queue.inject_P("G92 E0");
			current_position.e = HMI_flag.last_E3_Coordinate;
			HMI_ValueStruct.Move_E3_scale = current_position.e * MINUNITMULT;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX3), HMI_ValueStruct.Move_E3_scale);
			EncoderRate.enabled = true;
			break;
			#endif
				
			#if(E_STEPPERS > 3)
	 		case AXISMOVE_CASE_EX4: // Extruder4
    	// window tips
    	#ifdef PREVENT_COLD_EXTRUSION
    	if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
      	HMI_flag.ETempTooLow_flag = true;
      	Popup_Window_ETempTooLow();
      	DWIN_UpdateLCD();
      	return;
    	}
    	#endif
    	checkkey = Extruder4;
    	queue.inject_P("G92 E0");
			current_position.e = HMI_flag.last_E4_Coordinate;
			HMI_ValueStruct.Move_E4_scale = current_position.e * MINUNITMULT;
			DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX4), HMI_ValueStruct.Move_E4_scale);
    	EncoderRate.enabled = true;
    	break;
		  #endif
			
		#if ENABLED(MIXING_EXTRUDER)
			case AXISMOVE_CASE_EXALL: // Extruderall
				// window tips
				#ifdef PREVENT_COLD_EXTRUSION
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					DWIN_UpdateLCD();
					return;
				}
				#endif
				checkkey = ExtruderAll;
				queue.inject_P("G92 E0");
				current_position.e = HMI_flag.last_EALL_Coordinate;
				HMI_ValueStruct.Move_EAll_scale = current_position.e * MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EXALL), HMI_ValueStruct.Move_EAll_scale);
				EncoderRate.enabled = true;
	    	break;
   #endif //end if enable(MIXING_EXTRUDER)
	 #endif //end HAS_HOTEND
    }
  }
  DWIN_UpdateLCD();
}

/* TemperatureID */
void HMI_Temperature() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_temp.inc(1 + TEMP_CASE_TOTAL)) Move_Highlight(1, select_temp.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_temp.dec()) Move_Highlight(-1, select_temp.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_temp.now) {
      case 0: // Back
        checkkey = Control;
        select_control.set(1);
        index_control = MROWS;
        Draw_Control_Menu();
        break;
      #if HAS_HOTEND
        case TEMP_CASE_TEMP: // Nozzle temperature
          checkkey = ETemp;
          HMI_ValueStruct.E_Temp = thermalManager.temp_hotend[0].target;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(1), thermalManager.temp_hotend[0].target);
          EncoderRate.enabled = true;
          break;
      #endif
      #if HAS_HEATED_BED
        case TEMP_CASE_BED: // Bed temperature
          checkkey = BedTemp;
          HMI_ValueStruct.Bed_Temp = thermalManager.temp_bed.target;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(2), thermalManager.temp_bed.target);
          EncoderRate.enabled = true;
          break;
      #endif
      #if HAS_FAN
        case TEMP_CASE_FAN: // Fan speed
          checkkey = FanSpeed;
          HMI_ValueStruct.Fan_speed = thermalManager.fan_speed[0];
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(3), thermalManager.fan_speed[0]);
          EncoderRate.enabled = true;
          break;
      #endif
      #if HAS_HOTEND
        case TEMP_CASE_PLA: { // PLA preheat setting
          checkkey = PLAPreheat;
          select_PLA.reset();
          HMI_ValueStruct.show_mode = -2;

          Clear_Main_Window();

				  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_PLA], 14, 7);
				  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
				  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_PLA, Menu_Coordinate,14, 7);
				  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
		  		  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Nozzle, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_TEMP));
				  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Temp, Menu_Coordinate,LBLX+58, MBASE(PREHEAT_CASE_TEMP));
				  #if HAS_HEATED_BED
				   	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Bed, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_BED));
				  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Temp, Menu_Coordinate,LBLX+33, MBASE(PREHEAT_CASE_BED));
				  #endif
				  #if HAS_FAN
				  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Fan_Speed, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_FAN));
				  #endif
				  #if ENABLED(EEPROM_SETTINGS)
				    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Store, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_SAVE));
				  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Settings, Menu_Coordinate,LBLX+50, MBASE(PREHEAT_CASE_SAVE));
				  #endif
          
          Draw_Back_First();

          uint8_t i = 0;
          Draw_Menu_Line(++i, ICON_SetEndTemp);
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[0].hotend_temp);
          #if HAS_HEATED_BED
            Draw_Menu_Line(++i, ICON_SetBedTemp);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[0].bed_temp);
          #endif
          #if HAS_FAN
            Draw_Menu_Line(++i, ICON_FanSpeed);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[0].fan_speed);
          #endif
          #if ENABLED(EEPROM_SETTINGS)
            Draw_Menu_Line(++i, ICON_WriteEEPROM);
          #endif
        } break;

        case TEMP_CASE_ABS: { // ABS preheat setting
          checkkey = ABSPreheat;
          select_ABS.reset();
          HMI_ValueStruct.show_mode = -3;

          Clear_Main_Window();

				  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_ABS], 14, 7);
				  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
				  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_ABS, Menu_Coordinate,14, 7);
				  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
		  		  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Nozzle, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_TEMP));
				  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Temp, Menu_Coordinate,LBLX+58, MBASE(PREHEAT_CASE_TEMP));
				  #if HAS_HEATED_BED
				   	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Bed, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_BED));
				  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Temp, Menu_Coordinate,LBLX+33, MBASE(PREHEAT_CASE_BED));
				  #endif
				  #if HAS_FAN
				  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Fan_Speed, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_FAN));
				  #endif
				  #if ENABLED(EEPROM_SETTINGS)
				    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Store, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_SAVE));
				  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Settings, Menu_Coordinate,LBLX+50, MBASE(PREHEAT_CASE_SAVE));
				  #endif
          
          Draw_Back_First();

          uint8_t i = 0;
          Draw_Menu_Line(++i, ICON_SetEndTemp);
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[1].hotend_temp);
          #if HAS_HEATED_BED
            Draw_Menu_Line(++i, ICON_SetBedTemp);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[1].bed_temp);
          #endif
          #if HAS_FAN
            Draw_Menu_Line(++i, ICON_FanSpeed);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[1].fan_speed);
          #endif
          #if ENABLED(EEPROM_SETTINGS)
            Draw_Menu_Line(++i, ICON_WriteEEPROM);
          #endif

        } break;

      #endif // HAS_HOTEND
    }
  }
  DWIN_UpdateLCD();
}

inline void Draw_Max_Speed_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_FEEDRATE], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_FEEDRATE, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Max, Menu_Coordinate,LBLX, MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Feedrate, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_X, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Max, Menu_Coordinate,LBLX, MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Feedrate, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Y, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Max, Menu_Coordinate,LBLX, MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Feedrate, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Z, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Max, Menu_Coordinate,LBLX, MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Feedrate, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_E, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(4));
  
  Draw_Back_First();
  LOOP_L_N(i, 3 + ENABLED(HAS_HOTEND)) Draw_Menu_Line(i + 1, ICON_MaxSpeedX + i);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(1), planner.settings.max_feedrate_mm_s[X_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(2), planner.settings.max_feedrate_mm_s[Y_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(3), planner.settings.max_feedrate_mm_s[Z_AXIS]);
  #if HAS_HOTEND
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(4), planner.settings.max_feedrate_mm_s[E_AXIS]);
  #endif
}

inline void Draw_Max_Accel_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_ACCEL], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_ACCEL, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Max, Menu_Coordinate,LBLX, MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Accel, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_X, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Max, Menu_Coordinate,LBLX, MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Accel, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Y, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Max, Menu_Coordinate,LBLX, MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Accel, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Z, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Max, Menu_Coordinate,LBLX, MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Accel, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_E, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language], MBASE(4));
  
  Draw_Back_First();
  LOOP_L_N(i, 3 + ENABLED(HAS_HOTEND)) Draw_Menu_Line(i + 1, ICON_MaxAccX + i);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 5, 210, MBASE(1), planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 5, 210, MBASE(2), planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 5, 210, MBASE(3), planner.settings.max_acceleration_mm_per_s2[Z_AXIS]);
  #if HAS_HOTEND
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 5, 210, MBASE(4), planner.settings.max_acceleration_mm_per_s2[E_AXIS]);
  #endif
}

inline void Draw_Max_Jerk_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_JERK], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_JERK, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Max, Menu_Coordinate,LBLX, MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Jerk, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_X, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Max, Menu_Coordinate,LBLX, MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Jerk, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Y, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Max, Menu_Coordinate,LBLX, MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Jerk, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Z, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Max, Menu_Coordinate,LBLX, MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Jerk, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_E, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language], MBASE(4));
  
  Draw_Back_First();
  LOOP_L_N(i, 3 + ENABLED(HAS_HOTEND)) Draw_Menu_Line(i + 1, ICON_MaxSpeedJerkX + i);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 1, 210, MBASE(1), planner.max_jerk[X_AXIS] * MINUNITMULT);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 1, 210, MBASE(2), planner.max_jerk[Y_AXIS] * MINUNITMULT);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 1, 210, MBASE(3), planner.max_jerk[Z_AXIS] * MINUNITMULT);
  #if HAS_HOTEND
    DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 2, 1, 210, MBASE(4), planner.max_jerk[E_AXIS] * MINUNITMULT);
  #endif
}

inline void Draw_Steps_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_STEP], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_STEP, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Max, Menu_Coordinate,LBLX, MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Step, Menu_Coordinate,LBLX, MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_X, Menu_Coordinate,LBLX+Step_X_Coordinate[HMI_flag.language], MBASE(1));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Max, Menu_Coordinate,LBLX, MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Step, Menu_Coordinate,LBLX, MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Y, Menu_Coordinate,LBLX+Step_X_Coordinate[HMI_flag.language], MBASE(2));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Max, Menu_Coordinate,LBLX, MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Step, Menu_Coordinate,LBLX, MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Z, Menu_Coordinate,LBLX+Step_X_Coordinate[HMI_flag.language], MBASE(3));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Max, Menu_Coordinate,LBLX, MBASE(4));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Step, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(4));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_E, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Step_X_Coordinate[HMI_flag.language], MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Step, Menu_Coordinate,LBLX, MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_E, Menu_Coordinate,LBLX+Step_X_Coordinate[HMI_flag.language], MBASE(4));
  
  Draw_Back_First();
  LOOP_L_N(i, 3 + ENABLED(HAS_HOTEND)) Draw_Menu_Line(i + 1, ICON_StepX + i);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 4, 1, 210, MBASE(1), planner.settings.axis_steps_per_mm[X_AXIS] * MINUNITMULT);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 4, 1, 210, MBASE(2), planner.settings.axis_steps_per_mm[Y_AXIS] * MINUNITMULT);
  DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 4, 1, 210, MBASE(3), planner.settings.axis_steps_per_mm[Z_AXIS] * MINUNITMULT);
  #if HAS_HOTEND
    DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 4, 1, 210, MBASE(4), planner.settings.axis_steps_per_mm[E_AXIS] * MINUNITMULT);
  #endif
}

/* Motion */
void HMI_Motion() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_motion.inc(1 + MOTION_CASE_TOTAL)) Move_Highlight(1, select_motion.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_motion.dec()) Move_Highlight(-1, select_motion.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_motion.now) {
      case 0: // Back
        checkkey = Control;
        select_control.set(CONTROL_CASE_MOVE);
        index_control = MROWS;
        Draw_Control_Menu();
        break;
      case MOTION_CASE_RATE:   // Max speed
        checkkey = MaxSpeed;
        select_speed.reset();
        Draw_Max_Speed_Menu();
        break;
      case MOTION_CASE_ACCEL:  // Max acceleration
        checkkey = MaxAcceleration;
        select_acc.reset();
        Draw_Max_Accel_Menu();
        break;
      #if HAS_CLASSIC_JERK
        case MOTION_CASE_JERK: // Max jerk
          checkkey = MaxJerk;
          select_jerk.reset();
          Draw_Max_Jerk_Menu();
         break;
      #endif
      case MOTION_CASE_STEPS:  // Steps per mm
        checkkey = Step;
        select_step.reset();
        Draw_Steps_Menu();
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}


/* Bltouch */
#if ENABLED(BLTOUCH)
void HMI_Option_Bltouch() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_bltouch.inc(1 + BLTOUCH_CASE_TOTAL)) {
		Move_Highlight(1, select_bltouch.now);
		//for(uint8_t i=BLTOUCH_CASE_RESET; i<BLTOUCH_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
			//DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(select_bltouch.now));
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_bltouch.dec()) {
		Move_Highlight(-1, select_bltouch.now);
		//for(uint8_t i=BLTOUCH_CASE_RESET; i<BLTOUCH_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
		//if(select_bltouch.now) DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(select_bltouch.now));
    }
	
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_bltouch.now) {
	  case 0: 					// Back
        checkkey = Control;
        select_control.set(CONTROL_CASE_BLTOUCH);
        index_control = MROWS;
        Draw_Control_Menu();
        break;

      case BLTOUCH_CASE_RESET: 	// Reset
        checkkey = Bltouch;
				bltouch._reset();
        break;

	  case BLTOUCH_CASE_TEST: 	// Test
        checkkey = Bltouch;
				bltouch._selftest();
        break;

	  case BLTOUCH_CASE_STOW: 	// Stow
        checkkey = Bltouch;
				bltouch._stow();
        break;
	  
	  case BLTOUCH_CASE_DEPLOY: 	// Proc
        checkkey = Bltouch;
				bltouch._deploy();
        break;
	  case BLTOUCH_CASE_SW: 	// sw
        checkkey = Bltouch;
				bltouch._set_SW_mode();
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}
#endif

#if ENABLED(OPTION_WIFI_MODULE)
/* wifi */
void HMI_Wifi() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_ENTER) {
		if(select_page.now == 1) 
			select_page.set(1);
		else 
			select_page.set(0);
		checkkey = MainMenu;
		Goto_MainMenu();
  }
  DWIN_UpdateLCD();
}
#endif

/* DMixer */
void HMI_Mixer() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_mixer.inc(1 + MIXER_CASE_TOTAL)) Move_Highlight(1, select_mixer.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_mixer.dec()) Move_Highlight(-1, select_mixer.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_mixer.now) {
	  case 0: 																// Back
	    if(!HMI_flag.Is_Mixer_Print){
        	checkkey = Control;
        	select_control.set(CONTROL_CASE_MIXER);
        	index_control = MROWS;
        	Draw_Control_Menu();
	  	}
		else{
			checkkey = Tune;
        	HMI_ValueStruct.show_mode = 0;
			select_tune.set(TUNE_CASE_SPEED);
        	index_tune = MROWS;
        	Draw_Tune_Menu();
		}
        break;

      case MIXER_CASE_MANUAL: 	// Manual
        checkkey = Mix_Manual;
        select_manual.reset();
	    MixerCfg.Vtool_Backup  = mixer.selected_vtool;
	  	updata_mixer_from_vtool();
		MIXER_STEPPER_LOOP(i) {MixerCfg.Manual_Percent[mixer.selected_vtool][i] = mixer.mix[i];}
		index_manual = MROWS;
		MixerCfg.Mixer_Mode_Rg = 0;
		#if ENABLED(POWER_LOSS_RECOVERY)
		recovery.save(true);
		#endif
		Draw_Mixer_Manual_Menu();
        break;
      case MIXER_CASE_AUTO:   	// Auto
        checkkey = Mix_Auto;
        select_auto.reset();
        
		mixer.selected_vtool = mixer.gradient.start_vtool;     
		updata_mixer_from_vtool();
		MIXER_STEPPER_LOOP(i) {MixerCfg.Start_Percent[i] = mixer.mix[i];}
		mixer.selected_vtool = mixer.gradient.end_vtool;     
		updata_mixer_from_vtool();
		MIXER_STEPPER_LOOP(i) {MixerCfg.End_Percent[i] = mixer.mix[i];}
	    index_auto = MROWS;
		MixerCfg.Mixer_Mode_Rg = 1;
		#if ENABLED(POWER_LOSS_RECOVERY)
		recovery.save(true);
		#endif
		Draw_Mixer_Auto_Menu();
		//Draw_Print_ProgressModel();
        break;
      case MIXER_CASE_RANDOM:  	// Random
        checkkey = Mix_Random;
        select_random.reset();
	    index_random = MROWS;
		MixerCfg.Mixer_Mode_Rg = 2;
		#if ENABLED(POWER_LOSS_RECOVERY)
		recovery.save(true);
		#endif
		Draw_Mixer_Random_Menu();
		break;
	  case MIXER_CASE_VTOOL:  	// vtool
        checkkey = Mix_Vtool;
        DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
	  	EncoderRate.enabled = true;
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Config */
void HMI_Config() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
		if(IS_SD_PRINTING() || IS_SD_PAUSED()){
			if(select_config.inc(1 + CONFIG_TUNE_CASE_TOTAL)){
	      if (select_config.now > MROWS && select_config.now > index_config) {
	        index_config = select_config.now;
					// Scroll up and draw a blank bottom line
	        Scroll_Menu(DWIN_SCROLL_UP);
					//Draw_Menu_Icon(MROWS, ICON_Cursor+ select_config.now - 1);		
	      }
	      else 
					Move_Highlight(1, select_config.now + MROWS - index_config);
	  	}
		}
  	else if(select_config.inc(1 + CONFIG_CASE_TOTAL)){
      if (select_config.now > MROWS && select_config.now > index_config) {
        index_config = select_config.now;
				// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
				//Draw_Menu_Icon(MROWS, ICON_Cursor+ select_config.now - 1);
	
				#if ENABLED(OPTION_REPEAT_PRINTING) 
				if(index_config == CONFIG_CASE_REPRINT) Item_Config_Reprint(MROWS);
				#endif

				#if ENABLED(OPTION_BED_COATING) 
				if(index_config == CONFIG_CASE_COATING) Item_Config_bedcoating(MROWS);
				#endif

				#if ENABLED(ABL_GRID)
				if(index_config == CONFIG_CASE_LEVELING) Item_Config_Leveling(MROWS);
				if(index_config == CONFIG_CASE_ACTIVELEVEL) Item_Config_ActiveLevel(MROWS);
				#endif
     
				#if ENABLED(HAS_COLOR_LEDS)
				if(index_config == CONFIG_CASE_RGB) Item_Config_RGB(MROWS);
				#endif

				#if ENABLED(DEBUG_GCODE_M92)
				if(index_config == CONFIG_CASE_M92) Item_Config_M92(MROWS);
				#endif
      }
      else Move_Highlight(1, select_config.now + MROWS - index_config);
  	}
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
  	if (select_config.dec()) {
      if (select_config.now < index_config - MROWS) {
        index_config--;
        Scroll_Menu(DWIN_SCROLL_DOWN);
				if (index_config == MROWS) Draw_Back_First();
				
				#if ENABLED(FWRETRACT) 
				if(index_config - MROWS == CONFIG_CASE_RETRACT) Item_Config_Retract(0);
				#endif
     
				#if ENABLED(FILAMENT_RUNOUT_SENSOR)
				if(index_config - MROWS == CONFIG_CASE_FILAMENT) Item_Config_Filament(0);
				#endif

				#if ENABLED(POWER_LOSS_RECOVERY)
				if(index_config - MROWS == CONFIG_CASE_POWERLOSS) Item_Config_Powerloss(0);
				#endif

				#if ENABLED(OPTION_AUTOPOWEROFF)
				if(index_config - MROWS == CONFIG_CASE_SHUTDOWN) Item_Config_Shutdown(0);
				#endif				
				if(!IS_SD_PRINTING() && !IS_SD_PAUSED()){
					#if ENABLED(OPTION_WIFI_MODULE)
					if(index_config - MROWS == CONFIG_CASE_WIFI) Item_Config_Wifi(0);
					#endif
					#if ENABLED(ABL_GRID)
					if(index_config - MROWS == CONFIG_CASE_LEVELING) Item_Config_Leveling(0);
					if(index_config - MROWS == CONFIG_CASE_ACTIVELEVEL) Item_Config_ActiveLevel(0);
					#endif
				}
      }
      else {
		  	Move_Highlight(-1, select_config.now + MROWS - index_config);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_config.now) {
	  case 0: 									// Back
			if(!IS_SD_PRINTING() && !IS_SD_PAUSED()){
      	checkkey = Control;
      	select_control.set(CONTROL_CASE_CONFIG);
      	index_control = MROWS;
      	Draw_Control_Menu();
	  	}
			else{
				checkkey = Tune;
      	HMI_ValueStruct.show_mode = 0;
				select_tune.set(TUNE_CASE_SPEED);
      	index_tune = MROWS;
      	Draw_Tune_Menu();
			}
    break;

  #if ENABLED(FWRETRACT) 
    case CONFIG_CASE_RETRACT: 				// RETRACT
      checkkey = Retract;
			index_retract = MROWS;
      Draw_Retract_Menu();
    break;
  #endif
	  
  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    case CONFIG_CASE_FILAMENT:   				// FILAMENT
      checkkey = Config;
			if(runout.enabled){
				runout.enabled = 0;
				queue.inject_P("M412 S0");
				settings.save();
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_FILAMENT + MROWS - index_config), F("OFF"));
			}
			else{
				runout.enabled = 1;
				queue.inject_P("M412 S1");
				settings.save();
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_FILAMENT + MROWS - index_config), F("ON "));
			}
      break;
  #endif
	  
  #if ENABLED(POWER_LOSS_RECOVERY)
	  case CONFIG_CASE_POWERLOSS:   			// POWERLOSS
			checkkey = Config;
			if(recovery.enabled){
				recovery.enabled = 0;
				queue.inject_P("M413 S0");
				settings.save();
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_POWERLOSS + MROWS - index_config), F("OFF"));
			}
			else{
				recovery.enabled = 1;
				queue.inject_P("M413 S1");
				settings.save();
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_POWERLOSS + MROWS - index_config), F("ON "));
			}
    break;
  #endif
		
  #if ENABLED(OPTION_AUTOPOWEROFF)
	  case CONFIG_CASE_SHUTDOWN:
	  	checkkey = Config;
			HMI_flag.auto_shutdown = !HMI_flag.auto_shutdown;
	    if(HMI_flag.auto_shutdown) DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_SHUTDOWN + MROWS - index_config), F("ON "));
			else DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_SHUTDOWN + MROWS - index_config), F("OFF "));
		break;
  #endif
     
  #if ENABLED(OPTION_WIFI_MODULE)
	  case CONFIG_CASE_WIFI:
			if(IS_SD_PRINTING() || IS_SD_PAUSED())	break;
			
			WiFi_Enabled = !WiFi_Enabled;		
	    if(WiFi_Enabled) {
				checkkey = Popup_Window;
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_WIFI + MROWS - index_config), F("ON "));				
				HMI_flag.wifi_Handshake_ok = false;
				HMI_flag.wifi_link_timer = 0;
				Popup_window_Wifi_Connect();
				WIFI_onoff();
	    }
			else {
				checkkey = Config;
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_WIFI + MROWS - index_config), F("OFF "));
				WIFI_onoff();
			}
			settings.save();
		break;
  #endif
	 
  #if HAS_COLOR_LEDS
  	case CONFIG_CASE_RGB:
			checkkey = Config;
			HMI_flag.RGB_LED_Counter++;
	    if(HMI_flag.RGB_LED_Counter > BLACK_ON) HMI_flag.RGB_LED_Counter = 0;
			RGB_LED_Light(HMI_flag.RGB_LED_Counter);
	    switch(HMI_flag.RGB_LED_Counter) {
				case RED_ON:DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(CONFIG_CASE_RGB + MROWS - index_config), F("RED   "));break;
				case GREEN_ON:DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(CONFIG_CASE_RGB + MROWS - index_config), F("GREEN "));break;
				case BLUE_ON:DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(CONFIG_CASE_RGB + MROWS - index_config), F("BLUE  "));break;
				case YELLOW_ON:DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(CONFIG_CASE_RGB + MROWS - index_config), F("YELLOW"));break;
				case CYAN_ON:DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(CONFIG_CASE_RGB + MROWS - index_config), F("CYAN  "));break;
				case PURPLE_ON:DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(CONFIG_CASE_RGB + MROWS - index_config), F("PURPLE"));break;
				case WHITE_ON:DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(CONFIG_CASE_RGB + MROWS - index_config), F("WHITE "));break;
				case LOOP_ON:DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(CONFIG_CASE_RGB + MROWS - index_config), F("SKIP  "));	break;
				default:
					HMI_flag.RGB_LED_Loop = 0;
					DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 216, MBASE(5), F("BLACK "));
					break;
				}
		break;
  #endif
		
  #if ENABLED(OPTION_REPEAT_PRINTING)
  	case CONFIG_CASE_REPRINT:
			if(IS_SD_PRINTING() || IS_SD_PAUSED())	break;
			
			checkkey = Re_print;
			index_reprint = MROWS;
    	Draw_Reprint_Menu();
		break;
  #endif

  #if ENABLED(OPTION_BED_COATING)
  	case CONFIG_CASE_COATING:
			if(IS_SD_PRINTING() || IS_SD_PAUSED())	break;
			
			checkkey = COATING;	
			HMI_ValueStruct.coating_thickness = (int16_t)(coating_thickness * 100);
			DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 2, 2, 216, MBASE(MROWS -index_config + CONFIG_CASE_COATING), HMI_ValueStruct.coating_thickness);			
		break;
  #endif

  #if ENABLED(ABL_GRID)
    case CONFIG_CASE_LEVELING:
			if(IS_SD_PRINTING() || IS_SD_PAUSED()) 	break;
			
			checkkey = Config;
			if(HMI_flag.Auto_Leveling_Menu_Fg){
				HMI_flag.Auto_Leveling_Menu_Fg = false;
				set_bed_leveling_enabled(false);
			}
			else
				HMI_flag.Auto_Leveling_Menu_Fg = true;
			
    	if(HMI_flag.Auto_Leveling_Menu_Fg){
				HMI_flag.Leveling_Case_Total = 5 + ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET);
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_LEVELING + MROWS - index_config), F("ON "));
    	}
			else {
				HMI_flag.Leveling_Case_Total = 4;
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_LEVELING + MROWS - index_config), F("OFF "));
			}
			settings.save();
		break;

		case CONFIG_CASE_ACTIVELEVEL:
			if(IS_SD_PRINTING() || IS_SD_PAUSED()) 	break;
			
			checkkey = Config;			
			planner.leveling_active = !planner.leveling_active;
			if(planner.leveling_active) 
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_ACTIVELEVEL + MROWS - index_config), F("ON "));
			else
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(CONFIG_CASE_ACTIVELEVEL + MROWS - index_config), F("OFF "));			
			set_bed_leveling_enabled(planner.leveling_active);
		break;
  #endif

  #if ENABLED(DEBUG_GCODE_M92)
  	case CONFIG_CASE_M92:
			if(IS_SD_PRINTING() || IS_SD_PAUSED()) 	break;
			
			checkkey = Config;
    	queue.inject_P(PSTR("M92"));
		break;
  #endif
      
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Reprint*/
#if ENABLED(OPTION_REPEAT_PRINTING) 
void HMI_Reprint() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
  	if (select_reprint.inc(1 + REPRINT_CASE_TOTAL)) {
     if (select_reprint.now > MROWS && select_reprint.now > index_reprint) {
        index_reprint = select_reprint.now;
				// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
				if(index_reprint == REPRINT_CASE_BACK) Item_Reprint_Back(MROWS);
			}
      else {
        Move_Highlight(1, select_reprint.now + MROWS - index_reprint);
      }
  	}
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_reprint.dec()) {
      if (select_reprint.now < index_reprint - MROWS) {
        index_reprint--;
        Scroll_Menu(DWIN_SCROLL_DOWN);
				if (index_reprint == MROWS) Draw_Back_First();       
				if(index_reprint - MROWS == REPRINT_CASE_ENABLED) Item_Reprint_Enabled(0);
      }
      else {
        Move_Highlight(-1, select_reprint.now + MROWS - index_reprint);
      }
     }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_reprint.now) {
	  case 0: 										// Back
		  	ReprintManager.Back_Move_Stop();
				checkkey = Config;
        select_config.set(CONFIG_CASE_REPRINT);
        Draw_Config_Menu();
        break;

      case REPRINT_CASE_ENABLED: 					// ENABLED
        checkkey = Re_print;
        if(ReprintManager.enabled){
					ReprintManager.enabled = 0;
					queue.inject_P("M180 S0");
					DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(MROWS -index_reprint + REPRINT_CASE_ENABLED), F("OFF"));
				}
				else{
					ReprintManager.enabled = 1;
					queue.inject_P("M180 S1");
					DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(MROWS -index_reprint + REPRINT_CASE_ENABLED), F("ON "));
				}
        break;
	  
      case REPRINT_CASE_TIMES:   					// reprint times
        checkkey = Reprint_times;
				DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 4, 216, MBASE(MROWS -index_reprint + REPRINT_CASE_TIMES), ReprintManager.Reprint_times);
		    EncoderRate.enabled = true;
        break;

	  case REPRINT_CASE_LENGHT:   					// FORWARD MOVE LENGHT
        checkkey = Forward_lenght;
        DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 4, 216, MBASE(MROWS -index_reprint + REPRINT_CASE_LENGHT), ReprintManager.Forward_lenght);
		    EncoderRate.enabled = true;
        break;

	  case REPRINT_CASE_RESET:   					// reset
        checkkey = Re_print;
        ReprintManager.Back_Move_Start();
		    ReprintManager.Is_Reprint_Reset = 1;
				ReprintManager.tempbed_counter = 0;
        break;

	  case REPRINT_CASE_FORWARD:   					// forward
        checkkey = Re_print;
				ReprintManager.Forward_Move_Start();
        break;

	  case REPRINT_CASE_BACK:   					// BACK
        checkkey = Re_print;
				ReprintManager.Back_Move_Start();
				break;
		
      default: break;
    }
  }
  DWIN_UpdateLCD();
}
#endif

/* Retract */
#if ENABLED(FWRETRACT) 
void HMI_Retract() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
  	if (select_retract.inc(1 + RETRACT_CASE_TOTAL)) {
     if (select_retract.now > MROWS && select_retract.now > index_retract) {
        index_retract = select_retract.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);

      }
      else {
        Move_Highlight(1, select_retract.now + MROWS - index_retract);
      }
  	}
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_retract.dec()) {
      if (select_retract.now < index_retract - MROWS) {
        index_retract--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

      }
      else {
        Move_Highlight(-1, select_retract.now + MROWS - index_retract);
      }
     }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_retract.now) {
	  case 0: 									// Back
      checkkey = Config;
      select_config.set(CONFIG_CASE_RETRACT);
      index_config = MROWS;
      Draw_Config_Menu();
      break;

    case RETRACT_CASE_AUTO: 					// auto
      checkkey = Retract;
      if(fwretract.autoretract_enabled){
				fwretract.autoretract_enabled = 0;
				queue.inject_P("M209 S0");
				settings.save();
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(1), F("OFF"));
			}
			else{
				fwretract.autoretract_enabled = 1;
				queue.inject_P("M209 S1");
				settings.save();
				DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 226, MBASE(1), F("ON "));
			}
      break;
	  
    case RETRACT_CASE_RETRACT_MM:   			// RETRACT_MM
      checkkey = Retract_MM;
			HMI_ValueStruct.Retract_MM_scale = fwretract.settings.retract_length*100;
			DWIN_Draw_Signed_Float(font8x16, Color_White,Select_Color, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RETRACT_MM), HMI_ValueStruct.Retract_MM_scale);
	    EncoderRate.enabled = true;
      break;

	  case RETRACT_CASE_RETRACT_V:   			// RETRACT_V
      checkkey = Retract_V;
			HMI_ValueStruct.Retract_V_scale = fwretract.settings.retract_feedrate_mm_s*100;
      DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RETRACT_V), HMI_ValueStruct.Retract_V_scale);
	    EncoderRate.enabled = true;
      break;

	  case RETRACT_CASE_RECOVER_MM:   			// RECOVER_MM
      checkkey = UnRetract_MM;
			HMI_ValueStruct.unRetract_MM_scale = fwretract.settings.retract_recover_extra*100;
      DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RECOVER_MM), HMI_ValueStruct.unRetract_MM_scale);
	    EncoderRate.enabled = true;
      break;

	  case RETRACT_CASE_RECOVER_V:   			// RECOVER_V
      checkkey = UnRetract_V;
			HMI_ValueStruct.unRetract_V_scale = fwretract.settings.retract_recover_feedrate_mm_s*100;
      DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 2, 216, MBASE(MROWS -index_retract + RETRACT_CASE_RECOVER_V), HMI_ValueStruct.unRetract_V_scale);
	    EncoderRate.enabled = true;
      break;
      
      default: break;
    }
  }
  DWIN_UpdateLCD();
}
#endif

/* Mixer_Manual */
void HMI_Mixer_Manual() {
  uint8_t i = 0;
  signed int Temp_Buff[MIXING_STEPPERS] = {0};
  
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_manual.inc(1 + MANUAL_CASE_TOTAL)) Move_Highlight(1, select_manual.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_manual.dec()) Move_Highlight(-1, select_manual.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_manual.now) {
	  case 0: 						// Back
        checkkey = DMixer;
        select_mixer.set(MIXER_CASE_MANUAL);
        Draw_Mixer_Menu();
        break;

	  #if ENABLED(MIXING_EXTRUDER)
		#if(MIXING_STEPPERS == 4)
			case MANUAL_CASE_EXTRUDER1: 	// ex1
      			checkkey = Mix_Manual_Extruder1;
				DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(5), mixer.selected_vtool);
				//EncoderRate.enabled = true;
        		break;
      		case MANUAL_CASE_EXTRUDER2: 	// ex2
        		checkkey = Mix_Manual_Extruder2;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(2), MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(5), mixer.selected_vtool);
				//EncoderRate.enabled = true;
        		break;
      		case MANUAL_CASE_EXTRUDER3:  	// ex3
        		checkkey = Mix_Manual_Extruder3;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(3), MixerCfg.Manual_Percent[mixer.selected_vtool][2]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(5), mixer.selected_vtool);
				//EncoderRate.enabled = true;
        		break;
	  		case MANUAL_CASE_EXTRUDER4:  	// ex4
        		checkkey = Mix_Manual_Extruder4;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(4), MixerCfg.Manual_Percent[mixer.selected_vtool][3]);
	  			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(5), mixer.selected_vtool);
	  			//EncoderRate.enabled = true;
        		break;
		#elif(MIXING_STEPPERS == 3)
			case MANUAL_CASE_EXTRUDER1: 	// ex1
      			checkkey = Mix_Manual_Extruder1;
				DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
				//EncoderRate.enabled = true;
        		break;
      		case MANUAL_CASE_EXTRUDER2: 	// ex2
        		checkkey = Mix_Manual_Extruder2;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(2), MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
				//EncoderRate.enabled = true;
        		break;
      		case MANUAL_CASE_EXTRUDER3:  	// ex3
        		checkkey = Mix_Manual_Extruder3;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(3), MixerCfg.Manual_Percent[mixer.selected_vtool][2]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
				//EncoderRate.enabled = true;
        		break;
		#elif(MIXING_STEPPERS == 2)
			case MANUAL_CASE_EXTRUDER1: 	// ex1
      			checkkey = Mix_Manual_Extruder1;
				DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(3), mixer.selected_vtool);
				//EncoderRate.enabled = true;
        		break;
      		case MANUAL_CASE_EXTRUDER2: 	// ex2
        		checkkey = Mix_Manual_Extruder2;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(2), MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(3), mixer.selected_vtool);
				//EncoderRate.enabled = true;
        		break;
		#endif
	  #endif
		
	  case MANUAL_CASE_OK:  		// OK
	    checkkey = Mix_Manual;
		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_Bg_Red, Color_Bg_Black, 3, 216, MBASE(MIXING_STEPPERS+1), mixer.selected_vtool);
		//MIXER_STEPPER_LOOP(i) mixer.mix[i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i];
		//recalculation_mixer_percent();
		//MIXER_STEPPER_LOOP(i) {
			//mixer.color[mixer.selected_vtool][i] = mixer.mix[i];
			//DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), mixer.mix[i]);
			//DWIN_UpdateLCD();
		//}
	    #if(MIXING_STEPPERS == 4)
			if(!Check_Percent_equal()) {
				for(i=0;i<4;i++){
				mixer.color[mixer.selected_vtool][i] = mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 25;
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
				}
	    	}
			else 
			{
				for(i=0;i<4;i++){
				if(i < 3) Temp_Buff[i] = 100*MixerCfg.Manual_Percent[mixer.selected_vtool][i]/(MixerCfg.Manual_Percent[mixer.selected_vtool][0]+MixerCfg.Manual_Percent[mixer.selected_vtool][1]+MixerCfg.Manual_Percent[mixer.selected_vtool][2]+MixerCfg.Manual_Percent[mixer.selected_vtool][3]);
				else Temp_Buff[i] = 100 - Temp_Buff[0]-Temp_Buff[1]-Temp_Buff[2];	
  				}

				for(i=0;i<4;i++){
				mixer.color[mixer.selected_vtool][i] = mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = Temp_Buff[i];
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
				}
			}
		#elif(MIXING_STEPPERS == 3)
    	if(!Check_Percent_equal()) {
				for(i=0;i<2;i++){
					mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 33;
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
					DWIN_UpdateLCD();
				}
				i++;
				mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 34;
  			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
    	}
			else 
			{
				for(i=0;i<3;i++){
					if(i < 2) Temp_Buff[i] = 100*MixerCfg.Manual_Percent[mixer.selected_vtool][i]/(MixerCfg.Manual_Percent[mixer.selected_vtool][0]+MixerCfg.Manual_Percent[mixer.selected_vtool][1]+MixerCfg.Manual_Percent[mixer.selected_vtool][2]);
					else Temp_Buff[i] = 100 - Temp_Buff[0]-Temp_Buff[1];	
				}

				for(i=0;i<3;i++){
					mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = Temp_Buff[i];
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
					mixer.color[mixer.selected_vtool][i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i];
				}
			}
		#elif(MIXING_STEPPERS == 2)
	    	if(!Check_Percent_equal()) {
					for(i=0;i<2;i++){
					mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 50;
	    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
					}
	    	}
			else 
			{
				for(i=0;i<2;i++){
					if(i < 1) Temp_Buff[i] = 100*MixerCfg.Manual_Percent[mixer.selected_vtool][i]/(MixerCfg.Manual_Percent[mixer.selected_vtool][0]+MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
					else Temp_Buff[i] = 100 - Temp_Buff[0];	
				}

				for(i=0;i<2;i++){
					mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = Temp_Buff[i];
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
					mixer.color[mixer.selected_vtool][i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i];
					//DWIN_UpdateLCD();
				}
			}
		#endif
		mixer.copy_mix_to_collector();
    	mixer.normalize();
      break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Mixer_Auto */
void HMI_Mixer_Auto() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  
  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_auto.inc(1 + AUTO_CASE_TOTAL)) {
      if (select_auto.now > MROWS && select_auto.now > index_auto) {
        index_auto = select_auto.now;
      }
      else {
        Move_Highlight(1, select_auto.now + MROWS - index_auto);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_auto.dec()) {
      if (select_auto.now < index_auto - MROWS) {
        index_auto--;
        Scroll_Menu(DWIN_SCROLL_DOWN);
      }
      else {
        Move_Highlight(-1, select_auto.now + MROWS - index_auto);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_auto.now) {
	  case 0: 						// Back
        checkkey = DMixer;
        select_mixer.set(MIXER_CASE_AUTO);
        Draw_Mixer_Menu();
        break;
	  case AUTO_CASE_ZPOS_START:  		// zpos_start
        checkkey = Auto_Zpos_Start;
				HMI_ValueStruct.Auto_Zstart_scale = mixer.gradient.start_z*MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_START), HMI_ValueStruct.Auto_Zstart_scale);
		    EncoderRate.enabled = true;
        break;
	  case AUTO_CASE_ZPOS_END:  		// zpos_end
        checkkey = Auto_Zpos_End;
				HMI_ValueStruct.Auto_Zstart_scale = mixer.gradient.end_z*MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_END), HMI_ValueStruct.Auto_Zend_scale);
		    EncoderRate.enabled = true;
        break;
	  case AUTO_CASE_VTOOL_START:  		// vtool_start
        checkkey = Mix_VTool_Start;
        DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_START), mixer.gradient.start_vtool);
				EncoderRate.enabled = true;
        break;
	  case AUTO_CASE_VTOOL_END:  		// vtool_end
        checkkey = Mix_VTool_End;
        DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_END), mixer.gradient.end_vtool);
				EncoderRate.enabled = true;
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Mixer_Auto */
void HMI_Mixer_Random() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  
  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_random.inc(1 + RANDOM_CASE_TOTAL)) {
      if (select_random.now > MROWS && select_random.now > index_random) {
        index_random = select_random.now;
      }
      else {
        Move_Highlight(1, select_random.now + MROWS - index_random);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_random.dec()) {
      if (select_random.now < index_random - MROWS) {
        index_random--;
        Scroll_Menu(DWIN_SCROLL_DOWN);
      }
      else {
        Move_Highlight(-1, select_random.now + MROWS - index_random);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_random.now) {
	  case 0: 						// Back
        checkkey = DMixer;
        select_mixer.set(MIXER_CASE_RANDOM);
        Draw_Mixer_Menu();
      break;
		
	  case RANDOM_CASE_ZPOS_START:  		// zpos_start
        checkkey = Random_Zpos_Start;
				HMI_ValueStruct.Random_Zstart_scale = mixer.random_mix.start_z*MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
		    EncoderRate.enabled = true;
      break;
	  case RANDOM_CASE_ZPOS_END:  			// zpos_end
        checkkey = Random_Zpos_End;
				HMI_ValueStruct.Random_Zend_scale = mixer.random_mix.end_z*MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);
		    EncoderRate.enabled = true;
      break;
	   case RANDOM_CASE_HEIGHT:  		// Height
        checkkey = Random_Height;
				HMI_ValueStruct.Random_Height = mixer.random_mix.height*MINUNITMULT;
				DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_HEIGHT), HMI_ValueStruct.Random_Height);
		    EncoderRate.enabled = true;
      break;
	   case RANDOM_CASE_EXTRUDERS:  		// Extruders
        checkkey = Random_Extruders;
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(RANDOM_CASE_EXTRUDERS), mixer.random_mix.extruders);
		    EncoderRate.enabled = true;
      break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Mixer_Vtool */
void HMI_Adjust_Mixer_Vtool() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, mixer.selected_vtool)) {
      checkkey = DMixer;
      EncoderRate.enabled = false;
	  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
	  MixerCfg.Vtool_Backup  = mixer.selected_vtool;
	  #if ENABLED(POWER_LOSS_RECOVERY)
	  recovery.save(true);
	  #endif
	  updata_mixer_from_vtool();
	  DWIN_UpdateLCD();
      return;
    }
    NOLESS(mixer.selected_vtool, 0);
	NOMORE(mixer.selected_vtool, MIXING_VIRTUAL_TOOLS);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
	MixerCfg.Vtool_Backup  = mixer.selected_vtool;
	DWIN_UpdateLCD();
  }
}

/* Info */
uint8_t testmode_click_times = 0;
void HMI_Info() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW) {
  	testmode_click_times = 0;
  	if (select_info.inc(1 + INFO_CASE_TOTAL)) {
      if (select_info.now > MROWS && select_info.now > index_info) {
        index_info = select_info.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);		
        if(index_info == INFO_CASE_EXTRUDER_NUM) Item_Info_Extruder_Num(MROWS);
		if(index_info == INFO_CASE_EXTRUDER_MODEL) Item_Info_Extruder_Model(MROWS);		
		#if ENABLED(OPTION_DUALZ_DRIVE)
			if(index_info == INFO_CASE_DUALZ_DRIVE) Item_Info_DualZ_Drive(MROWS);
		#endif
		#if ENABLED(OPTION_Z2_ENDSTOP)
			if(index_info == INFO_CASE_DUALZ_ENDSTOP) Item_Info_DualZ_Endstop(MROWS);
		#endif
		if(index_info == INFO_CASE_BAUDRATE) Item_Info_Baudrate(MROWS);
		if(index_info == INFO_CASE_PROTOCOL) Item_Info_Protocol(MROWS);
		if(index_info == INFO_CASE_PSU) Item_Info_Psu(MROWS);
		if(index_info == INFO_CASE_DATE) Item_Info_Date(MROWS);
		if(index_info == INFO_CASE_THERMISTOR) Item_Info_Thermistor(MROWS);
		if(index_info == INFO_CASE_BED) Item_Info_Bed(MROWS);
		if(index_info == INFO_CASE_HOT) Item_Info_Hot(MROWS);
      }
      else Move_Highlight(1, select_info.now + MROWS - index_info);
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
  	testmode_click_times = 0;
  	if (select_info.dec()) {
      if (select_info.now < index_info - MROWS) {
        index_info--;
        Scroll_Menu(DWIN_SCROLL_DOWN);		
				if (index_info == MROWS){
          Draw_Back_First();
				  DWIN_Draw_Line(Line_Color, 16, MBASE(1) + 33, 256, MBASE(1) + 34);
				}
        else
          DWIN_Draw_Line(Line_Color, 16, MBASE(0) + 33, 256, MBASE(0) + 34);
		
				if(index_info - MROWS == INFO_CASE_BAUDRATE) Item_Info_Baudrate(0);

			#if ENABLED(OPTION_Z2_ENDSTOP)
				if(index_info - MROWS == INFO_CASE_DUALZ_ENDSTOP) Item_Info_DualZ_Endstop(0);
			#endif

			#if ENABLED(OPTION_DUALZ_DRIVE)
				if(index_info - MROWS == INFO_CASE_DUALZ_DRIVE) Item_Info_DualZ_Drive(0);
			#endif

				if(index_info - MROWS == INFO_CASE_EXTRUDER_MODEL) Item_Info_Extruder_Model(0);
				if(index_info - MROWS == INFO_CASE_EXTRUDER_NUM) Item_Info_Extruder_Num(0);
				if(index_info - MROWS == INFO_CASE_BOARD) Item_Info_Board(0);
				if(index_info - MROWS == INFO_CASE_MODEL) Item_Info_Model(0);
        if(index_info - MROWS == INFO_CASE_WEBSITE) Item_Info_Website(0);
				if(index_info - MROWS == INFO_CASE_FIRMWARE) Item_Info_Firmware(0);
				if(index_info - MROWS == INFO_CASE_VERSION) Item_Info_Version(0);
      }
      else {
		  	Move_Highlight(-1, select_info.now + MROWS - index_info);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
  	switch (select_info.now) {
      case 0: // Back
        select_page.set(3);
        Goto_MainMenu();
        break;
	  case INFO_CASE_DATE:	  	
		 if(++testmode_click_times >= 5)	HMI_StartTest();
		break;
	  default:
	  	break;
  	}
  }
  DWIN_UpdateLCD();
}

/* Tune */
void HMI_Tune() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_tune.inc(1 + TUNE_CASE_TOTAL)) {
      if (select_tune.now > MROWS && select_tune.now > index_tune) {
        index_tune = select_tune.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
		
        if(index_tune == TUNE_CASE_MIXER) Item_Tune_Mixer(MROWS);
				if(index_tune == TUNE_CASE_CONFIG)	Item_Tune_Config(MROWS);
      }
      else {
        Move_Highlight(1, select_tune.now + MROWS - index_tune);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_tune.dec()) {
      if (select_tune.now < index_tune - MROWS) {
        index_tune--;
        Scroll_Menu(DWIN_SCROLL_DOWN);
				if (index_tune == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_Setspeed + select_tune.now - 1);

        if(index_tune - MROWS == TUNE_CASE_TEMP) Item_Tune_Temperture(0);
				if(index_tune - MROWS == TUNE_CASE_SPEED) Item_Tune_Speed(0);
      }
      else {
        Move_Highlight(-1, select_tune.now + MROWS - index_tune);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
	#if ENABLED(BABYSTEPPING)
  	Babysteps_timer_second = millis();
		if(Babysteps_timer_second - Babysteps_timer_first < 1000){
			Babysteps_timer_second = Babysteps_timer_first = 0;
			checkkey = Babysteps;
			Draw_Babystep_Menu();
		}
		else{
			Babysteps_timer_first = millis();	
	#else
			{
  #endif
    	switch (select_tune.now) {
      	case 0: // Back
        	select_print.set(0);
        	Goto_PrintProcess();
      	break;
      	case TUNE_CASE_SPEED: // Print speed
        	checkkey = PrintSpeed;
        	HMI_ValueStruct.print_speed = feedrate_percentage;
        	DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(TUNE_CASE_SPEED + MROWS - index_tune), feedrate_percentage);
        	EncoderRate.enabled = true;
        	break;
    	#if HAS_HOTEND
      	case TUNE_CASE_TEMP: // Nozzle temp
        		checkkey = ETemp;
        		HMI_ValueStruct.E_Temp = thermalManager.temp_hotend[0].target;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(TUNE_CASE_TEMP + MROWS - index_tune), thermalManager.temp_hotend[0].target);
        	EncoderRate.enabled = true;
        	break;
    	#endif
    	#if HAS_HEATED_BED
      	case TUNE_CASE_BED: // Bed temp
       	 	checkkey = BedTemp;
        		HMI_ValueStruct.Bed_Temp = thermalManager.temp_bed.target;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(TUNE_CASE_BED + MROWS - index_tune), thermalManager.temp_bed.target);
        		EncoderRate.enabled = true;
        	break;
    	#endif
    	#if HAS_FAN
      	case TUNE_CASE_FAN: // Fan speed
        		checkkey = FanSpeed;
        		HMI_ValueStruct.Fan_speed = thermalManager.fan_speed[0];
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(TUNE_CASE_FAN + MROWS - index_tune), thermalManager.fan_speed[0]);
        		EncoderRate.enabled = true;
        	break;
    	#endif
    	#if ENABLED(BABYSTEPPING)
      	case TUNE_CASE_ZOFF: // Z-offset
	      	checkkey = Homeoffset;
        	DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 2, 2, 202, MBASE(TUNE_CASE_ZOFF + MROWS - index_tune), HMI_ValueStruct.zoffset_value);
        	EncoderRate.enabled = true;
      	break;
    	#endif

	  	case TUNE_CASE_MIXER:
	  			checkkey = DMixer;
        	select_mixer.reset();
        	Draw_Mixer_Menu();
	  		break;

			case TUNE_CASE_CONFIG:
	  		checkkey = Config;
      	select_config.reset();
      	Draw_Config_Menu();
	  		break;
		
      	default: break;
    	}  			
		}
  }
  DWIN_UpdateLCD();
}

#if HAS_PREHEAT

  /* PLA Preheat */
  void HMI_PLAPreheatSetting() {
    ENCODER_DiffState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    // Avoid flicker by updating only the previous menu
    if (encoder_diffState == ENCODER_DIFF_CW) {
      if (select_PLA.inc(1 + PREHEAT_CASE_TOTAL)) Move_Highlight(1, select_PLA.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      if (select_PLA.dec()) Move_Highlight(-1, select_PLA.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      switch (select_PLA.now) {
        case 0: // Back
          checkkey = TemperatureID;
          select_temp.now = TEMP_CASE_PLA;
          HMI_ValueStruct.show_mode = -1;
          Draw_Temperature_Menu();
          break;
        #if HAS_HOTEND
          case PREHEAT_CASE_TEMP: // Nozzle temperature
            checkkey = ETemp;
            HMI_ValueStruct.E_Temp = ui.material_preset[0].hotend_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_TEMP), ui.material_preset[0].hotend_temp);
            EncoderRate.enabled = true;
            break;
        #endif
        #if HAS_HEATED_BED
          case PREHEAT_CASE_BED: // Bed temperature
            checkkey = BedTemp;
            HMI_ValueStruct.Bed_Temp = ui.material_preset[0].bed_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_BED), ui.material_preset[0].bed_temp);
            EncoderRate.enabled = true;
            break;
        #endif
        #if HAS_FAN
          case PREHEAT_CASE_FAN: // Fan speed
            checkkey = FanSpeed;
            HMI_ValueStruct.Fan_speed = ui.material_preset[0].fan_speed;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_FAN), ui.material_preset[0].fan_speed);
            EncoderRate.enabled = true;
            break;
        #endif
        #if ENABLED(EEPROM_SETTINGS)
          case PREHEAT_CASE_SAVE: { // Save PLA configuration
            HMI_AudioFeedback(settings.save());
          } break;
        #endif
        default: break;
      }
    }
    DWIN_UpdateLCD();
  }

  /* ABS Preheat */
  void HMI_ABSPreheatSetting() {
    ENCODER_DiffState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    // Avoid flicker by updating only the previous menu
    if (encoder_diffState == ENCODER_DIFF_CW) {
      if (select_ABS.inc(1 + PREHEAT_CASE_TOTAL)) Move_Highlight(1, select_ABS.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      if (select_ABS.dec()) Move_Highlight(-1, select_ABS.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      switch (select_ABS.now) {
        case 0: // Back
          checkkey = TemperatureID;
          select_temp.now = TEMP_CASE_ABS;
          HMI_ValueStruct.show_mode = -1;
          Draw_Temperature_Menu();
          break;
        #if HAS_HOTEND
          case PREHEAT_CASE_TEMP: // Set nozzle temperature
            checkkey = ETemp;
            HMI_ValueStruct.E_Temp = ui.material_preset[1].hotend_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_TEMP), ui.material_preset[1].hotend_temp);
            EncoderRate.enabled = true;
            break;
        #endif
        #if HAS_HEATED_BED
          case PREHEAT_CASE_BED: // Set bed temperature
            checkkey = BedTemp;
            HMI_ValueStruct.Bed_Temp = ui.material_preset[1].bed_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_BED), ui.material_preset[1].bed_temp);
            EncoderRate.enabled = true;
            break;
        #endif
        #if HAS_FAN
          case PREHEAT_CASE_FAN: // Set fan speed
            checkkey = FanSpeed;
            HMI_ValueStruct.Fan_speed = ui.material_preset[1].fan_speed;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_FAN), ui.material_preset[1].fan_speed);
            EncoderRate.enabled = true;
            break;
        #endif
        #if ENABLED(EEPROM_SETTINGS)
          case PREHEAT_CASE_SAVE: { // Save ABS configuration
            const bool success = settings.save();
            HMI_AudioFeedback(success);
          } break;
        #endif
        default: break;
      }
    }
    DWIN_UpdateLCD();
  }

#endif

/* Max Speed */
void HMI_MaxSpeed() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_speed.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_speed.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_speed.dec()) Move_Highlight(-1, select_speed.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (WITHIN(select_speed.now, 1, 4)) {
      checkkey = MaxSpeed_value;
      HMI_flag.feedspeed_axis = AxisEnum(select_speed.now - 1);
      HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[HMI_flag.feedspeed_axis];
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 4, 210, MBASE(select_speed.now), HMI_ValueStruct.Max_Feedspeed);
      EncoderRate.enabled = true;
    }
    else { // Back
      checkkey = Motion;
      select_motion.now = MOTION_CASE_RATE;
      Draw_Motion_Menu();
    }
  }
  DWIN_UpdateLCD();
}

/* Max Acceleration */
void HMI_MaxAcceleration() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_acc.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_acc.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_acc.dec()) Move_Highlight(-1, select_acc.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (WITHIN(select_acc.now, 1, 4)) {
      checkkey = MaxAcceleration_value;
      HMI_flag.acc_axis = AxisEnum(select_acc.now - 1);
      HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[HMI_flag.acc_axis];
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 5, 210, MBASE(select_acc.now), HMI_ValueStruct.Max_Acceleration);
      EncoderRate.enabled = true;
    }
    else { // Back
      checkkey = Motion;
      select_motion.now = MOTION_CASE_ACCEL;
      Draw_Motion_Menu();
    }
  }
  DWIN_UpdateLCD();
}

#if HAS_CLASSIC_JERK
  /* Max Jerk */
  void HMI_MaxJerk() {
    ENCODER_DiffState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    // Avoid flicker by updating only the previous menu
    if (encoder_diffState == ENCODER_DIFF_CW) {
      if (select_jerk.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_jerk.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      if (select_jerk.dec()) Move_Highlight(-1, select_jerk.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      if (WITHIN(select_jerk.now, 1, 4)) {
        checkkey = MaxJerk_value;
        HMI_flag.jerk_axis = AxisEnum(select_jerk.now - 1);
        HMI_ValueStruct.Max_Jerk = planner.max_jerk[HMI_flag.jerk_axis] * MINUNITMULT;
        DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 2, 1, 210, MBASE(select_jerk.now), HMI_ValueStruct.Max_Jerk);
        EncoderRate.enabled = true;
      }
      else { // Back
        checkkey = Motion;
        select_motion.now = MOTION_CASE_JERK;
        Draw_Motion_Menu();
      }
    }
    DWIN_UpdateLCD();
  }
#endif // HAS_CLASSIC_JERK

/* Step */
void HMI_Step() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_step.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_step.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_step.dec()) Move_Highlight(-1, select_step.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (WITHIN(select_step.now, 1, 4)) {
      checkkey = Step_value;
      HMI_flag.step_axis = AxisEnum(select_step.now - 1);
      HMI_ValueStruct.Max_Step = planner.settings.axis_steps_per_mm[HMI_flag.step_axis] * MINUNITMULT;
      DWIN_Draw_Signed_Float(font8x16, Color_White, Select_Color, 4, 1, 210, MBASE(select_step.now), HMI_ValueStruct.Max_Step);
      EncoderRate.enabled = true;
    }
    else { // Back
      checkkey = Motion;
      select_motion.now = MOTION_CASE_STEPS;
      Draw_Motion_Menu();
    }
  }
  DWIN_UpdateLCD();
}

void HMI_Init() {
  #if HAS_COLOR_LEDS 
  RGB_LED_Light(HMI_flag.RGB_LED_Counter);
  #endif
  
  DWIN_JPG_ShowAndCache(0);
  HMI_SDCardInit();
  
  for (uint16_t t = 0; t <= 100; t += 2) {
    DWIN_ICON_Show(ICON, ICON_Bar, 15, 260);
    DWIN_Draw_Rectangle(1, Color_Bg_Black, 15 + t * 242 / 100, 260, 257, 280);
    DWIN_UpdateLCD();
    delay(20);
  }
  HMI_SetLanguage();

	HMI_flag.print_finish = false;
	HMI_flag.done_confirm_flag = false;
	dwin_abort_flag = false;
}

#if ENABLED(DEBUG_GCODE_M92)
void DWIN_Gcode_Show_M92(uint8_t AXIS,float lengh){
	if(AXIS == X_AXIS) {
		DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 14, 456, F("X:"));
		DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 30, 456, lengh*10);
	}
	if(AXIS == Y_AXIS) {
		DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 78, 456, F("Y:"));
		DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 94, 456, lengh*10);
	}
	if(AXIS == Z_AXIS) {
		DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 140, 456, F("Z:"));
		DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 156, 456, lengh*10);
	}
	if(AXIS == E_AXIS) {
		DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 204, 456, F("E:"));
		DWIN_Draw_Signed_Float(font8x16, Color_White, Color_Bg_Black, 3, 1, 220, 456, lengh*10);
	}
}
#endif

#if ENABLED(OPTION_WIFI_MODULE)
#define MAX_WIFI_MESSAGE_LENGTH  27
#define START_OF_UTF8_CHAR(C) (((C) & 0xC0u) != 0x80U)

void DWIN_Wifi_Show_M117(const char * const message){
	HMI_flag.wifi_Handshake_ok = true;
	HMI_flag.wifi_link_timer = 0;
	WiFi_Connected_fail = false;
  if(IS_SD_PRINTING() || IS_SD_PAUSED()) return;
	
	char wifi_status_message[MAX_WIFI_MESSAGE_LENGTH+1] = {0};
	checkkey = Wifi;
	Clear_Main_Window();
  Draw_Popup_Bkgd_60();
	Draw_Popup_Bkgd_Wifi();
 	Draw_Title(F("WIFI"));
	Draw_Wifi_Title("M117");
  DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 231);

	// Here we have a problem. The message is encoded in UTF8, so
	// arbitrarily cutting it will be a problem. We MUST be sure
	// that there is no cutting in the middle of a multibyte character!

	// Get a pointer to the null terminator
	const char* pend = message + strlen(message);

	//  If length of supplied UTF8 string is greater than
	// our buffer size, start cutting whole UTF8 chars
  while ((pend - message) > MAX_WIFI_MESSAGE_LENGTH) {
	 --pend;
	 while (!START_OF_UTF8_CHAR(*pend)) --pend;
 };
	uint8_t maxLen = pend - message;
	strncpy(wifi_status_message, message, maxLen);
	wifi_status_message[maxLen] = '\0';
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * strlen(wifi_status_message)) / 2, 168, wifi_status_message);
	
}
#endif

void DWIN_Update() {
  if(HMI_flag.auto_test_flag == 0xaa){
  	Dwin_Lcd_Test();
  }
  else{
  	EachMomentUpdate();   // Status update
  	HMI_SDCardUpdate();   // SD card update
  	DWIN_HandleScreen();  // Rotary encoder update
  }
}

#if HAS_COLOR_LEDS 
void RGB_LED_Light(uint8_t color){
	switch(color){
		case RED_ON: queue.inject_P(PSTR("M150 R255 U0 B0")); break;
		case GREEN_ON: queue.inject_P(PSTR("M150 R0 U255 B0")); break;
		case BLUE_ON: queue.inject_P(PSTR("M150 R0 U0 B255")); break;
		case YELLOW_ON: queue.inject_P(PSTR("M150 R255 U255 B0")); break;
		case CYAN_ON: queue.inject_P(PSTR("M150 R0 U255 B255")); break;
		case PURPLE_ON: queue.inject_P(PSTR("M150 R255 U0 B255")); break;
		case WHITE_ON: queue.inject_P(PSTR("M150 R255 U255 B255")); break;
		default: queue.inject_P(PSTR("M150")); break;
	}
}
#endif

void EachMomentUpdate() {
  #if ENABLED(OPTION_REPEAT_PRINTING)
  if(ReprintManager.Is_Reprint_Reset) {
  	if(ReprintManager.Check_ENDSTOP()){
				ReprintManager.Back_Move_Start();
  	    ReprintManager.Is_Reprint_Reset = 0;
				ReprintManager.Reprint_Reset_Enabled = 1;
	  		ReprintManager.Reprint_Wait_Enabled = 0;
	  		ReprintManager.tempbed_counter = 0;
  	}
  }
  #endif
  
  static millis_t next_rts_update_ms = 0;
  const millis_t ms = millis();
  if (PENDING(ms, next_rts_update_ms)) 	return;
  next_rts_update_ms = ms + DWIN_SCROLL_UPDATE_INTERVAL;

  #if ENABLED(OPTION_REPEAT_PRINTING)
    if(ReprintManager.Reprint_Reset_Enabled){
		if(ReprintManager.Forward_Move_Process(2)){
	   		ReprintManager.Reprint_Reset_Enabled = 0;
	   		ReprintManager.Reprint_Wait_Enabled = 1;
	   		ReprintManager.tempbed_counter = 0;
	   		ReprintManager.Forward_Move_Start();	
		}
		else ReprintManager.Back_Move_Start();
	}

	if(ReprintManager.Reprint_Wait_Enabled){
		if(ReprintManager.Forward_Move_Process(4)){
	   		ReprintManager.Reprint_Wait_Enabled = 0;	
	   		ReprintManager.tempbed_counter = 0;
		    ReprintManager.Reprint_over = 1;
	   		ReprintManager.Back_Move_Stop();
		}
		else ReprintManager.Forward_Move_Start();
	}
	
	if(ReprintManager.Is_Reprint_Print && ReprintManager.enabled){
		if(ReprintManager.Repeat_Print_Process()){
			ReprintManager.Is_Reprint_Print = 0;
			ReprintManager.reprt_state = REPRINT_INIT;
			if(ReprintManager.Reprint_times <= 0) {
				ReprintManager.enabled = false;
				#if HAS_SUICIDE
					HMI_flag.putdown_close_machine = true;
					HMI_flag.putdown_close_timer_rg = POWERDOWN_MACHINE_TIMER;
				#endif
			}
			select_print.reset();
			HMI_flag.heat_flag = true;
  		HMI_flag.print_finish = false;
  		HMI_ValueStruct.show_mode = 0;			
			HMI_flag.done_confirm_flag = false;
			card.openAndPrintFile(card.filename);
  		Goto_PrintProcess();
		}
	}
  #endif
  
  #if ENABLED(OPTION_WIFI_MODULE)
	if(WiFi_Enabled){
  	if(!HMI_flag.wifi_Handshake_ok && !WiFi_Connected_fail){
			if(HMI_flag.wifi_link_timer++ > 12){
				HMI_flag.wifi_link_timer = 0;
				WiFi_Connected_fail = true;
				Popup_window_Wifi_Disconnect();
			}
			else{
				if(checkkey != Popup_Window){
					checkkey = Popup_Window;
					HMI_flag.wifi_link_timer = 0;		
					Popup_window_Wifi_Connect();
				}
				else{
					if((HMI_flag.wifi_link_timer % 5) != 0)	
						DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, 172 + FIL.Font_W*(HMI_flag.wifi_link_timer % 5 - 1), 240, F("."));
					else 
						DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, 172, 240, F("    "));
				}
			}
		}		
		else if(WiFi_Connected_fail && checkkey == Popup_Window){
			if(HMI_flag.wifi_link_timer++ > 3){
				HMI_flag.wifi_link_timer = 0;
				checkkey = Config;
				select_config.set(CONFIG_CASE_WIFI);
				index_config = MROWS;
				Draw_Config_Menu();
			}
		}
	}
  #endif
 
  #if HAS_COLOR_LEDS  
    if(HMI_flag.RGB_LED_Counter == LOOP_ON){
  		RGB_LED_Light(HMI_flag.RGB_LED_Loop);
		if(HMI_flag.RGB_LED_Loop++ >= WHITE_ON) HMI_flag.RGB_LED_Loop = 0;
    }
  #endif
  
  #if ENABLED(OPTION_AUTOPOWEROFF)
   if(HMI_flag.auto_shutdown){
  	if(checkkey == MainMenu){
  		if(HMI_flag.free_close_timer_rg == 0){
				queue.inject_P(PSTR("M81")); 
				HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
  		}
  		else {
				HMI_flag.free_close_timer_rg--;
				//if((HMI_flag.free_close_timer_rg < 2)&&(HMI_flag.free_close_timer_rg>0)) buzzer.tone(100, 700);
				if(HMI_flag.free_close_timer_rg == 0) buzzer.tone(100, 8000);
	  			Draw_Freedown_Machine();
  		}
  	}else HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
  
  	if(HMI_flag.putdown_close_machine){
			Draw_Powerdown_Machine();
  		if(HMI_flag.putdown_close_timer_rg == 0){
				HMI_flag.putdown_close_timer_rg = POWERDOWN_MACHINE_TIMER;
  			HMI_flag.putdown_close_machine = false;
				queue.inject_P(PSTR("M81")); 
  		}
			else {
				HMI_flag.putdown_close_timer_rg--;
				//if((HMI_flag.putdown_close_timer_rg < 2)&&(HMI_flag.putdown_close_timer_rg>0)) buzzer.tone(100, 700);
				if(HMI_flag.putdown_close_timer_rg == 0) buzzer.tone(100, 8000);
			}
  	}
   }
  #endif
  
  // variable update
  update_variable();
  
 #if ENABLED(POWER_LOSS_RECOVERY)
  if((current_position.z != HMI_flag.current_zpos_backup)&&(recovery.enabled)){
		HMI_flag.current_zpos_backup = current_position.z;
		recovery.save(true);
  }
 #endif
  
  if (checkkey == PrintProcess) {
		if(HMI_flag.print_finish && !IS_SD_PRINTING() && !IS_SD_PAUSED() && !IS_SD_FILE_OPEN()) {
			//print finished      
			HMI_flag.print_finish = false;
			HMI_flag.done_confirm_flag = true;
	  	mixer.reset_vtools();	  
      TERN_(POWER_LOSS_RECOVERY, recovery.cancel());
      // show percent bar and value
      Percentrecord = 0;
      Draw_Print_ProgressBar();

		  #if ENABLED(OPTION_REPEAT_PRINTING)
		  if(!ReprintManager.enabled || (current_position.z < 20)){
		  //if(!ReprintManager.enabled){
		  #endif
		  	planner.finish_and_disable();
			#if HAS_SUICIDE
				HMI_flag.putdown_close_machine = true;
				HMI_flag.putdown_close_timer_rg = POWERDOWN_MACHINE_TIMER;
			#endif
      	// show print done confirm
      	DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 250, DWIN_WIDTH - 1, STATUS_Y_START);
      	DWIN_ICON_Show(ICON,ICON_Confirm_E, 86, 283);
	  #if ENABLED(OPTION_REPEAT_PRINTING)
		  }
		  else{		  	
		  	planner.synchronize();
				ReprintManager.Reprint_times--;
		  	Popup_Window_BTempCooldown();				
				ReprintManager.reprt_state = REPRINT_INIT;
				ReprintManager.Is_Reprint_Print = 1;
		  }
	  #endif
    }
    else if (HMI_flag.pause_flag != printingIsPaused()) {
      // print status update
      //mixer.selected_vtool = MixerCfg.Vtool_Backup;
      HMI_flag.pause_zpos_backup = current_position.z;
      HMI_flag.pause_flag = printingIsPaused();

      if (HMI_flag.pause_flag){
		  	if(!HMI_flag.filament_runout_star) ICON_Continue();
      }
		  else {
		  	if(!HMI_flag.filament_runout_end) 
					ICON_Pause();
				else{
					HMI_flag.filament_runout_star = 0;
					HMI_flag.filament_runout_end = 0;
					Goto_PrintProcess();
				}
		  }
    }
  }

  // pause after homing
  if (HMI_flag.pause_action && printingIsPaused() && !planner.has_blocks_queued()) {
    HMI_flag.pause_action = false;

    #if ENABLED(PAUSE_HEAT)
      #if HAS_HEATED_BED
        tempbed = thermalManager.temp_bed.target;
      #endif
      #if HAS_HOTEND
        temphot = thermalManager.temp_hotend[0].target;
      #endif
      thermalManager.disable_all_heaters();
    #endif
    queue.inject_P(PSTR("G1 F1200 Y0 X0"));
  }

  if (card.isPrinting() && checkkey == PrintProcess) { // print process
    const uint8_t card_pct = card.percentDone();
    static uint8_t last_cardpercentValue = 101;
	
    if (last_cardpercentValue != card_pct) { // print percent
      last_cardpercentValue = card_pct;
      if (card_pct) {
        Percentrecord = card_pct;
        Draw_Print_ProgressBar();
      }
    }
    duration_t elapsed = print_job_timer.duration(); // print timer

    // Print time so far
    static uint16_t last_Printtime = 0;
    const uint16_t min = (elapsed.value % 3600) / 60;
    if (last_Printtime != min) { // 1 minute update
      last_Printtime = min;
      Draw_Print_ProgressElapsed();
    }

    // Estimate remaining time every 20 seconds
    static millis_t next_remain_time_update = 0;
    if (Percentrecord > 1 && ELAPSED(ms, next_remain_time_update) && !HMI_flag.heat_flag) {
      remain_time = (elapsed.value - dwin_heat_time) / (Percentrecord * 0.01f) - (elapsed.value - dwin_heat_time);
      next_remain_time_update += 20 * 1000UL;
      Draw_Print_ProgressRemain();
    }
  }
  else if (dwin_abort_flag) { // Print Stop
    dwin_abort_flag = false;
    HMI_ValueStruct.print_speed = feedrate_percentage = 100;
		HMI_flag.print_finish = false;
		HMI_flag.done_confirm_flag= false;
    select_page.set(0);
    Goto_MainMenu();
		mixer.reset_vtools();
  }
  #if (ENABLED(POWER_LOSS_RECOVERY))
    else if (DWIN_lcd_sd_status && recovery.dwin_flag && recovery.enabled) { // resume print before power off
      static bool recovery_flag = false;

      TERN_(HAS_DWIN_LCD,  recovery.dwin_flag = false);
      recovery_flag = true;

      auto update_selection = [&](const bool sel) {
        HMI_flag.select_flag = sel;
        const uint16_t c1 = sel ? Color_Bg_Window : Select_Color;
        DWIN_Draw_Rectangle(0, c1, 25, 306, 126, 345);
        DWIN_Draw_Rectangle(0, c1, 24, 305, 127, 346);
        const uint16_t c2 = sel ? Select_Color : Color_Bg_Window;
        DWIN_Draw_Rectangle(0, c2, 145, 306, 246, 345);
        DWIN_Draw_Rectangle(0, c2, 144, 305, 247, 346);
      };

      Popup_Window_Resume();
      update_selection(true);

      /// TODO: Get the name of the current file from someplace
      //
      //(void)recovery.interrupted_file_exists();
      char * const name = card.longest_filename();
      const int8_t npos = _MAX(0U, DWIN_WIDTH - strlen(name) * (MENU_CHR_W)) / 2;
      DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, npos, 252, name);
      DWIN_UpdateLCD();

      while (recovery_flag) {
        ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
        if (encoder_diffState != ENCODER_DIFF_NO) {
          if (encoder_diffState == ENCODER_DIFF_ENTER) {
            recovery_flag = false;
            if (HMI_flag.select_flag) break;
            TERN_(POWER_LOSS_RECOVERY, queue.inject_P(PSTR("M1000C")));
            HMI_StartFrame(true);
            return;
          }
          else
            update_selection(encoder_diffState == ENCODER_DIFF_CW);
          DWIN_UpdateLCD();
        }
				TERN_(USE_WATCHDOG, HAL_watchdog_refresh());
      }

      select_print.set(0);
      HMI_ValueStruct.show_mode = 0;
      queue.inject_P(PSTR("M1000"));
      
		  Draw_Status_Area(true);
		  Goto_PrintProcess();
		  Draw_Mixer_Status_Area(true);
    }
  #endif
  DWIN_UpdateLCD();
}

void DWIN_HandleScreen() {
  switch (checkkey) {
    case MainMenu:        HMI_MainMenu(); break;
    case SelectFile:      HMI_SelectFile(); break;
    case Prepare:         HMI_Prepare(); break;
    case Control:         HMI_Control(); break;
    case Leveling:        break;
		case Last_Level_CatchPop: 			
	  case Leveling0:       HMI_Leveling0(); break;
		
	  case Home:       	  HMI_Home(); break; 
    case PrintProcess:    HMI_Printing(); break;
    case Print_window:    HMI_PauseOrStop(); break;
    case AxisMove:        HMI_AxisMove(); break;
    case TemperatureID:   HMI_Temperature(); break;
    case Motion:          HMI_Motion(); break;
	  case DMixer:          HMI_Mixer(); break;
    case Info:            HMI_Info(); break;
    case Tune:            HMI_Tune(); break;
    #if HAS_PREHEAT
      case PLAPreheat:    HMI_PLAPreheatSetting(); break;
      case ABSPreheat:    HMI_ABSPreheatSetting(); break;
    #endif
    case MaxSpeed:        HMI_MaxSpeed(); break;
    case MaxAcceleration: HMI_MaxAcceleration(); break;
    case MaxJerk:         HMI_MaxJerk(); break;
    case Step:            HMI_Step(); break;
    case Move_X:          HMI_Move_X(); break;
    case Move_Y:          HMI_Move_Y(); break;
    case Move_Z:          HMI_Move_Z(); break;
#if HAS_HOTEND
		case ETemp:         	HMI_ETemp(); break;
		case Extruder1:      	HMI_Move_E1(); break;
	#if(E_STEPPERS > 1)
    case Extruder2:      	HMI_Move_E2(); break;
	#endif
	#if(E_STEPPERS > 2)
		case Extruder3:      	HMI_Move_E3(); break;
	#endif
	#if(E_STEPPERS > 3)
		case Extruder4:      	HMI_Move_E4(); break;
	#endif
	#if ENABLED(MIXING_EXTRUDER)
		case ExtruderAll:      	HMI_Move_AllExtr(); break;	
	#endif      
#endif
	
  #if ENABLED(BABYSTEPPING)
    case Homeoffset:    HMI_Zoffset(); break;
  #endif

  #if HAS_HEATED_BED
    case BedTemp:       HMI_BedTemp(); break;
  #endif
  #if HAS_PREHEAT
    case FanSpeed:      HMI_FanSpeed(); break;
  #endif
    case PrintSpeed:      HMI_PrintSpeed(); break;
    case MaxSpeed_value:  HMI_MaxFeedspeedXYZE(); break;
    case MaxAcceleration_value: HMI_MaxAccelerationXYZE(); break;
    case MaxJerk_value:   HMI_MaxJerkXYZE(); break;
    case Step_value:      HMI_StepXYZE(); break;

  #if ENABLED(MIXING_EXTRUDER)
		case Mix_Manual:      HMI_Mixer_Manual(); break;
		case Mix_Auto:        HMI_Mixer_Auto(); break;
		case Mix_Random:      HMI_Mixer_Random(); break;
		case Mix_Vtool:       HMI_Adjust_Mixer_Vtool(); break;
		#if(MIXING_STEPPERS == 4)
		case Mix_Manual_Extruder1:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;//HMI_Adjust_Manual_Ext1_Percent(); break;
		case Mix_Manual_Extruder2:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER2);break;//HMI_Adjust_Manual_Ext2_Percent(); break;
		case Mix_Manual_Extruder3:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER3);break;//HMI_Adjust_Manual_Ext3_Percent(); break;
		case Mix_Manual_Extruder4:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER4);break;//HMI_Adjust_Manual_Ext4_Percent(); break;
		
	  #elif(MIXING_STEPPERS == 3)
		case Mix_Manual_Extruder1:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;//HMI_Adjust_Manual_Ext1_Percent(); break;
		case Mix_Manual_Extruder2:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER2);break;//HMI_Adjust_Manual_Ext2_Percent(); break;
		case Mix_Manual_Extruder3:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER3);break;//HMI_Adjust_Manual_Ext3_Percent(); break;
	  #elif(MIXING_STEPPERS == 2)
		case Mix_Manual_Extruder1:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;//HMI_Adjust_Manual_Ext1_Percent(); break;
		case Mix_Manual_Extruder2:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER2);break;//HMI_Adjust_Manual_Ext2_Percent(); break;
	  #else
		case Mix_Manual_Extruder1:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;//HMI_Adjust_Manual_Ext1_Percent(); break;
	  #endif
  #endif
	
		case Auto_Zpos_Start:     	 		HMI_Adjust_Auto_Zpos_Start(); break;
		case Auto_Zpos_End:   	 	 		HMI_Adjust_Auto_Zpos_End(); break;
		case Mix_VTool_Start:     	 		HMI_Adjust_Auto_VTool_Start(); break;
		case Mix_VTool_End:     	 		HMI_Adjust_Auto_VTool_End(); break;

		case Random_Zpos_Start:     	 	HMI_Adjust_Random_Zpos_Start(); break;
		case Random_Zpos_End:   	 	 	HMI_Adjust_Random_Zpos_End(); break;
		case Random_Height:     	 		HMI_Adjust_Random_Height(); break;
		case Random_Extruders:   	 	 	HMI_Adjust_Random_Extruders(); break;

		case Filament_Option:				HMI_Filament_Runout_Option(); break;

		case Powerdown:						HMI_Powerdown(); break;
		case Language:						HMI_Language(); break;

  #if ENABLED(BABYSTEPPING)
		case Babysteps:						HMI_Pop_Babystep(); break;
	#endif

		case Config:						HMI_Config(); break;

	#if ENABLED(FWRETRACT) 
		case Retract:						HMI_Retract(); break;
		case Retract_MM:					HMI_Retract_MM(); break;
		case Retract_V:						HMI_Retract_V(); break;
		case UnRetract_MM:					HMI_UnRetract_MM(); break;
		case UnRetract_V:					HMI_UnRetract_V(); break;
	#endif

	#if ENABLED(OPTION_WIFI_MODULE)
		case Wifi:						HMI_Wifi(); break;
	#endif
	
	#if ENABLED(BLTOUCH)
		case Bltouch:					HMI_Option_Bltouch(); break;
	#endif
	
	#if ENABLED(OPTION_REPEAT_PRINTING)
	  case Re_print:					HMI_Reprint(); break;
		case Reprint_times:				HMI_Reprint_Times(); break;
		case Forward_lenght:			HMI_Forward_Lenght(); break;
	#endif

	#if ENABLED(OPTION_BED_COATING)
	    case COATING:					HMI_Adjust_Coating_Thickness(); break;
	#endif
	
    default: break;
  }
}

void DWIN_CompletedHoming() {
  if (checkkey == Last_Prepare) {	  
  	checkkey = Home;
    Draw_Home_Menu();
  }
  else if (checkkey == Back_Main) {
    HMI_ValueStruct.print_speed = feedrate_percentage = 100;
    planner.finish_and_disable();
    Goto_MainMenu();
  }
}

#if ENABLED(ABL_GRID)
void DWIN_CompletedLeveling() {
  if (checkkey == Last_Leveling) {
    DWIN_G29_Show_Messge(G29_MESH_DONE);
		checkkey = Leveling0;
		index_leveling = LEVELING_CASE_CATCHOFFSET;
    select_leveling.set(LEVELING_CASE_CATCHOFFSET);
    Draw_Leveling_Menu();
  }
	#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
	else if(checkkey == Last_Level_CatchOffset){
    //DWIN_G29_Show_Messge(G29_CATCH_DONE);
		checkkey = Leveling0;
		index_leveling = LEVELING_CASE_SAVE;
    select_leveling.set(LEVELING_CASE_SAVE);
    Draw_Leveling_Menu();
  }
	#endif
}
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
void DRAW_Filament_Runout_Message(char message,char mode){
	switch (message){
		case DWIN_PAUSE_MESSAGE_CHANGING:
			Popup_window_Filament_Runout_Start(mode);
			break;
		case DWIN_PAUSE_MESSAGE_WAITING:
			break;
		case DWIN_PAUSE_MESSAGE_UNLOAD:
			Popup_window_Filament_Runout_Unload(mode);
			break;
		case DWIN_PAUSE_MESSAGE_INSERT:
			Popup_window_Filament_Runout_Insert(mode);
			break;
		case DWIN_PAUSE_MESSAGE_LOAD:
			Popup_window_Filament_Runout_Load(mode);
			break;
		case DWIN_PAUSE_MESSAGE_PURGE:
			Popup_window_Filament_Runout_Purge(mode);
			break;
		case DWIN_PAUSE_MESSAGE_OPTION:
			checkkey = Filament_Option;
			Popup_window_Filament_Runout_Option(mode);
			break;
		case DWIN_PAUSE_MESSAGE_HEATING:
			Popup_window_Filament_Runout_Heating(mode);
			break;
		case DWIN_PAUSE_MESSAGE_RESUME:
			Popup_window_Filament_Runout_Resume(mode);
			break;
		default:break;
	}
}

void DWIN_Pause_Show_Message(
  const DWINPauseMessage message,
  const DWINPauseMode mode/*=DWIN_PAUSE_MODE_SAME*/
  //const uint8_t extruder/*=active_extruder*/
) {
  DRAW_Filament_Runout_Message(message,mode);
}
#endif

void DWIN_G29_Show_Messge(const DWIN_G29_MESSAGE message/*=G29_LEVLE_DEFAULT*/,const int pt_index,const int all_points,const float fvalue){
	switch(message){
		case G29_CATCH_START:
			Clear_Bottom_Area();
			DWIN_Draw_String(false, true, font10x20, Color_White, Color_Bg_Black, 12, 454, F("To catch offset"));
			break;
		case G29_CATCH_NORMAL:
			Clear_Bottom_Area();
			DWIN_Draw_String(false, true, font10x20, Color_White, Color_Bg_Black, 12, 454, F("Catch Point:   "));
			DWIN_Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Black, 1, 132, 454, pt_index);
			break;
		case G29_CATCH_PROBE_TOO_HIGH:
			Clear_Bottom_Area();
			DWIN_Draw_String(false, true, font10x20, Color_White, Color_Bg_Black, 12, 454, F("Fail! Move down Probe"));
			break;
    case G29_CATCH_FAIL:
			Clear_Bottom_Area();			
			DWIN_Draw_String(false, true, font10x20, Color_White, Color_Bg_Black, 12, 454, F("Fail! Manual level."));
			break;
		case G29_CATCH_DONE:
			Clear_Bottom_Area();
			DWIN_Draw_String(false, true, font10x20, Color_White, Color_Bg_Black, 12, 454, F("Offset catched!= "));
			DWIN_Draw_Signed_Float(font10x20, Color_Red, Color_Bg_Black, 2, 2, 192, 454, (probe.offset.z * 100));			
			break;
		case G29_MESH_START:
			Clear_Bottom_Area();
			DWIN_Draw_String(false, true, font10x20, Color_White, Color_Bg_Black, 12, 454, F("Probing Start!"));
			break;
		case G29_MESH_NORMAL:
			DWIN_Draw_String(false, true, font10x20, Color_White, Color_Bg_Black, 12, 454, F("Point:    /   "));			
			DWIN_Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Black, 2, 82, 454, pt_index);
			DWIN_Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Black, 2, 144, 454, all_points);
			break;
		case G29_MESH_VALUE:
			DWIN_Draw_String(false, true, font10x20, Color_Red, Color_Bg_Black, 180, 454, F("Z="));
			DWIN_Draw_Signed_Float(font10x20, Color_Red, Color_Bg_Black, 2, 2, 200, 454, (fvalue * 100));
			break;
		case G29_MESH_DONE:
			Clear_Bottom_Area();
			DWIN_Draw_String(false, true, font10x20, Color_White, Color_Bg_Black, 12, 454, F("Bed Leveling finished!"));						
			break;
		default:
			Clear_Bottom_Area();
			break;
	}
}

#define HEIGH_12X24   24
#define WIDTH_12X24   12
#define HEIGH_10X20   20
#define WIDTH_10X20   10

#define FONE_GAP	  8
#define LINE_START	  5

#define WORD_WIDTH	  45

static float test_temp_hotend_target = 0;
static float test_temp_hotbed_target = 0;
static float test_temp_hotend_celsius = 0;
static float test_temp_hotbed_celsius = 0;
static float test_temp_hotend_first = 0;
static float test_temp_hotbed_first = 0;

#define TEST_EXTRUDER_AUTO_FAN_TEMPERATURE	40
void Dwin_Lcd_Test() {
  static millis_t test_next_rts_update_ms = 0;

  Check_Rotary();
  
  const millis_t test_ms = millis();
  if (PENDING(test_ms, test_next_rts_update_ms)) return;
  test_next_rts_update_ms = test_ms + 10;

  if(test_temp_hotend_celsius != thermalManager.temp_hotend[0].celsius){
    DWIN_Draw_IntValue(true, true, 0, font12x24, Color_White, Color_Bg_Window, 3, DWIN_WIDTH/2+4*WIDTH_12X24, 3*HEIGH_12X24+3*FONE_GAP, thermalManager.temp_hotend[0].celsius);
	test_temp_hotend_celsius = thermalManager.temp_hotend[0].celsius;
  }
		
  if(test_temp_hotbed_celsius != thermalManager.temp_bed.celsius){
    DWIN_Draw_IntValue(true, true, 0, font12x24, Color_White, Color_Bg_Window, 3, DWIN_WIDTH/2+4*WIDTH_12X24, 5*HEIGH_12X24+5*FONE_GAP, thermalManager.temp_bed.celsius);
	test_temp_hotbed_celsius = thermalManager.temp_bed.celsius;
  }

  if(test_temp_hotend_target != thermalManager.temp_hotend[0].target){
    DWIN_Draw_IntValue(true, true, 0, font12x24, Color_White, Color_Bg_Window, 3, DWIN_WIDTH/2, 3*HEIGH_12X24+3*FONE_GAP, thermalManager.temp_hotend[0].target);
	test_temp_hotend_target = thermalManager.temp_hotend[0].target;
  }
		
  if(test_temp_hotbed_target != thermalManager.temp_bed.target){
    DWIN_Draw_IntValue(true, true, 0, font12x24, Color_White, Color_Bg_Window, 3, DWIN_WIDTH/2, 5*HEIGH_12X24+5*FONE_GAP, thermalManager.temp_bed.target);
	test_temp_hotbed_target = thermalManager.temp_bed.target;
  }

  if((thermalManager.temp_hotend[0].celsius <= TEST_EXTRUDER_AUTO_FAN_TEMPERATURE)&&(HMI_flag.test_loop_rg > CHECK_FAN_SPEED)){
	thermalManager.fan_speed[0] = 0;
	if(!HMI_flag.test_fan_fg){
		HMI_flag.test_fan_fg = 1;
		DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 7*HEIGH_12X24+7*FONE_GAP, DWIN_WIDTH, 8*HEIGH_12X24+7*FONE_GAP);
		DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Black, LINE_START, 7*HEIGH_12X24+7*FONE_GAP, F("ALL Fan Off"));
	}
  }

  switch(HMI_flag.test_loop_rg){
  	case CHECK_SD:
		if(IS_SD_INSERTED()){
			DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 1*HEIGH_12X24+FONE_GAP, DWIN_WIDTH, 3*HEIGH_12X24+2*FONE_GAP);
			DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Black, LINE_START, 1*HEIGH_12X24+1*FONE_GAP, F("SD Card OK!"));
		    DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Black, LINE_START, 2*HEIGH_12X24+2*FONE_GAP, F("SD Size(M):"));
			DWIN_Draw_IntValue(true, true, 0, font12x24, Color_Bg_Red, Color_Bg_Black, 5, (strlen("SD Size(M):")+1)*WIDTH_12X24, 2*HEIGH_12X24+2*FONE_GAP, CardReader::sd2card.cardSize()/2000);
			thermalManager.temp_hotend[0].target = 60;
			thermalManager.temp_bed.target = 50;
			DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Window, LINE_START, 4*HEIGH_12X24+4*FONE_GAP, F("Hot end Heating..."));
 			DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Window, LINE_START, 6*HEIGH_12X24+6*FONE_GAP, F("Hot bed Heating..."));
      		HMI_flag.test_timer_rg = 0;
			HMI_flag.test_loop_rg++;
		}
		else{
			if(HMI_flag.test_timer_rg++ >= 50) {
				HMI_flag.test_timer_rg = 0;
				HMI_flag.test_dir_fg = !HMI_flag.test_dir_fg;
				if(HMI_flag.test_dir_fg){
					DWIN_Draw_Rectangle(1, Color_Bg_Red, 0, 1*HEIGH_12X24+FONE_GAP, DWIN_WIDTH, 3*HEIGH_12X24+2*FONE_GAP);
					DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Red, LINE_START, 1*HEIGH_12X24+1*FONE_GAP, F("Please insert SD Card!"));
					DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Red, LINE_START, 2*HEIGH_12X24+2*FONE_GAP, F("Or SD Card error!"));
				}
				else{
					DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 1*HEIGH_12X24+FONE_GAP, DWIN_WIDTH, 3*HEIGH_12X24+2*FONE_GAP);
					DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START, 1*HEIGH_12X24+1*FONE_GAP, F("Please insert SD Card!"));
					DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START, 2*HEIGH_12X24+2*FONE_GAP, F("Or SD Card error!"));
				}
			}
		}
	break;

	case CHECK_HOTEND_TEMP:
		if(HMI_flag.test_timer_rg++ > 1500){
			HMI_flag.test_timer_rg = 0;
		    DWIN_Draw_Rectangle(1, Color_Bg_Window, 0, 4*HEIGH_12X24+4*FONE_GAP, DWIN_WIDTH, 5*HEIGH_12X24+4*FONE_GAP);
			if((test_temp_hotend_celsius - test_temp_hotend_first) >= 4) {
				//thermalManager.temp_hotend[0].target = 0;
				DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Window, LINE_START, 4*HEIGH_12X24+4*FONE_GAP, F("Hot end Temp. OK!"));
			}
			else DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Red, LINE_START, 4*HEIGH_12X24+4*FONE_GAP, F("Please check Hot end!"));

			HMI_flag.test_loop_rg++;
			//test_temp_hotbed_first = thermalManager.temp_bed.celsius;
		}
		break;

	case CHECK_HOTBED_TEMP:
		if(HMI_flag.test_timer_rg++ > 1000){
			HMI_flag.test_timer_rg = 0;
		    DWIN_Draw_Rectangle(1, Color_Bg_Window, 0, 6*HEIGH_12X24+6*FONE_GAP, DWIN_WIDTH, 7*HEIGH_12X24+6*FONE_GAP);
			if((test_temp_hotbed_celsius - test_temp_hotbed_first) > 3) {
				thermalManager.temp_bed.target = 0;
				DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Window, LINE_START, 6*HEIGH_12X24+6*FONE_GAP, F("Hot bed Temp. OK!"));
			}
			else DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Red, LINE_START, 6*HEIGH_12X24+6*FONE_GAP, F("Please check Hot bed!"));

			HMI_flag.test_fan_fg = 0;
			HMI_flag.test_loop_rg++;
		}
		break;

	case CHECK_FAN_SPEED:
		thermalManager.fan_speed[0] = 255;
		thermalManager.checkExtruderAutoFans();
		DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Black, LINE_START, 7*HEIGH_12X24+7*FONE_GAP, F("ALL Fan On..."));
		
		if(thermalManager.temp_hotend[0].celsius >= TEST_EXTRUDER_AUTO_FAN_TEMPERATURE + 10){
			HMI_flag.test_timer_rg = 0;
			HMI_flag.test_counter_rg = 0;
			DWIN_Draw_Rectangle(1, Color_Bg_Window, 0, 8*HEIGH_12X24+8*FONE_GAP, DWIN_WIDTH, 11*HEIGH_12X24+10*FONE_GAP);
			thermalManager.temp_hotend[0].target = 0;
			HMI_flag.test_loop_rg++;
		}
		break;

	case CHECK_XY_MOTOR:
		DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Window, LINE_START, 8*HEIGH_12X24+8*FONE_GAP, F("XY Axis Motor On..."));
		if (!planner.is_full()) {
        	planner.synchronize();
        	planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
      	}

		if(HMI_flag.test_timer_rg++ >= 100){
			HMI_flag.test_timer_rg = 0;
			HMI_flag.test_dir_fg = !HMI_flag.test_dir_fg;
		    if(HMI_flag.test_dir_fg) {
				current_position.x += 10;
				#if DISABLED(COREXY)
				current_position.y += 10;
				#endif
		    }
			else {
				#if DISABLED(COREXY)
				current_position.y -= 10;
				#endif
				current_position.x -= 10;
			}
		
			if(HMI_flag.test_counter_rg++ >=5){
				HMI_flag.test_counter_rg = 0;

				DWIN_Draw_Rectangle(1, Color_Bg_Window, 0, 8*HEIGH_12X24+8*FONE_GAP, DWIN_WIDTH, 9*HEIGH_12X24+8*FONE_GAP);
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Window, LINE_START, 8*HEIGH_12X24+8*FONE_GAP, F("XY Axis Motor Off"));
				HMI_flag.test_loop_rg++;
			}
		
		}
		break;

    case CHECK_Z_MOTOR:
		DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START, 9*HEIGH_12X24+9*FONE_GAP, F("Z Axis Motor On..."));
		if (!planner.is_full()) {
        	planner.synchronize();
        	planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
      	}

		if(HMI_flag.test_timer_rg++ >= 100){
			HMI_flag.test_timer_rg = 0;
			HMI_flag.test_dir_fg = !HMI_flag.test_dir_fg;
		    if(HMI_flag.test_dir_fg) current_position.z += 3;
			else current_position.z -= 2;
		
			if(HMI_flag.test_counter_rg++ >=5){
				HMI_flag.test_counter_rg = 0;
				
				DWIN_Draw_Rectangle(1, Color_Bg_Window, 0, 9*HEIGH_12X24+9*FONE_GAP, DWIN_WIDTH, 10*HEIGH_12X24+9*FONE_GAP);
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Window, LINE_START, 9*HEIGH_12X24+9*FONE_GAP, F("Z Axis Motor Off"));
				HMI_flag.test_loop_rg++;
			}
		}
		break;

	case CHECK_MOTOR_E1:
		  DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Window, LINE_START, 10*HEIGH_12X24+10*FONE_GAP, F("Extruder1 Motor On..."));
		  if(HMI_flag.test_timer_rg++ >= 100){
		  	HMI_flag.test_timer_rg = 0;
			HMI_flag.test_dir_fg = !HMI_flag.test_dir_fg;
		    if(HMI_flag.test_dir_fg) {
		  		queue.inject_P("T0\nG92 E0\nG1 E10 F3000");
		    }
			else{
				queue.inject_P("T0\nG92 E0\nG1 E-10 F3000");
			}

			if(HMI_flag.test_counter_rg++ >=3){
				HMI_flag.test_counter_rg = 0;
				
				DWIN_Draw_Rectangle(1, Color_Bg_Window, 0, 10*HEIGH_12X24+10*FONE_GAP, DWIN_WIDTH, 11*HEIGH_12X24+10*FONE_GAP);
				//DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Window, LINE_START, 10*HEIGH_12X24+10*FONE_GAP, F("Extruder1 Motor Off"));
				HMI_flag.test_loop_rg++;
			}
		  }
		break;

	#if (E_STEPPERS > 1)
	case CHECK_MOTOR_E2:
		  DWIN_Draw_String(false, false, font12x24, Color_Yellow, Color_Bg_Window, LINE_START, 10*HEIGH_12X24+10*FONE_GAP, F("Extruder2 Motor On..."));
		  if(HMI_flag.test_timer_rg++ >= 100){
		  	HMI_flag.test_timer_rg = 0;
			HMI_flag.test_dir_fg = !HMI_flag.test_dir_fg;
		    if(HMI_flag.test_dir_fg) {
		  		queue.inject_P("T1\nG92 E0\nG1 E10 F3000");
		    }
			else{
				queue.inject_P("T1\nG92 E0\nG1 E-10 F3000");
			}

			if(HMI_flag.test_counter_rg++ >=3){
				HMI_flag.test_counter_rg = 0;
				
				DWIN_Draw_Rectangle(1, Color_Bg_Window, 0, 10*HEIGH_12X24+10*FONE_GAP, DWIN_WIDTH, 11*HEIGH_12X24+10*FONE_GAP);
				//DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Window, LINE_START, 10*HEIGH_12X24+10*FONE_GAP, F("Extruder2 Motor Off"));
				
				HMI_flag.test_loop_rg++;
			}
		  }
		break;
	#endif

	#if (E_STEPPERS > 2)
	case CHECK_MOTOR_E3:
		  DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Window, LINE_START, 10*HEIGH_12X24+10*FONE_GAP, F("Extruder3 Motor On..."));
		  if(HMI_flag.test_timer_rg++ >= 100){
		  	HMI_flag.test_timer_rg = 0;
			HMI_flag.test_dir_fg = !HMI_flag.test_dir_fg;
		    if(HMI_flag.test_dir_fg) {
		  		queue.inject_P("T2\nG92 E0\nG1 E10 F3000");
		    }
			else{
				queue.inject_P("T2\nG92 E0\nG1 E-10 F3000");
			}

			if(HMI_flag.test_counter_rg++ >=3){
				HMI_flag.test_counter_rg = 0;
				
				DWIN_Draw_Rectangle(1, Color_Bg_Window, 0, 10*HEIGH_12X24+10*FONE_GAP, DWIN_WIDTH, 11*HEIGH_12X24+10*FONE_GAP);
				//DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Window, LINE_START, 10*HEIGH_12X24+10*FONE_GAP, F("Extruder3 Motor Off"));
				
				HMI_flag.test_loop_rg++;
			}
		  }
		break;
	#endif

	#if (E_STEPPERS > 3)
	case CHECK_MOTOR_E4:
		  DWIN_Draw_String(false, false, font12x24, Percent_Color, Color_Bg_Window, LINE_START, 10*HEIGH_12X24+10*FONE_GAP, F("Extruder4 Motor On..."));
		  if(HMI_flag.test_timer_rg++ >= 100){
		  	HMI_flag.test_timer_rg = 0;
			HMI_flag.test_dir_fg = !HMI_flag.test_dir_fg;
		    if(HMI_flag.test_dir_fg) {
		  		queue.inject_P("T3\nG92 E0\nG1 E10 F3000");
		    }
			else{
				queue.inject_P("T3\nG92 E0\nG1 E-10 F3000");
			}

			if(HMI_flag.test_counter_rg++ >=3){
				HMI_flag.test_counter_rg = 0;
				
				DWIN_Draw_Rectangle(1, Color_Bg_Window, 0, 10*HEIGH_12X24+10*FONE_GAP, DWIN_WIDTH, 11*HEIGH_12X24+10*FONE_GAP);
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Window, LINE_START, 10*HEIGH_12X24+10*FONE_GAP, F("ALL Extruder Motor Off"));

				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START, 11*HEIGH_12X24+11*FONE_GAP, F("Sw:"));
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START+3*WIDTH_12X24+0*WORD_WIDTH, 11*HEIGH_12X24+11*FONE_GAP, F("X"));
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START+3*WIDTH_12X24+1*WORD_WIDTH, 11*HEIGH_12X24+11*FONE_GAP, F("Y"));
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START+3*WIDTH_12X24+2*WORD_WIDTH, 11*HEIGH_12X24+11*FONE_GAP, F("Z1"));
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START+3*WIDTH_12X24+3*WORD_WIDTH, 11*HEIGH_12X24+11*FONE_GAP, F("Z2"));
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START+3*WIDTH_12X24+4*WORD_WIDTH, 11*HEIGH_12X24+11*FONE_GAP, F("F"));
				HMI_flag.Is_X_Endstop = 0;
				HMI_flag.Is_Y_Endstop = 0;
				HMI_flag.Is_Z1_Endstop = 0;
				HMI_flag.Is_Z2_Endstop = 0;
				HMI_flag.Is_F_Endstop = 0;
				DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 12*HEIGH_12X24+12*FONE_GAP, DWIN_WIDTH, 13*HEIGH_12X24+12*FONE_GAP);
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START, 12*HEIGH_12X24+12*FONE_GAP, F("St:"));
				HMI_flag.test_loop_rg++;
				thermalManager.temp_bed.target = 0;
			}
		  }
		break;
	#endif

	case CHECK_ENDSTOPS:
	    if(HMI_flag.test_timer_rg++ >= 10){
		  	HMI_flag.test_timer_rg = 0;
			
			if(READ(X_STOP_PIN)&&(!HMI_flag.Is_X_Endstop)){
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START+3*WIDTH_12X24+0*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, F("Off"));
			}
			else {
				delay(10);
				if(!READ(X_STOP_PIN)){
					if(!HMI_flag.Is_X_Endstop)DWIN_Draw_Rectangle(1, Color_Bg_Black, LINE_START+3*WIDTH_12X24+0*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, LINE_START+3*WIDTH_12X24+0*WORD_WIDTH+3*WIDTH_12X24, 13*HEIGH_12X24+12*FONE_GAP);
					HMI_flag.Is_X_Endstop = 1;
					DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Black, LINE_START+3*WIDTH_12X24+0*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, F("OK"));
				}
			}
		
			if(READ(Y_STOP_PIN)&&(!HMI_flag.Is_Y_Endstop)){
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START+3*WIDTH_12X24+1*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, F("Off"));
			}
			else {
				delay(10);
				if(!READ(Y_STOP_PIN)){
					if(!HMI_flag.Is_Y_Endstop)DWIN_Draw_Rectangle(1, Color_Bg_Black, LINE_START+3*WIDTH_12X24+1*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, LINE_START+3*WIDTH_12X24+1*WORD_WIDTH+3*WIDTH_12X24, 13*HEIGH_12X24+12*FONE_GAP);
					HMI_flag.Is_Y_Endstop = 1;
					DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Black, LINE_START+3*WIDTH_12X24+1*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, F("OK"));
				}
			}
		
			if(READ(Z_MIN_PIN)&&(!HMI_flag.Is_Z1_Endstop)){
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START+3*WIDTH_12X24+2*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, F("Off"));
			}
			else {
				delay(10);
				if(!READ(Z_MIN_PIN)){
					if(!HMI_flag.Is_Z1_Endstop)DWIN_Draw_Rectangle(1, Color_Bg_Black, LINE_START+3*WIDTH_12X24+2*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, LINE_START+3*WIDTH_12X24+2*WORD_WIDTH+3*WIDTH_12X24, 13*HEIGH_12X24+12*FONE_GAP);
					HMI_flag.Is_Z1_Endstop = 1;
					DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Black, LINE_START+3*WIDTH_12X24+2*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, F("OK"));
				}
			}
		
			#if PIN_EXISTS(Z2_MIN)
			if(READ(Z2_MIN_PIN)&&(!HMI_flag.Is_Z2_Endstop)){
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START+3*WIDTH_12X24+3*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, F("Off"));
			}
			else {
				delay(10);
				if(!READ(Z2_MIN_PIN)){
					if(!HMI_flag.Is_Z2_Endstop)DWIN_Draw_Rectangle(1, Color_Bg_Black, LINE_START+3*WIDTH_12X24+3*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, LINE_START+3*WIDTH_12X24+3*WORD_WIDTH+3*WIDTH_12X24, 13*HEIGH_12X24+12*FONE_GAP);
					HMI_flag.Is_Z2_Endstop = 1;
					DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Black, LINE_START+3*WIDTH_12X24+3*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, F("OK"));
				}
			}
			#endif

			#if ENABLED(FILAMENT_RUNOUT_SENSOR)
			if(READ(FIL_RUNOUT_PIN)&&(!HMI_flag.Is_F_Endstop)){
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, LINE_START+3*WIDTH_12X24+4*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, F("Off"));
			}
			else {
				delay(10);
				if(!READ(FIL_RUNOUT_PIN)){
					if(!HMI_flag.Is_F_Endstop)DWIN_Draw_Rectangle(1, Color_Bg_Black, LINE_START+3*WIDTH_12X24+4*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, LINE_START+3*WIDTH_12X24+4*WORD_WIDTH+3*WIDTH_12X24, 13*HEIGH_12X24+12*FONE_GAP);
					HMI_flag.Is_F_Endstop = 1;
					DWIN_Draw_String(false, false, font12x24, Color_Bg_Red, Color_Bg_Black, LINE_START+3*WIDTH_12X24+4*WORD_WIDTH, 12*HEIGH_12X24+12*FONE_GAP, F("OK"));
				}
			}
            #endif
			
			if(HMI_flag.Is_X_Endstop&&HMI_flag.Is_Y_Endstop&&HMI_flag.Is_Z1_Endstop&&HMI_flag.Is_Z2_Endstop&&HMI_flag.Is_F_Endstop){
				HMI_flag.test_loop_rg++;
				HMI_flag.test_dir_fg = 0;
			    HMI_flag.test_stop_rg = 0;
			}
	    }
		break;

	case CHECK_KEY:
		if(!HMI_flag.test_dir_fg) {
			DWIN_Draw_Rectangle(1, Color_Bg_Blue, LINE_START+DWIN_WIDTH/2+(strlen("Buzzer:"))*WIDTH_12X24, DWIN_HEIGHT-HEIGH_12X24, LINE_START+DWIN_WIDTH/2+(strlen("Buzzer:"))*WIDTH_12X24+3*WIDTH_12X24, DWIN_HEIGHT);
			DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Blue, LINE_START+DWIN_WIDTH/2+(strlen("Buzzer:"))*WIDTH_12X24, DWIN_HEIGHT-HEIGH_12X24, F("Off"));
		}
		
		HMI_flag.test_dir_fg = 1;
		if(HMI_flag.test_rotary_fg){
			if(HMI_flag.test_timer_rg++ >= 100){
				HMI_flag.test_timer_rg = 0;
				HMI_flag.test_dir_fg = 0;
				HMI_flag.test_rotary_fg = 0;
				DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Blue, LINE_START+DWIN_WIDTH/2+(strlen("Buzzer:"))*WIDTH_12X24, DWIN_HEIGHT-HEIGH_12X24, F("Off"));

				if((HMI_flag.test_stop_rg++ > 3) && (HMI_flag.test_rotary_counter_rg !=0)) {
					HMI_flag.test_loop_rg++;					
					HMI_flag.test_stop_rg = 0;					
				}
			}
		}
		break;
	
	default:
	case CHECK_START:
		HMI_flag.test_loop_rg++;
		HMI_flag.test_stop_rg = 0;
		break;

	case CHECK_END:
		//queue.inject_P(PSTR("M81"));
		HMI_flag.auto_test_flag = 0x55;
		HMI_flag.test_loop_rg = 0;
		Goto_MainMenu();
		break;

  }	
}

void HMI_StartTest() {
	HMI_flag.auto_test_flag = 0xaa;
	HMI_flag.test_loop_rg = 0;

	DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 0, DWIN_WIDTH, DWIN_HEIGHT);
	DWIN_Draw_Rectangle(1, Color_Bg_Blue, 0, 0, DWIN_WIDTH, HEIGH_12X24);
	DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Black, DWIN_WIDTH/2-(strlen("3D Printer Test")*WIDTH_12X24)/2, 0, F("3D Printer Test"));
	DWIN_Draw_Rectangle(1, Color_Bg_Blue, 0, DWIN_HEIGHT-HEIGH_12X24, DWIN_WIDTH, DWIN_HEIGHT);
	DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Blue, LINE_START, DWIN_HEIGHT-HEIGH_12X24, F("Rotary:"));
	DWIN_Draw_IntValue(true, true, 0, font12x24, Color_White, Color_Bg_Blue, 3, LINE_START+(strlen("Rotary:"))*WIDTH_12X24, DWIN_HEIGHT-HEIGH_12X24, HMI_flag.test_rotary_counter_rg);
	DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Blue, LINE_START+DWIN_WIDTH/2, DWIN_HEIGHT-HEIGH_12X24, F("Buzzer:"));
	DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Blue, LINE_START+DWIN_WIDTH/2+(strlen("Buzzer:"))*WIDTH_12X24, DWIN_HEIGHT-HEIGH_12X24, F("Off"));
	thermalManager.temp_hotend[0].target = 0;
	thermalManager.temp_bed.target = 0;
	DWIN_Draw_Rectangle(1, Color_Bg_Window, 0, 3*HEIGH_12X24+3*FONE_GAP, DWIN_WIDTH, 7*HEIGH_12X24+6*FONE_GAP);
	DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Red, LINE_START, 3*HEIGH_12X24+3*FONE_GAP, F("Hot end:"));
	DWIN_Draw_IntValue(true, true, 0, font12x24, Color_White, Color_Bg_Window, 3, DWIN_WIDTH/2, 3*HEIGH_12X24+3*FONE_GAP, thermalManager.temp_hotend[0].target);
	DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Red, DWIN_WIDTH/2+3*WIDTH_12X24, 3*HEIGH_12X24+3*FONE_GAP, F("/"));
	DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Red, LINE_START, 5*HEIGH_12X24+5*FONE_GAP, F("Hot bed:"));
	DWIN_Draw_IntValue(true, true, 0, font12x24, Color_White, Color_Bg_Window, 3, DWIN_WIDTH/2, 5*HEIGH_12X24+5*FONE_GAP, thermalManager.temp_bed.target);
	DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Red, DWIN_WIDTH/2+3*WIDTH_12X24, 5*HEIGH_12X24+5*FONE_GAP, F("/"));
	test_temp_hotend_first = thermalManager.temp_hotend[0].celsius;
	test_temp_hotbed_first = thermalManager.temp_bed.celsius;
}

void Check_Rotary(){
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;
	
	if (encoder_diffState == ENCODER_DIFF_CW) HMI_flag.test_rotary_counter_rg++;
	else if (encoder_diffState == ENCODER_DIFF_CCW) HMI_flag.test_rotary_counter_rg--;
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		DWIN_Draw_Rectangle(1, Color_Bg_Blue, LINE_START+DWIN_WIDTH/2+(strlen("Buzzer:"))*WIDTH_12X24, DWIN_HEIGHT-HEIGH_12X24, LINE_START+DWIN_WIDTH/2+(strlen("Buzzer:"))*WIDTH_12X24+3*WIDTH_12X24, DWIN_HEIGHT);
		DWIN_Draw_String(false, false, font12x24, Color_White, Color_Bg_Blue, LINE_START+DWIN_WIDTH/2+(strlen("Buzzer:"))*WIDTH_12X24, DWIN_HEIGHT-HEIGH_12X24, F("On"));
		buzzer.tone(200, 3000);
		HMI_flag.test_rotary_fg = 1;
	}
	DWIN_Draw_IntValue(true, true, 0, font12x24, Color_White, Color_Bg_Blue, 3, LINE_START+(strlen("Rotary:"))*WIDTH_12X24, DWIN_HEIGHT-HEIGH_12X24, HMI_flag.test_rotary_counter_rg);
}

#endif // DWIN_CREALITY_LCD
