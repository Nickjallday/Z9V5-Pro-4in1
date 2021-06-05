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
#pragma once

/**
 * DWIN by Creality3D
 */

#include "../dwin_lcd.h"
#include "rotary_encoder.h"
#include "../../../libs/BL24CXX.h"

#include "../../../inc/MarlinConfigPre.h"

#if ANY(HAS_HOTEND, HAS_HEATED_BED, HAS_FAN) && PREHEAT_COUNT
  #define HAS_PREHEAT 1
  #if PREHEAT_COUNT < 2
    #error "Creality DWIN requires two material preheat presets."
  #endif
#endif

#define VTOOL_START 0
#define VTOOL_END   1
#define ZPOS_START 0
#define ZPOS_END   1
#define RANDMIX_HEIGHT   3
#define RANDMIX_EXTRUDES   4



#if HAS_SUICIDE
	#define POWERDOWN_MACHINE_TIMER 900
#endif

enum processID : uint8_t {
  // Process ID
  MainMenu,
  SelectFile,
  Prepare,
  Control,
  Leveling,
  Leveling0,
  Home,
  PrintProcess,
  AxisMove,
  TemperatureID,
  Motion,
  DMixer,
  Config,
  Info,
  Tune,
  #if HAS_PREHEAT
    PLAPreheat,
    ABSPreheat,
  #endif
  MaxSpeed,
  MaxSpeed_value,
  MaxAcceleration,
  MaxAcceleration_value,
  MaxJerk,
  MaxJerk_value,
  Step,
  Step_value,

  // Last Process ID
  Last_Prepare,

  // Back Process ID
  Back_Main,
  Back_Print,

  // Date variable ID
  Move_X,
  Move_Y,
  Move_Z,
#if HAS_HOTEND
  Extruder1,
  #if(E_STEPPERS > 1)
  Extruder2,
  #endif
  #if(E_STEPPERS > 2)
  Extruder3,
  #endif
  #if(E_STEPPERS > 3)
  Extruder4,
  #endif
  #if ENABLED(MIXING_EXTRUDER) 
  ExtruderAll,
  #endif
  ETemp,
#endif//#if HAS_HOTEND
  Homeoffset,
  #if HAS_HEATED_BED
    BedTemp,
  #endif
  #if HAS_FAN
    FanSpeed,
  #endif
  PrintSpeed,

  #if ENABLED(MIXING_EXTRUDER)
  Mix_Manual,
  Mix_Auto,
  Mix_Random,
  Mix_Vtool,
  #endif 

  Mix_Manual_Extruder1,
  Mix_Manual_Extruder2,
  Mix_Manual_Extruder3,
  Mix_Manual_Extruder4,
  Mix_Manual_OK,

  Auto_Zpos_Start,
  Auto_Zpos_End,
  Mix_VTool_Start,
  Mix_VTool_End,

  Random_Zpos_Start,
  Random_Zpos_End,
  Random_Height,
  Random_Extruders,

  Filament_Option,

  Powerdown,

  Bltouch,

  Language,

  Last_Level_CatchPop,
  Last_Level_CatchOffset,
  Last_Leveling,

  Babysteps,

  #if ENABLED(FWRETRACT) 
  	Retract,
  	Retract_MM,
  	Retract_V,
  	Retract_ZHOP,
  	UnRetract_MM,
  	UnRetract_V,
  #endif

  #if ENABLED(OPTION_WIFI_MODULE)
  	Wifi,
  #endif

  #if ENABLED(OPTION_REPEAT_PRINTING)
  	Re_print,
  	Reprint_times,
  	Forward_lenght,
  #endif

  #if ENABLED(OPTION_BED_COATING)
    COATING,
  #endif
  
  // Window ID
  Print_window,
  Popup_Window
};

// Picture ID
#define Start_Process       0
#define Language_English    1
#define Language_Chinese    2

// ICON ID
#define ICON                      0x08
#define ICON_LOGO                  0
#define ICON_Print_0               1
#define ICON_Print_1               2
#define ICON_Prepare_0             3
#define ICON_Prepare_1             4
#define ICON_Control_0             5
#define ICON_Control_1             6
#define ICON_Leveling_0            7
#define ICON_Leveling_1            8
#define ICON_HotendTemp            9
#define ICON_BedTemp              10
#define ICON_Speed                11
#define ICON_Zoffset              12
#define ICON_Back                 13
#define ICON_File                 14
#define ICON_PrintTime            15
#define ICON_RemainTime           16
#define ICON_Setup_0              17
#define ICON_Setup_1              18
#define ICON_Pause_0              19
#define ICON_Pause_1              20
#define ICON_Continue_0           21
#define ICON_Continue_1           22
#define ICON_Stop_0               23
#define ICON_Stop_1               24
#define ICON_Bar                  25
#define ICON_More                 26

#define ICON_Axis                 27
#define ICON_CloseMotor           28
#define ICON_Homing               29
#define ICON_SetHome              30
#define ICON_PLAPreheat           31
#define ICON_ABSPreheat           32
#define ICON_Cool                 33
#define ICON_Language             34

#define ICON_MoveX                35
#define ICON_MoveY                36
#define ICON_MoveZ                37
#define ICON_Extruder             38

#define ICON_Temperature          39
#define ICON_Motion               40
#define ICON_Mixer                41
#define ICON_BLTouch              42
#define ICON_WriteEEPROM          43
#define ICON_ReadEEPROM           44
#define ICON_ResumeEEPROM         45
#define ICON_Info                 46

#define ICON_SetEndTemp           47
#define ICON_SetBedTemp           48
#define ICON_FanSpeed             49
#define ICON_SetPLAPreheat        50
#define ICON_SetABSPreheat        51

#define ICON_MaxSpeed             52
#define ICON_MaxAccelerated       53
#define ICON_MaxJerk              54
#define ICON_Step                 55
#define ICON_PrintSize            56
#define ICON_Version              57
#define ICON_Contact              58
#define ICON_StockConfiguraton    59
#define ICON_MaxSpeedX            60
#define ICON_MaxSpeedY            61
#define ICON_MaxSpeedZ            62
#define ICON_MaxSpeedE            63
#define ICON_MaxAccX              64
#define ICON_MaxAccY              65
#define ICON_MaxAccZ              66
#define ICON_MaxAccE              67
#define ICON_MaxSpeedJerkX        68
#define ICON_MaxSpeedJerkY        69
#define ICON_MaxSpeedJerkZ        70
#define ICON_MaxSpeedJerkE        71
#define ICON_StepX                72
#define ICON_StepY                73
#define ICON_StepZ                74
#define ICON_StepE                75

#define ICON_Setspeed             76
#define ICON_SetZOffset           77
#define ICON_Rectangle            78

#define ICON_TempTooLow           79
#define ICON_AutoLeveling         80
#define ICON_TempTooHigh          81
#define ICON_NoTips_C             82
#define ICON_NoTips_E             83
#define ICON_Continue_C           84
#define ICON_Continue_E           85
#define ICON_Cancel_C             86
#define ICON_Cancel_E             87
#define ICON_Confirm_C            88
#define ICON_Confirm_E            89
#define ICON_Info_0               90
#define ICON_Info_1               91

#define ICON_Mixer_Manual         92
#define ICON_Mixer_Auto           93
#define ICON_Mixer_Random         94

#define ICON_Extruder1            95
#define ICON_Extruder2            96
#define ICON_Extruder3            97
#define ICON_Extruder4            98

#define ICON_C_VTOOL              99
#define ICON_S_VTOOL              100

#define ICON_Extruder1_P          101
#define ICON_Extruder2_P          102
#define ICON_Extruder3_P          103
#define ICON_Extruder4_P          104
#define ICON_VTool_P              105

#define ICON_VTool_Drop           106
#define ICON_VTool_Rise           107
#define ICON_Zpos_Drop            108
#define ICON_Zpos_Rise            109

#define ICON_Leveling0            110
#define ICON_Leveling_Point1      111
#define ICON_Leveling_Point2      112
#define ICON_Leveling_Point3      113
#define ICON_Leveling_Point4      114
#define ICON_Leveling_Auto        115
#define ICON_Leveling_Save        116

#define ICON_Blank      		  117
#define ICON_Cursor      		  118

#define ICON_HOME_ALL      		  119
#define ICON_HOME_X      		  120
#define ICON_HOME_Y      		  121
#define ICON_HOME_Z      		  122

#define ICON_FIL_OPTION      	  123

#define ICON_YES_0      	  	  124
#define ICON_YES_1      	  	  125

#define ICON_NO_0      	  		  126
#define ICON_NO_1      	  		  127

#define ICON_BLTOUCH_RESET        128
#define ICON_BLTOUCH_TEST         129
#define ICON_BLTOUCH_STOW         130
#define ICON_BLTOUCH_DEPLOY       131
#define ICON_BLTOUCH_SW           132

#define ICON_POWER_DOWN           133
#define ICON_POWERDOWN            134

#define ICON_REMAIN_TIME          135
#define ICON_PRINT_TIME           136


#define ICON_EN					  140
#define ICON_SP					  141
#define ICON_RU					  142
#define ICON_FR					  143
#define ICON_PO					  144
#define ICON_CH					  145

#define ICON_WIFI				  146

#define ICON_PRINT_EN             150
#define ICON_PREPARE_EN           151
#define ICON_CONTROL_EN           152
#define ICON_STARTINFO_EN         153
#define ICON_TUNE_EN   	  		  154
#define ICON_PAUSE_EN   	  	  155
#define ICON_STOP_EN   	  		  156
#define ICON_CONTINUE_EN   	  	  ICON_PRINT_EN

#define ICON_PRINT_SP             160
#define ICON_PREPARE_SP           161
#define ICON_CONTROL_SP           162
#define ICON_STARTINFO_SP         163
#define ICON_TUNE_SP   	  		  164
#define ICON_PAUSE_SP   	  	  165
#define ICON_STOP_SP   	  		  166
#define ICON_CONTINUE_SP   	  	  ICON_PRINT_SP

#define ICON_PRINT_RU             170
#define ICON_PREPARE_RU           171
#define ICON_CONTROL_RU           172
#define ICON_STARTINFO_RU         173
#define ICON_TUNE_RU   	  		  174
#define ICON_PAUSE_RU   	  	  175
#define ICON_STOP_RU   	  		  176
#define ICON_CONTINUE_RU   	  	  ICON_PRINT_RU

#define ICON_PRINT_FR             180
#define ICON_PREPARE_FR           181
#define ICON_CONTROL_FR           182
#define ICON_STARTINFO_FR         183
#define ICON_TUNE_FR   	  		  184
#define ICON_PAUSE_FR   	  	  185
#define ICON_STOP_FR   	  		  186
#define ICON_CONTINUE_FR   	  	  187

#define ICON_PRINT_PT             190
#define ICON_PREPARE_PT           191
#define ICON_CONTROL_PT           192
#define ICON_STARTINFO_PT         193
#define ICON_TUNE_PT   	  		  194
#define ICON_PAUSE_PT   	  	  195
#define ICON_STOP_PT   	  		  196
#define ICON_CONTINUE_PT   	  	  197



/**
 * 3-.0：The font size, 0x00-0x09, corresponds to the font size below:
 * 0x00=6*12   0x01=8*16   0x02=10*20  0x03=12*24  0x04=14*28
 * 0x05=16*32  0x06=20*40  0x07=24*48  0x08=28*56  0x09=32*64
 */
#define font6x12  0x00
#define font8x16  0x01
#define font10x20 0x02
#define font12x24 0x03
#define font14x28 0x04
#define font16x32 0x05
#define font20x40 0x06
#define font24x48 0x07
#define font28x56 0x08
#define font32x64 0x09

// Color
#define Color_White       0xFFFF
#define Color_Yellow      0xFF0F
#define Color_Green       0x07E0
#define Color_Red         0xF00F  // Red background color
#define Color_Bg_Window   0x31E8  //0x6516  //0x31E8// Popup background color//
#define Color_Bg_Blue     0x1125  //1125// Dark blue background color
#define Color_Bg_Black    0x0000  //841// Black background color      
#define Color_Bg_Red      0xF00F  // Red background color
#define Popup_Text_Color  0xD6BA  // Popup font background color
#define Line_Color        0x3A6A  // Split line color
#define Rectangle_Color   0xEE2F  // Blue square cursor color
#define Percent_Color     0xFE29  // Percentage color
#define BarFill_Color     0x10E4  // Fill color of progress bar
#define Select_Color      0x33BB  // Selected color

//#define DEBUG_FILAMENT_RUNOUT 
//#define DEBUG_POWER_LOSS
//
// logo  offset define
//
#define Logo_offset_X         		20
//#define Logo_offset_Y         		STATUS_Y_END + 10
#define Logo_offset_Y         		45

//
// Status Area offset define
//
#define State_space_Y         		20
#define State_icon_offset_X    		13
#define State_icon_offset_Y    		381
#define State_text_offset_X    		State_icon_offset_X + STAT_CHR_W*2
#define State_text_offset_Y    		State_icon_offset_Y + 1
#define State_string_offset_X    	State_icon_offset_X + STAT_CHR_W*5
#define State_string_offset_Y    	State_icon_offset_Y + 2

#define State_text_extruder_num		3
#define State_icon_extruder_X 		State_icon_offset_X
#define State_icon_extruder_Y		State_icon_offset_Y
#define State_text_extruder_X		State_text_offset_X
#define State_text_extruder_Y		State_text_offset_Y
#define State_string_extruder_X		State_string_offset_X
#define State_string_extruder_Y		State_string_offset_Y

#define State_text_bed_num			3
#define State_icon_bed_X 			State_icon_offset_X + DWIN_WIDTH/2
#define State_icon_bed_Y			State_icon_offset_Y
#define State_text_bed_X			State_text_offset_X	+ DWIN_WIDTH/2
#define State_text_bed_Y			State_text_offset_Y
#define State_string_bed_X			State_string_offset_X + DWIN_WIDTH/2
#define State_string_bed_Y			State_string_offset_Y

#define State_text_speed_num		3
#define State_icon_speed_X 			State_icon_offset_X
#define State_icon_speed_Y			State_icon_offset_Y + STAT_CHR_H + State_space_Y
#define State_text_speed_X			State_text_offset_X
#define State_text_speed_Y			State_text_offset_Y + STAT_CHR_H + State_space_Y
#define State_string_speed_X		State_string_offset_X
#define State_string_speed_Y		State_string_offset_Y + STAT_CHR_H + State_space_Y

#define State_text_Zoffset_inum		3
#define State_text_Zoffset_fnum		2
#define State_icon_Zoffset_X 		State_icon_offset_X + DWIN_WIDTH/2
#define State_icon_Zoffset_Y		State_icon_offset_Y + STAT_CHR_H + State_space_Y
#define State_text_Zoffset_X		State_text_offset_X + DWIN_WIDTH/2
#define State_text_Zoffset_Y		State_text_offset_Y + STAT_CHR_H + State_space_Y
#define State_string_Zoffset_X		State_string_offset_X + DWIN_WIDTH/2
#define State_string_Zoffset_Y		State_string_offset_Y + STAT_CHR_H + State_space_Y

#define State_text_vtool_num		2

#define State_text_mix_num			3
#define State_icon_mix_X 			13
#define State_icon_mix_Y			45
#define State_text_mix_X			13
#define State_text_mix_Y			State_icon_mix_Y + 25

//
// Menu Area offset define
//
#define Menu_control_start_temp_X		57
#define Menu_control_start_temp_Y		104
#define Menu_control_end_temp_X			84
#define Menu_control_end_temp_Y			116

#define Menu_control_start_motion_X		Menu_control_start_temp_X + 30
#define Menu_control_start_motion_Y		Menu_control_start_temp_Y
#define Menu_control_end_motion_X		Menu_control_end_temp_X + 30
#define Menu_control_end_motion_Y		Menu_control_end_temp_Y

#define Menu_control_start_mixer_X		Menu_control_start_motion_X + 30
#define Menu_control_start_mixer_Y		Menu_control_start_motion_Y
#define Menu_control_end_mixer_X		Menu_control_end_motion_X + 30
#define Menu_control_end_mixer_Y		Menu_control_end_motion_Y

#define Menu_control_start_store_X		Menu_control_start_mixer_X + 30
#define Menu_control_start_store_Y		Menu_control_start_mixer_Y
#define Menu_control_end_store_X		Menu_control_end_mixer_X + 30
#define Menu_control_end_store_Y		Menu_control_end_mixer_Y

#define Menu_control_start_read_X		Menu_control_start_store_X + 30
#define Menu_control_start_read_Y		Menu_control_start_store_Y
#define Menu_control_end_read_X			Menu_control_end_store_X + 30
#define Menu_control_end_read_Y			Menu_control_end_store_Y

#define Menu_control_start_reset_X		Menu_control_start_read_X + 30
#define Menu_control_start_reset_Y		Menu_control_start_read_Y
#define Menu_control_end_reset_X		Menu_control_end_read_X + 30
#define Menu_control_end_reset_Y		Menu_control_end_read_Y

#define Menu_control_start_info_X		Menu_control_start_reset_X + 30
#define Menu_control_start_info_Y		Menu_control_start_reset_Y
#define Menu_control_end_info_X			Menu_control_end_reset_X + 30
#define Menu_control_end_info_Y			Menu_control_end_reset_Y

extern uint8_t checkkey;
extern float zprobe_zoffset;
extern char print_filename[16];

extern millis_t dwin_heat_time;

typedef struct {
  TERN_(HAS_HOTEND,     int16_t E_Temp    = 0);
  TERN_(HAS_HEATED_BED, int16_t Bed_Temp  = 0);
  TERN_(HAS_PREHEAT,    int16_t Fan_speed = 0);
  int16_t print_speed     = 100;
  float Max_Feedspeed     = 0;
  float Max_Acceleration  = 0;
  float Max_Jerk          = 0;
  float Max_Step          = 0;
  float Move_X_scale      = 0;
  float Move_Y_scale      = 0;
  float Move_Z_scale      = 0;
  float Auto_Zstart_scale  = 0;
  float Auto_Zend_scale    = 0;
  float Random_Zstart_scale  = 0;
  float Random_Zend_scale    = 0;
  float Random_Height = 0;
	#if ENABLED(OPTION_BED_COATING)
  int16_t coating_thickness = 0;
	#endif
  char Random_Extruders = 0;
  float Retract_MM_scale      = 0;
  float Retract_V_scale      = 0;
  float unRetract_MM_scale      = 0;
  float unRetract_V_scale      = 0;
  #if HAS_HOTEND
    float Move_E_scale    = 0;
    float Move_E1_scale    = 0;  
	#if(E_STEPPERS > 1)
	float Move_E2_scale    = 0;
	#endif
	#if(E_STEPPERS > 2)
	float Move_E3_scale    = 0;
	#endif
	#if(E_STEPPERS > 3)
	float Move_E4_scale    = 0;
	#endif
	#if ENABLED(MIXING_EXTRUDER)
	float Move_EAll_scale    = 0;
	#endif		
  #endif
  int16_t zoffset_value      = 0;
  char show_mode          = 0;    // -1: Temperature control    0: Printing temperature
} HMI_value_t;

#define DWIN_CHINESE 123
#define DWIN_ENGLISH 0

enum {
    CHECK_START = 0,
	CHECK_SD,
	CHECK_HOTBED_TEMP,
	CHECK_HOTEND_TEMP,
	CHECK_FAN_SPEED,
	
	CHECK_XY_MOTOR,
	CHECK_Z_MOTOR,

	CHECK_MOTOR_E1,
	#if (E_STEPPERS > 1)
	CHECK_MOTOR_E2,
	#endif
	#if (E_STEPPERS > 2)
	CHECK_MOTOR_E3,
	#endif
	#if (E_STEPPERS > 3)
	CHECK_MOTOR_E4,
	#endif
	CHECK_ENDSTOPS,
	
	CHECK_KEY,
	CHECK_END
};

#if HAS_COLOR_LEDS
enum RGB_LED_COLOR : uint8_t {
	RED_ON,
	GREEN_ON,
	BLUE_ON,
	YELLOW_ON,
	CYAN_ON,
	PURPLE_ON,
	WHITE_ON,
	LOOP_ON,
	BLACK_ON
};
#endif

enum DWIN_G29_MESSAGE : uint8_t {
	G29_LEVLE_DEFAULT,
	G29_CATCH_START,
	G29_CATCH_NORMAL,
	G29_CATCH_PROBE_TOO_HIGH,
	G29_CATCH_FAIL,
	G29_CATCH_DONE,
	G29_MESH_START,
	G29_MESH_NORMAL,
	G29_MESH_VALUE,
	G29_MESH_DONE
};

enum filament_runout : uint8_t {
	FILAMENT_RUNOUT_START=1,
	FILAMENT_RUNOUT_HOME,
	FILAMENT_RUNOUT_UNLOAD,
	FILAMENT_RUNOUT_LOAD,
	//FILAMENT_RUNOUT_WAIT,
	FILAMENT_RUNOUT_END
};

enum DWINPauseMode : char {
  DWIN_PAUSE_MODE_SAME,
  //DWIN_PAUSE_MODE_STATE,
  DWIN_PAUSE_MODE_PAUSE_PRINT,
  DWIN_PAUSE_MODE_CHANGE_FILAMENT,
  DWIN_PAUSE_MODE_LOAD_FILAMENT,
  DWIN_PAUSE_MODE_INSERT_FILAMENT,
  DWIN_PAUSE_MODE_UNLOAD_FILAMENT,
  DWIN_PAUSE_MODE_RESUME_FILAMENT,
  DWIN_PAUSE_MODE_PURGE_FILAMENT,
  DWIN_PAUSE_MODE_OPTION_FILAMENT
};

enum DWINPauseMessage : char {
  DWIN_PAUSE_MESSAGE_PARKING,
  DWIN_PAUSE_MESSAGE_CHANGING,
  DWIN_PAUSE_MESSAGE_WAITING,
  DWIN_PAUSE_MESSAGE_UNLOAD,
  DWIN_PAUSE_MESSAGE_INSERT,
  DWIN_PAUSE_MESSAGE_LOAD,
  DWIN_PAUSE_MESSAGE_PURGE,
  DWIN_PAUSE_MESSAGE_OPTION,
  DWIN_PAUSE_MESSAGE_RESUME,
  DWIN_PAUSE_MESSAGE_STATUS,
  DWIN_PAUSE_MESSAGE_HEAT,
  DWIN_PAUSE_MESSAGE_HEATING
};

typedef struct Mixer_Display_cfg{
	uint16_t Area_X_Start = 10;
	uint16_t Area_X_End = 267;
	uint16_t Area_Y_Start = 143;
	uint16_t Area_Y_End = 171;
	uint16_t Extruder_X_Coordinate[MIXING_STEPPERS] = {0};
	uint8_t Extruder_Int_Number[MIXING_STEPPERS] = {0};
	uint16_t VTool_X_Coordinate = 0;
	uint8_t VTool_Int_Number = 0;
	uint8_t Extruder_X_Start_Coordinate[5] = {0,0,34,19,8};
	uint8_t Extruder_X_Start_Gap[5] = {0,0,78,63,52};
	uint8_t Y_Coordinate = 143;
}MIXER_DIS;
extern MIXER_DIS MixerDis;

typedef struct Mixer_Print_cfg{
	uint8_t Mixer_Mode_Rg;
	float  Zpos_Buff = 0;
	int8_t Current_Percent[MIXING_STEPPERS] = {0};
	int8_t Manual_Percent[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS] = {0};
	int8_t Start_Percent[MIXING_STEPPERS] = {0};
	int8_t End_Percent[MIXING_STEPPERS] = {0};
	int8_t Auto_Percent[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS] = {0};
	//int8_t occupy_vtool = MIXING_VIRTUAL_TOOLS - 1;
	uint8_t Vtool_Backup = 0;
}MIXER_CFG;
extern MIXER_CFG MixerCfg;

typedef struct Filament_Runout_cfg{
	bool    Puge_More_Yes = 0;
	bool    Puge_More_No = 0;
	uint8_t Font = font12x24;
	uint8_t Font_W = 12;
	uint8_t Font_H = 24;
	uint16_t Text_Color = Popup_Text_Color;
	uint16_t Window_Color = Color_Bg_Window;
	uint16_t Text_Pos_Y = 240;
}FIL_CFG;
extern FIL_CFG FIL;

typedef struct {
  uint8_t language;
  uint8_t Title_Menu_Backup = 0;
  
  uint16_t Y_Coordinate;
  bool refersh_mix_flag:1;
  bool pause_flag:1;
  bool pause_action:1;
  bool print_finish:1;
  bool done_confirm_flag:1;
  bool select_flag:1;
  bool heat_flag:1;  // 0: heating done  1: during heating
  bool filament_runout_star:1; // 0: runout normal operation  1: runout response start
  bool filament_runout_end:1; // 0: runout normal operation  1: runout response end
  bool Is_Mixer_Print:1;
  bool need_home_flag:1;
  
  #if ENABLED(OPTION_WIFI_MODULE)
  bool wifi_Handshake_ok:1;
  uint8_t wifi_link_timer;
  #endif

  
  #if ENABLED(LCD_BED_LEVELING)
  bool Auto_Leveling_Menu_Fg;
  uint8_t Leveling_Case_Total = 4;
  #endif

  float current_zpos_backup;
  float pause_zpos_backup;

  float last_E1_Coordinate;
  float last_E2_Coordinate;
  float last_E3_Coordinate;
  float last_E4_Coordinate;
  float last_EALL_Coordinate;
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    bool ETempTooLow_flag:1;
  #endif
  #if HAS_FAN
    AxisEnum feedspeed_axis;
  #endif
  #if HAS_SUICIDE
    bool putdown_close_machine:1;
    bool auto_shutdown:1;
    uint16_t putdown_close_timer_rg = POWERDOWN_MACHINE_TIMER;
	uint16_t free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
  #endif 

  #if HAS_COLOR_LEDS
  	uint8_t RGB_LED_Counter;
  	uint8_t RGB_LED_Loop;
  #endif
  
    //Z9V5_AUTO_TEST
    uint8_t auto_test_flag = 0x55; //0x55: disable, 0xAA: enabled
  	uint8_t test_loop_rg = 0;
    uint16_t test_timer_rg = 0;
	uint16_t test_counter_rg = 0;
	uint8_t test_rotary_counter_rg = 0;
	uint8_t test_stop_rg = 0;
	bool test_dir_fg:1;
	bool test_fan_fg:1;
	bool test_rotary_fg:1;
	bool Is_X_Endstop:1;
	bool Is_Y_Endstop:1;
	bool Is_Z1_Endstop:1;
	bool Is_Z2_Endstop:1;
	bool Is_F_Endstop:1;
	
  AxisEnum acc_axis, jerk_axis, step_axis;
} HMI_Flag_t;

extern HMI_value_t HMI_ValueStruct;
extern HMI_Flag_t HMI_flag;

// Show ICO
void ICON_Print(bool show);
void ICON_Prepare(bool show);
void ICON_Control(bool show);

#ifdef HAS_ONESTEP_LEVELING
void ICON_Leveling(bool show);
#endif

void ICON_StartInfo(bool show);

void ICON_Setting(bool show);
void ICON_Pause(bool show);
void ICON_Continue(bool show);
void ICON_Stop(bool show);

#if HAS_HOTEND || HAS_HEATED_BED
  // Popup message window
  void DWIN_Popup_Temperature(const char *msg);
#endif

#if HAS_HOTEND
  void Popup_Window_ETempTooLow();
#endif

void Popup_Window_Resume();
void Popup_Window_HomeAll(const bool parking/*=false*/);
void Popup_Window_HomeX(const bool parking/*=false*/);
void Popup_Window_HomeY(const bool parking/*=false*/);
void Popup_Window_HomeZ(const bool parking/*=false*/);

#ifdef HAS_ONESTEP_LEVELING
void Popup_Window_Leveling();
#endif

#ifdef LCD_BED_LEVELING
void Popup_Remove_Glass();
#endif

void Goto_PrintProcess();
void Goto_MainMenu();

// Variable control
void HMI_Move_X();
void HMI_Move_Y();
void HMI_Move_Z();
void HMI_Move_E();

void HMI_Zoffset();

TERN_(HAS_HOTEND,     void HMI_ETemp());
TERN_(HAS_HEATED_BED, void HMI_BedTemp());
TERN_(HAS_FAN,        void HMI_FanSpeed());

void HMI_PrintSpeed();

void HMI_MaxFeedspeedXYZE();
void HMI_MaxAccelerationXYZE();
void HMI_MaxJerkXYZE();
void HMI_StepXYZE();

void update_variable();
FORCE_INLINE void DWIN_Draw_Signed_Float(uint8_t size, uint16_t Color/* = Color_White*/, uint16_t bColor /* = Color_Bg_Black*/, uint8_t iNum, uint8_t fNum, uint16_t x, uint16_t y, long value);

// SD Card
void HMI_SDCardInit();
void HMI_SDCardUpdate();

// Main Process
void Icon_print(bool value);
void Icon_control(bool value);
void Icon_temperature(bool value);
void Icon_leveling(bool value);

// Other
void Draw_Status_Area(const bool with_update); // Status Area
void Draw_Mixer_Status_Area(const bool with_update); // Mixer Status Area
void HMI_StartFrame(const bool with_update);   // Prepare the menu view
void HMI_MainMenu();    // Main process screen
void HMI_SelectFile();  // File page
void HMI_Printing();    // Print page
void HMI_Prepare();     // Prepare page
void HMI_Control();     // Control page
void HMI_Leveling();    // Level the page
void HMI_AxisMove();    // Axis movement menu
void HMI_Temperature(); // Temperature menu
void HMI_Motion();      // Sports menu
void HMI_Info();        // Information menu
void HMI_Tune();        // Adjust the menu

#if HAS_PREHEAT
  void HMI_PLAPreheatSetting(); // PLA warm-up setting
  void HMI_ABSPreheatSetting(); // ABS warm-up setting
#endif

void HMI_MaxSpeed();        // Maximum speed submenu
void HMI_MaxAcceleration(); // Maximum acceleration submenu
void HMI_MaxJerk();         // Maximum jerk speed submenu
void HMI_Step();            // Transmission ratio

void HMI_Init();
void DWIN_Update();
void EachMomentUpdate();
void DWIN_HandleScreen();

void DWIN_CompletedHoming();
void DWIN_CompletedLeveling();

void Refresh_Percent_display();
void Draw_Print_ProgressModel();
void updata_mixer_from_vtool();

void Dwin_Lcd_Test();
void HMI_StartTest();

void Clear_Bottom_Area();
void Popup_Window_FMoveStart();
void Popup_Window_BMoveStart();
void Popup_Window_BMoveStop();


#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #include "../../../feature/pause.h"
void DWIN_Pause_Show_Message(
  const DWINPauseMessage message,
  const DWINPauseMode mode/*=PAUSE_MODE_SAME*/
  //const uint8_t extruder/*=active_extruder*/
);
#endif

void Check_Rotary();

#if HAS_COLOR_LEDS
void RGB_LED_Light(uint8_t color);
#endif

#if ENABLED(OPTION_WIFI_MODULE)
void DWIN_Wifi_Show_M117(const char * const message);
#endif

void DWIN_G29_Show_Messge(const DWIN_G29_MESSAGE message = G29_LEVLE_DEFAULT,const int pt_index = 0,const int all_points = 0,const float fvalue = 0.0);

extern void srand(unsigned int seed);
extern int rand();

