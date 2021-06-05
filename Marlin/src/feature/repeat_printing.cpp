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
 * repeat_printing.cpp - repeat printing control
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(OPTION_REPEAT_PRINTING)

#include "repeat_printing.h"
#include "../module/temperature.h"
#include "../module/stepper/indirection.h"
#include "../MarlinCore.h"

#if HAS_DWIN_LCD
  #include "../lcd/dwin/e3v2/dwin.h"
#endif


RePrint ReprintManager;

void RePrint::Init() {
  OUT_WRITE(FPRWARD_PIN,LOW);
  delayMicroseconds(10);
  OUT_WRITE(BACK_PIN,LOW);
  delayMicroseconds(10);
  SET_INPUT_PULLUP(REPRINT_STOP_PIN);
  
  TERN_(HAS_DWIN_LCD, HMI_flag.Is_Reprint_Reset = 1);
  Back_Move_Start();
	reprt_timer = millis();
}

bool RePrint::Forward_Move_Process(int Fmove_Timer) {
  if(reprt_timer - millis() > Fmove_Timer){
  	reprt_timer = millis();
		 return true;
  }
  return false;
}

bool RePrint::Check_ENDSTOP() {
  if(!READ(REPRINT_STOP_PIN))
	{
		delay(2);
		if(!READ(REPRINT_STOP_PIN))	
			return true;
	}
	return false;
}


bool RePrint::Reprint_BTemp_Check() {
  if((reprt_timer - millis())/100 > CHECK_TEMP_PER_TIMER){
  	reprt_timer = millis();
		if(thermalManager.temp_bed.celsius <= 25) 
			return true;
		else if((ABS(raw_bedtemp_old - thermalManager.temp_bed.raw) < TEMP_DIFFER) && (thermalManager.temp_bed.celsius < 30)) {
			if(tempbed_var++ > 10){
				tempbed_var = 0;
				raw_bedtemp_old = thermalManager.temp_bed.raw;
				return true;
			}
		}
		else 
		raw_bedtemp_old = thermalManager.temp_bed.raw;
  }
  return false;
}

void RePrint::Forward_Move_Start() {
  OUT_WRITE(FPRWARD_PIN,LOW);
  delayMicroseconds(1);
  OUT_WRITE(BACK_PIN,HIGH);
}

void RePrint::Back_Move_Start() {
  OUT_WRITE(FPRWARD_PIN,HIGH);
  delayMicroseconds(1);
  OUT_WRITE(BACK_PIN,LOW);
}

void RePrint::Back_Move_Stop() {
  OUT_WRITE(FPRWARD_PIN,LOW);
  delayMicroseconds(1);
  OUT_WRITE(BACK_PIN,LOW);
}



bool RePrint::Repeat_Print_Process(RePrint_state_t state) {
  uint8_t RPrint_Fg = 0;
  switch(reprt_state){
  	case REPRINT_INIT:
			raw_bedtemp_old = thermalManager.temp_bed.raw;
	    reprt_timer = millis();
			tempbed_var = 0;
			reprt_state = REPRINT_WAIT;
		break;
		
	case REPRINT_WAIT:
		if(Reprint_BTemp_Check()) 
			reprt_state = FORWARD_START;
		break;
		
	case FORWARD_START:
		Forward_Move_Start();
		reprt_state = FORWARD_MOVE;
    reprt_timer = millis(); 
		TERN_(HAS_DWIN_LCD,Popup_Window_FMoveStart());
		break;

	case FORWARD_MOVE:
		if(Forward_Move_Process(Forward_lenght*60/8/DCMOTOR_SPEED_RMP)) 
			reprt_state = BACK_START;
		else {
			#if HAS_DWIN_LCD
			const time_lapse = (reprt_timer - millis())/1000;
			DWIN_Draw_Rectangle(1, Color_Bg_Window, 14, 120, 258, 160);
			if(time_lapse >= 100) 
				DWIN_Draw_IntValue(true, true, 0, font20x40, Color_White, Color_Bg_Window, 3, 106, 120, time_lapse);
			else if(time_lapse >= 10) 
				DWIN_Draw_IntValue(true, true, 0, font20x40, Color_White, Color_Bg_Window, 2, 116, 120, time_lapse);
			else if(time_lapse < 10) 
				DWIN_Draw_IntValue(true, true, 0, font20x40, Color_White, Color_Bg_Window, 1, 126, 120, time_lapse);
			#endif
		}
		break;

	case BACK_START:
		Back_Move_Start();
	  TERN_(HAS_DWIN_LCD,HMI_flag.Is_Reprint_Reset = 1);
		TERN_(HAS_DWIN_LCD,Popup_Window_BMoveStart());
		Reprint_over = 0;
		reprt_state = BACK_MOVE;
		break;

	case BACK_MOVE:
		if(Reprint_over) {
			Reprint_over = 0;
			TERN_(HAS_DWIN_LCD,Popup_Window_BMoveStop());
			reprt_state = BACK_STOP;
		}
		break;

	case BACK_STOP:
			RPrint_Fg = 1;
		break;
		
	default: RPrint_Fg = 0;
		break;
  } 
  return RPrint_Fg;
}

#endif // REPEAT_PRINTING_CONTROL
