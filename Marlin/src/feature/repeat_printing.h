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
 * repeat_printing.h - repeat printing control
 */

#include "../core/millis_t.h"

class RePrint {
  public:
		bool enabled = false;
		bool Reprint_over = false;
		bool Is_Reprint_Reset = false;
		bool Is_Reprint_Print = false;
		bool Reprint_Reset_Enabled = false;
		bool Reprint_Wait_Enabled = false;
		 
		uint16_t Reprint_times = REPEAT_PRINTING_TIMES;
		uint16_t Forward_lenght = FORWARD_PRINTING_LENGHT;
		uint16_t tempbed_counter = 0;
		millis_t reprt_timer;
		enum RePrint_state_t: uint8_t{
			REPRINT_INIT = 0,
			REPRINT_WAIT,
			FORWARD_START,
			FORWARD_MOVE,
			BACK_START,
			BACK_MOVE,
			BACK_STOP
		};
		RePrint_state_t reprt_state = REPRINT_INIT;
		
		static void Init();
		static bool Repeat_Print_Process();
		static bool Back_Move_Process();
		static void Back_Move_Stop();
		static void Back_Move_Start();
		static void Forward_Move_Start();
		static bool Forward_Move_Process(int Fmove_Timer);
		bool Check_ENDSTOP(); 
		
  private:
  
		int16_t raw_bedtemp_old;
		uint8_t tempbed_var;
		
		static bool Reprint_BTemp_Check();
	
};

extern RePrint ReprintManager;

