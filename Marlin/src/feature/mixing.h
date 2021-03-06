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

#include "../inc/MarlinConfig.h"

#if ENABLED(MIXING_EXTRUDER)
//#define MIXER_NORMALIZER_DEBUG

#ifndef __AVR__ // 
  // Use 16-bit (or fastest) data for the integer mix factors
  typedef uint_fast16_t mixer_comp_t;
  typedef uint_fast16_t mixer_accu_t;
  #define COLOR_A_MASK 0x8000
  #define COLOR_MASK 0x7FFF
#else
  // Use 8-bit data for the integer mix factors
  // Exactness is sacrificed for speed
  #define MIXER_ACCU_SIGNED
  typedef uint8_t mixer_comp_t;
  typedef int8_t mixer_accu_t;
  #define COLOR_A_MASK 0x80
  #define COLOR_MASK 0x7F
#endif

typedef int8_t mixer_perc_t;

#ifndef MIXING_VIRTUAL_TOOLS
  #define MIXING_VIRTUAL_TOOLS 1
#endif

enum MixTool {
    FIRST_USER_VIRTUAL_TOOL = 0
  , LAST_USER_VIRTUAL_TOOL = MIXING_VIRTUAL_TOOLS - 1
  , NR_USER_VIRTUAL_TOOLS
  , MIXER_DIRECT_SET_TOOL = NR_USER_VIRTUAL_TOOLS
  #if HAS_MIXER_SYNC_CHANNEL
    , MIXER_AUTORETRACT_TOOL
  #endif
  , NR_MIXING_VIRTUAL_TOOLS
};

#define MAX_VTOOLS TERN(HAS_MIXER_SYNC_CHANNEL, 254, 255)
static_assert(NR_MIXING_VIRTUAL_TOOLS <= MAX_VTOOLS, "MIXING_VIRTUAL_TOOLS must be <= " STRINGIFY(MAX_VTOOLS) "!");

#define MIXER_BLOCK_FIELD       mixer_comp_t b_color[MIXING_STEPPERS]
#define MIXER_POPULATE_BLOCK()  mixer.populate_block(block->b_color)
#define MIXER_STEPPER_SETUP()   mixer.stepper_setup(current_block->b_color)
#define MIXER_STEPPER_LOOP(VAR) for (uint_fast8_t VAR = 0; VAR < MIXING_STEPPERS; VAR++)

#if ENABLED(GRADIENT_MIX)

  typedef struct {
    bool enabled;                         // This gradient is enabled
    mixer_comp_t color[MIXING_STEPPERS];  // The current gradient color
    float start_z, end_z;                 // Region for gradient
    int8_t start_vtool, end_vtool;        // Start and end virtual tools
    mixer_perc_t start_mix[MIXING_STEPPERS],   // Start and end mixes from those tools
                 end_mix[MIXING_STEPPERS];
    TERN_(GRADIENT_VTOOL, int8_t vtool_index); // Use this virtual tool number as index
  } gradient_t;

#endif

#if ENABLED(RANDOM_MIX)
typedef struct {
  bool enabled;    
  float start_z, end_z;
  float height;							//Minimum height of changing mixing rate  
  uint8_t extruders;
}randommix_t;
#endif

#if ENABLED(HOTEND_CLEAN_TOWER)
typedef struct {
  uint8_t pattern;
  uint8_t colors;
  float factor;
  uint16_t x, y;
  uint16_t length;
  uint8_t flow_length;
  float retraction_length;
  int retraction_speed;
}cleantower_t;
#endif
/**
 * @brief Mixer class
 * @details Contains data and behaviors for a Mixing Extruder
 */
class Mixer {
  public:
  static mixer_perc_t mix[MIXING_STEPPERS];  // Scratch array for the Mix in proportion to 100
  static float collector[MIXING_STEPPERS];    // M163 components, also editable from LCD
  static int8_t selected_vtool;
  static mixer_comp_t color[NR_MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];
  
  static void init(); // Populate colors at boot time
  static void reset_vtools();
  static void refresh_collector(const float proportion=1.0, const uint8_t t=selected_vtool, float (&c)[MIXING_STEPPERS]=collector);

  // Used up to Planner level
  FORCE_INLINE static void set_collector(const uint8_t c, const float f) { collector[c] = _MAX(f, 0.0f); }
  FORCE_INLINE static void set_Percentmix(const uint8_t c, const uint8_t d) { mix[c] = d; }
  FORCE_INLINE static void reset_collector(const uint8_t t){
  	MIXER_STEPPER_LOOP(i) set_collector(i, i == t ? 1.0 : 0.0);}

  static void copy_mix_to_collector();
  static void copy_collector_to_mix();
  static void init_collector(const uint8_t index);
  //
  static void normalize(const uint8_t tool_index);
  FORCE_INLINE static void normalize() { normalize(selected_vtool); }

  FORCE_INLINE static uint8_t get_current_vtool() { return selected_vtool; }

  FORCE_INLINE static void T(const uint_fast8_t c) {
    selected_vtool = c;
    TERN_(GRADIENT_VTOOL, refresh_gradient());
    update_mix_from_vtool();
  }

  // Used when dealing with blocks
  FORCE_INLINE static void populate_block(mixer_comp_t b_color[MIXING_STEPPERS]) {
    #if ENABLED(GRADIENT_MIX)
      if (gradient.enabled) {
        MIXER_STEPPER_LOOP(i) b_color[i] = gradient.color[i];
        return;
      }
    #endif
    MIXER_STEPPER_LOOP(i) b_color[i] = color[selected_vtool][i];
  }

  FORCE_INLINE static void stepper_setup(mixer_comp_t b_color[MIXING_STEPPERS]) {
    MIXER_STEPPER_LOOP(i) s_color[i] = b_color[i];
  }

    static inline void copy_mix_to_color(mixer_comp_t (&tcolor)[MIXING_STEPPERS]) {
	  uint16_t sum_mix = 0;
	  MIXER_STEPPER_LOOP(i) sum_mix+= mix[i];
	  const float scaleMix = 100/sum_mix;
	  MIXER_STEPPER_LOOP(i) mix[i] *= scaleMix;
	  sum_mix = 0;
	  for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) sum_mix += mix[i];
	  mix[MIXING_STEPPERS-1] = 100 - sum_mix;
      // Scale each component to the largest one in terms of COLOR_A_MASK
      // So the largest component will be COLOR_A_MASK and the other will be in proportion to it
      const float scale = (COLOR_A_MASK) * RECIPROCAL(_MAX(
        LIST_N(MIXING_STEPPERS, mix[0], mix[1], mix[2], mix[3], mix[4], mix[5])
      ));

      // Scale all values so their maximum is COLOR_A_MASK
      MIXER_STEPPER_LOOP(i) tcolor[i] = mix[i] * scale;

      #ifdef MIXER_NORMALIZER_DEBUG
	    SERIAL_ECHOLNPGM("copy_mix_to_color >> ");
        SERIAL_ECHOPGM("Mix [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, int(mix[0]), int(mix[1]), int(mix[2]), int(mix[3]), int(mix[4]), int(mix[5]));
        SERIAL_ECHOPGM(" ] to Color [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, int(tcolor[0]), int(tcolor[1]), int(tcolor[2]), int(tcolor[3]), int(tcolor[4]), int(tcolor[5]));
        SERIAL_ECHOLNPGM(" ]");
      #endif
    }

    static inline void update_mix_from_vtool(const uint8_t j=selected_vtool) {
      float ctot = 0;
	  int16_t sum_mix = 0;
      MIXER_STEPPER_LOOP(i) ctot += color[j][i];
      //MIXER_STEPPER_LOOP(i) mix[i] = 100.0f * color[j][i] / ctot;
      MIXER_STEPPER_LOOP(i) mix[i] = (mixer_perc_t)(100.0f * color[j][i] / ctot);
	  
	  for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) sum_mix += mix[i];
	  	mix[MIXING_STEPPERS-1] = 100 - sum_mix;

      #ifdef MIXER_NORMALIZER_DEBUG
	  	SERIAL_ECHOLNPGM("update_mix_from_vtool");
	  	SERIAL_EOL();		
        SERIAL_ECHOPAIR("V-tool ", int(j), " [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, int(color[j][0]), int(color[j][1]), int(color[j][2]), int(color[j][3]), int(color[j][4]), int(color[j][5]));
        SERIAL_ECHOPGM(" ] to Mix [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, int(mix[0]), int(mix[1]), int(mix[2]), int(mix[3]), int(mix[4]), int(mix[5]));
        SERIAL_ECHOLNPGM(" ]");
		SERIAL_EOL();
      #endif
    }
  
    // Update the virtual tool from an edited mix
    static inline void update_vtool_from_mix() {
      copy_mix_to_color(color[selected_vtool]);
      #if ENABLED(GRADIENT_MIX)
        refresh_gradient();
      #endif
      //MIXER_STEPPER_LOOP(i) collector[i] = mix[i];
      //normalize();
    }

  #if ENABLED(GRADIENT_MIX)

    static gradient_t gradient;
    static float gradient_prev_z;

    // Update the current mix from the gradient for a given Z
    static void update_gradient_for_z(const float z);
    static void update_gradient_for_planner_z();
    static inline void gradient_control(const float z) {
	#if ENABLED(RANDOM_MIX)
      if (!random_mix.enabled && gradient.enabled) {
	#else
	  if (gradient.enabled) {
	#endif
        if (z >= gradient.end_z){
          T(gradient.end_vtool);
		#if DISABLED(GRADIENT_VTOOL)
		  gradient.start_vtool = gradient.end_vtool = 0;
		  gradient.end_z = gradient.start_z = 0;
		  gradient.enabled = false;
		#endif
        }
        else{
          update_gradient_for_z(z);
        }
      }
    }

    static inline void update_mix_from_gradient() {
      float ctot = 0;
      MIXER_STEPPER_LOOP(i) ctot += gradient.color[i];
      MIXER_STEPPER_LOOP(i) mix[i] = (mixer_perc_t)CEIL(100.0f * gradient.color[i] / ctot);

      #ifdef MIXER_NORMALIZER_DEBUG
	    SERIAL_ECHOLNPGM("update_mix_from_gradient");
	  	SERIAL_EOL();
        SERIAL_ECHOPGM("Gradient [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, int(gradient.color[0]), int(gradient.color[1]), int(gradient.color[2]), int(gradient.color[3]), int(gradient.color[4]), int(gradient.color[5]));
        SERIAL_ECHOPGM(" ] to Mix [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, int(mix[0]), int(mix[1]), int(mix[2]), int(mix[3]), int(mix[4]), int(mix[5]));
        SERIAL_ECHOLNPGM(" ]");
		SERIAL_EOL();
      #endif
    }

    // Refresh the gradient after a change
    static void refresh_gradient() {
      #if ENABLED(GRADIENT_VTOOL)
        const bool is_grd = (gradient.vtool_index == -1 || selected_vtool == (uint8_t)gradient.vtool_index);
      #else
        constexpr bool is_grd = true;
      #endif
      gradient.enabled = is_grd && gradient.start_vtool != gradient.end_vtool && gradient.start_z < gradient.end_z;
      if (gradient.enabled) {
	  	#if ENABLED(RANDOM_MIX)
		random_mix.start_z = random_mix.end_z = 0;
		random_mix.enabled = false;
		#endif
        mixer_perc_t mix_bak[MIXING_STEPPERS];
        COPY(mix_bak, mix);
        update_mix_from_vtool(gradient.start_vtool);
        COPY(gradient.start_mix, mix);
        update_mix_from_vtool(gradient.end_vtool);
        COPY(gradient.end_mix, mix);
        update_gradient_for_planner_z();
        COPY(mix, mix_bak);
        gradient_prev_z = -1;
      }
    }

  #endif // GRADIENT_MIX
  
  #if ENABLED(RANDOM_MIX)
   static randommix_t random_mix;
   static float prev_z_randommix;
   static void update_randommix_for_z(const float z);
   static void update_randommix_for_planner_z();
   
   static inline void randommix_control(const float z) {
   	#if ENABLED(GRADIENT_MIX)
      if (random_mix.enabled && !gradient.enabled) {
	#else
	  if (random_mix.enabled) {
	#endif
        if (z <= random_mix.end_z){
          update_randommix_for_z(z);
        }
		else{
		  random_mix.enabled = false;
		  random_mix.end_z = random_mix.start_z = 0;
		  random_mix.height = 0.2;
		  random_mix.extruders = MIXING_STEPPERS;
	    }
      }
    }

    // Refresh the random after a change
    static void refresh_random_mix() {
      random_mix.enabled = random_mix.start_z < random_mix.end_z;
      if (random_mix.enabled) {
	  	selected_vtool = 0;
	  	#if ENABLED(GRADIENT_MIX)		
		gradient.start_vtool = gradient.end_vtool = 0;
		gradient.start_z = gradient.end_z = 0;
		gradient.enabled = false;
		#endif
		prev_z_randommix = -1;
      }
    }
  #endif

  #if ENABLED(HOTEND_CLEAN_TOWER)
  static cleantower_t cleantower;
  static void build_clean_tower();
  #endif

  // Used in Stepper
  FORCE_INLINE static uint8_t get_stepper() { return runner; }
  FORCE_INLINE static uint8_t get_next_stepper() {
    for (;;) {
      if (--runner < 0) runner = MIXING_STEPPERS - 1;
      accu[runner] += s_color[runner];
      if (
        #ifdef MIXER_ACCU_SIGNED
          accu[runner] < 0
        #else
          accu[runner] & COLOR_A_MASK
        #endif
      ) {
        accu[runner] &= COLOR_MASK;
        return runner;
      }
    }
  }

  
  
  private:
  // Used up to Planner level
  //static uint_fast8_t selected_vtool;
  //static mixer_comp_t color[NR_MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];

  // Used in Stepper
  static int_fast8_t  runner;
  static mixer_comp_t s_color[MIXING_STEPPERS];
  static mixer_accu_t accu[MIXING_STEPPERS];

};

extern Mixer mixer;

#endif // MIXING_EXTRUDER
