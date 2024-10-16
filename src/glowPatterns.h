/*
 * glowPatterns.h
 *
 *  Created on: May 7, 2014
 *      Author: michael
 */

#ifndef GLOWPATTERNS_H_
#define GLOWPATTERNS_H_
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <main.h>

//--------------------------------------------------
// Defines
//--------------------------------------------------
extern int16_t g_maximumCoffeeLevel; // In energy points / 8 s
#define MAXIMUM_VOLTAGE                                                        \
  (HBATT_THRESHOLD - 50)       // Will output MAXIMUM_COFFEE_LEVEL
#define MINIMUM_VOLTAGE (3500) // Will output coffeeLevel = 0
#define MAX_ELEMENTS 35
#define MAX_ADR_curPWMvalues &curPWMvalues[NLEDS - 1]

//--------------------------------------------------
// Custom data-types
//--------------------------------------------------
typedef enum {
  BUG_INACTIVE_PENALTY,
  BUG_INACTIVE_RESCHEDULE,
  BUG_ACTIVE_SAMPLES
} bugState_t;
typedef enum {
  BLACK = 0,
  GLIMMER,
  STARS,
  RAMP_UP,
  ROTATE,
  RAND_ROTATE,
  COLOR_FADE
} fullScreenFlashMode_t;
typedef struct {
  bugState_t b_state;
  const uint16_t
      *samplePointer; // Pointing to next valid sample if remainingSamples >0
  uint16_t remainingSamples; // How long is the sequence (counted down)
  uint8_t sampleDelay; // additional delay between samples (in 1/30 s, 0=30fps,
                       // 1=15fps, etc)
  int16_t availableEnergy; // Energy of the firefly. The higher, the brighter a
                           // pattern it can play
} oneBug_t;

//--------------------------------------------------
// Global variables
//--------------------------------------------------
extern uint16_t ticksSinceBlinking; // in 8 s intervals
extern fullScreenFlashMode_t currentFSFlashMode;
extern uint16_t curPWMvalues[MAX_ELEMENTS]; // Cache only
extern oneBug_t bugs[NLEDS]; // Every bug controls one LED channel

//--------------------------------------------------
// Functions
//--------------------------------------------------
void newFrameGlueh();                     // Call with 30 fps
void newFrameFullscreen(int8_t newState); // Call with 30 fps
void houseKeepingWhileGlowing();          // Call every 8 s!

// read word from FLASH with postincrement addresspointer
#define pgm_read_word_inc(addr)                                                \
  (__extension__({                                                             \
    uint16_t __result;                                                         \
    __asm__("lpm %A0, Z+"                                                      \
            "\n\t"                                                             \
            "lpm %B0, Z+"                                                      \
            "\n\t"                                                             \
            : "=r"(__result), "=z"(addr)                                       \
            : "1"(addr));                                                      \
    __result;                                                                  \
  }))

#endif /* GLOWPATTERNS_H_ */
