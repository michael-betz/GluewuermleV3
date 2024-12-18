/*
 * pwmGlueh.c
 *
 *  Created on: May 7, 2014
 *      Author: michael
 */

// Note on frequencies: 122 Hz PWM,  122 / 3 = 40 fps framefrate

#include "glowPatterns.h"
#include "lfsr32.h"
#include "main.h"
#include "myWaves.h"
#include "pulseCode.h"
#include "rprintf.h"
#include "simplex.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <inttypes.h>
#include <util/delay.h>

int16_t g_maximumCoffeeLevel = 300;
// firefly activity scaled with voltage and time from 0 to g_maximumCoffeeLevel
int16_t g_coffeeLevel = 300;

oneBug_t bugs[NLEDS]; // Every bug controls one LED channel
#define LAST_BUG_ADR (&bugs[NLEDS - 1])
uint16_t curPWMvalues[MAX_ELEMENTS];  // Cache only
uint16_t ticksSinceBlinking = 0x0000; // in 8 s intervals
fullScreenFlashMode_t currentFSFlashMode = RAMP_UP;


void houseKeepingWhileGlowing() {
	int32_t temp_32;

	oneBug_t *bug;
	if (currentState == STATE_GLOW_GLUEH || currentState == STATE_GLOW_SIMPLEX) {
		//  -----------------------------------------------------------
		//   Update the firefly activity by setting fireflySleepieness
		//  -----------------------------------------------------------
		//  fireflyCoffeeLevel = 0 (not active at all) ... 300 (very active)
		// 0 ... (MAXIMUM_VOLTAGE-MINIMUM_VOLTAGE) * MAXIMUM_COFFEE_LEVEL
		temp_32 = ((int32_t)(vCap - MINIMUM_VOLTAGE)) * g_maximumCoffeeLevel;

		// 0 .. MAXIMUM_COFFEE_LEVEL
		temp_32 /= (MAXIMUM_VOLTAGE - MINIMUM_VOLTAGE);

		// Get more sleepy with time. 1 hour = 450 ticks = 28 penalty points.
		temp_32 -= ticksSinceBlinking / 16;

		if (temp_32 > g_maximumCoffeeLevel)
			temp_32 = g_maximumCoffeeLevel;
		if (temp_32 < 0)
			temp_32 = 0;
		g_coffeeLevel = temp_32;
	}
	if (currentState == STATE_GLOW_GLUEH) {
		//  -----------------------------------------------------------
		//   Fed the bugs with "fireflyCoffeeLevel" energy every 8 seconds
		//   The value is spread out amongst all 12 channels
		//  -----------------------------------------------------------
		int16_t coffeeLevel = g_coffeeLevel;
		bug = &bugs[lfsr(4) % NLEDS];
		if (coffeeLevel <= 1) {
			bug->availableEnergy += coffeeLevel;
		} else {
			while (coffeeLevel > 0) {
				coffeeLevel /= 2;
				bug->availableEnergy += coffeeLevel;
				bug++;
				if (bug > LAST_BUG_ADR) {
					bug = bugs;
				}
			}
		}
#if DEBUG_LEVEL > 0
		int32_t sum = 0;
		rprintf("eng[");
		for (uint8_t temp = 0; temp < NLEDS; temp++) {
			rprintf(" %5d", bugs[temp].availableEnergy);
			sum += bugs[temp].availableEnergy;
		}
		rprintf("] S=%5ld\n", sum);
#endif
	}

	ticksSinceBlinking++;
	//  -----------------------------------------------------------
	//   Check if there is enough sun to switch to charging mode
	//  -----------------------------------------------------------
	if (vSolar > CHARGING_THRESHOLD) {
		rprintf("currentState = STATE_CHARGING  (a lot of sun)\n");
		pwmTimerOff();
		WDT_SLOW_MODE();
		currentState = STATE_CHARGING;
		ticksSinceBlinking = 0;
	}
}

oneBug_t *startSeqGlueh(uint8_t bugId) {
	oneBug_t *bug = &bugs[bugId];
	oneWaveform_t tmpWave;
	int16_t enCost;
	int8_t propBonus = bug->availableEnergy / 512;

	if (propBonus < 0)
		propBonus = 0;

	if (propBonus > 4)
		propBonus = 4;

	// nRand = 0-31,   if 3 is the prop bonus!
	uint8_t patternID = lfsr(8 - propBonus);

	// Start a new pattern (with number nRand) on nRand = 0-5
	if (patternID < N_WAVEFORMS) {
		//  Our first job is to find a pattern which does not use too much
		//  energy
		memcpy_P(&tmpWave, &waveformTable[patternID], sizeof(oneWaveform_t));
		bug->samplePointer = tmpWave.samplePointer;
		bug->remainingSamples = tmpWave.length;
		bug->b_state = BUG_ACTIVE_SAMPLES;
		enCost = tmpWave.energyCost;
		rprintf("bug[%2d]: ACTIVE, en = %d, cost = %d,  pBonus = %d\n", bugId,
				bug->availableEnergy, enCost, propBonus);

		// When a wave gets started, energy gets shifted around!
		// if bug[0] was blinking with en:  bug[0]-=en, bug[1]+=en/2,
		// bug[3]-=en/4, bug[4]-=en/8, etc. overall energy billiance is
		// reduced by en
		bug->availableEnergy -= enCost;
		enCost /= 2;
		bug++;
		if (bug > LAST_BUG_ADR)
			bug = bugs;
		bug->availableEnergy += enCost;
		while (enCost > 0) {
			bug++;
			if (bug > LAST_BUG_ADR)
				bug = bugs;
			enCost /= 2;
			bug->availableEnergy -= enCost;
		}
	} else { // Let the bug sleep for nRand samples otherwise
		bug->b_state = BUG_INACTIVE_PENALTY;
		// up to 1 min waiting time until next try
		bug->remainingSamples = lfsr(11);
		// rprintf( "bug[%2d]: PENALTY for %d samples, pBonus = %d\n", bugId,
		// bug->remainingSamples, propBonus );
	}
	return bug;
}

// This triggered with about 30 Hz but called by main
void newFrameGlueh() {
	uint8_t nled,
		ledsAreOn = 0; // zero if no LEDs are on during this frame upgrade
	static uint8_t frameDelayCounter = 0;
	//  Iterate over all LED channels (= all bugs) and handle them accordingly
	//  -----------------------------------------------------------------------
	oneBug_t *bug = bugs;
	for (nled = 0; nled < NLEDS; nled++) {
		switch (bug->b_state) {
		case BUG_ACTIVE_SAMPLES:
			if (bug->remainingSamples > 0) {
				ledsAreOn++;
				if (frameDelayCounter++ >= bug->sampleDelay) {
					frameDelayCounter = 0;
					setPwmValue(nled, pgm_read_word(bug->samplePointer));
					bug->samplePointer++;
					bug->remainingSamples--;
				}
			} else {
				bug->b_state = BUG_INACTIVE_RESCHEDULE;
				setPwmValue(nled, 0);
			}
			break;
		case BUG_INACTIVE_PENALTY:
			// A bug on standby still has some remaining samples
			if (bug->remainingSamples > 0) {
				bug->remainingSamples--;
			} else {
				bug->b_state = BUG_INACTIVE_RESCHEDULE;
			}
			break;
		case BUG_INACTIVE_RESCHEDULE:
			if (bug->availableEnergy > 0) {
				// Starts a new sequence (with a rather small probability)
				startSeqGlueh(nled);
			}
		}
		bug++;
	}
	if (ledsAreOn) {
		// Switch off pwm timer in this cycle to save energy
		pwmTimerOn();
	} else {
		// At least one LED is active, activate pwmTimer if necessary
		pwmTimerOff();
	}
}

// This triggered with about 30 Hz but called by main
// If newState < 0: just put a new frame
// If ne<wState >= 0: (Re-)initialize in noise
void newFramePerlin(int8_t newState) {
	static uint32_t seeds[NLEDS];
	static uint16_t t = 0;

	if (newState >= 0) {
		for (uint8_t led = 0; led < NLEDS; led++)
			seeds[led] = (uint32_t)(lfsr(16)) << 16 | lfsr(16);
		pwmTimerOn();
		return;
	}

	// g_coffeeLevel,	burn_threshold
	//   0,				0x10000  // Off
	// 100,				0x0e0c0  // Calm (1 - 2 at the same time)
	// 200,				0x0c180
	// 300,				0x0a240
	// 400,				0x08300  // Very active  (6 - 7 at the same time)
	// 500,				0x063c0
	int32_t burn_threshold = 0x10000 - 0x50 * g_coffeeLevel;

	rprintf("simplex [");
	for (uint8_t led = 0; led < NLEDS; led++) {
		// int tmp = smooth1d(t / 1024.0, 3, seeds[led]) * MAX_PWM_VALUE;
		int32_t tmp = snoise_1D(((uint32_t)(t) << 3) + seeds[led]);
		tmp -= burn_threshold;
		if (tmp < 0)
			tmp = 0;
		tmp = (tmp * tmp) >> 16;
		if (tmp > MAX_PWM_VALUE)
			tmp = MAX_PWM_VALUE;
		rprintf("%6ld ", tmp);
		setPwmValue(led, tmp);
	}
	rprintf("]\n");

	t++;
}



// This triggered with about 30 Hz but called by main
// If newState < 0: just put a new frame
// If newState > 0: change flash mode to newState
void newFrameFullscreen(int8_t newState) {
	// Channel indexes (4 bit each) for Blue
	uint16_t colorMatching[] = {
		0xFFF0,  // blue
		0xFF41,  // orange
		0xFF82,  // red
		0xFF53,  // green
		0xFF96,  // white
		0xFFA7   // UV
	};
	#define N_COLORS (sizeof(colorMatching) / sizeof(colorMatching[0]))
	uint8_t temp, nled;
	uint16_t *pwmValue_p, temp2;
	static uint16_t framesInThisMode = 0;
	static uint8_t tempFrameCounter = 0;
	if (newState < 0) { // Render a new frame
		switch (currentFSFlashMode) {
		default:
		case BLACK: // Fade out the lights and then keep them dark
			temp = 1;
			pwmValue_p = curPWMvalues;
			for (nled = 0; nled < MAX_ELEMENTS; nled++) {
				if (*pwmValue_p > 100) {
					*pwmValue_p -= 10;
					temp = 0;
				} else if (*pwmValue_p > 0) {
					*pwmValue_p -= 1;
					temp = 0;
				}
				pwmValue_p++;
			}
			if (temp) {
				// Now all lights are completely faded out
				// We will only wake up on the next mutate() call
				pwmTimerOff();
			}
			break;
		case GLIMMER: // All leds kind of on, on low brightness
			// Every frame a random LED is made a bit brighter
			curPWMvalues[lfsr(8) % NLEDS] += 1;
			if (tempFrameCounter++ > 3) {
				// Every 3 frames a LED is turned OFF
				tempFrameCounter = 0;
				curPWMvalues[lfsr(8) % NLEDS] = 0;
			}
			if (framesInThisMode >= (60 * 30)) {
				currentFSFlashMode = BLACK;
			}
			break;
		case STARS:
			for (pwmValue_p = curPWMvalues; pwmValue_p <= MAX_ADR_curPWMvalues;
				 pwmValue_p++) {
				if (lfsr(8) > 250) {
					// Make every fifth LED super bright
					*pwmValue_p = MAX_PWM_VALUE;
				} else {
					*pwmValue_p = 0;
				}
			}
			if (framesInThisMode >= (2 * 30)) {
				currentFSFlashMode = BLACK;
			}
			break;
		case RAMP_UP:
			if (tempFrameCounter <= NLEDS - 1) {
				pwmValue_p = &curPWMvalues[tempFrameCounter];
				if (*pwmValue_p >= 40) {
					tempFrameCounter++;
				} else {
					*pwmValue_p += 1;
				}
			} else if (tempFrameCounter <= 2 * NLEDS - 1) {
				pwmValue_p = &curPWMvalues[tempFrameCounter - NLEDS];
				if (*pwmValue_p <= 0) {
					tempFrameCounter++;
				} else {
					*pwmValue_p -= 1;
				}
			} else {
				tempFrameCounter = 0;
			}
			if (framesInThisMode >= (60 * 30)) {
				currentFSFlashMode = BLACK;
			}
			break;
		case RAND_ROTATE:
			if (framesInThisMode == 0) {
				pwmValue_p = curPWMvalues;
				*pwmValue_p++ = lfsr(8);			// 1
				for (nled = 0; nled < 11; nled++) { // 11
					*pwmValue_p++ = lfsr(6);
				}
				*pwmValue_p++ = lfsr(5);
				*pwmValue_p++ = lfsr(4);
				*pwmValue_p++ = lfsr(3);			// 3
				for (nled = 0; nled < 17; nled++) { // 17, Sum = 32 values
					*pwmValue_p++ = lfsr(2);
				}
				framesInThisMode++;
			}
		case ROTATE:
			if (framesInThisMode == 0) {
				pwmValue_p = curPWMvalues;
				*pwmValue_p++ = 2;
				*pwmValue_p++ = 8;
				*pwmValue_p++ = 16;
				*pwmValue_p++ = 64;
				*pwmValue_p++ = 32;
				*pwmValue_p++ = 16;
				*pwmValue_p++ = 8;
				*pwmValue_p++ = 6;
				*pwmValue_p++ = 4;
				*pwmValue_p++ = 3;
				*pwmValue_p++ = 2;
				*pwmValue_p++ = 1;
			}
			if (tempFrameCounter++ >= 2) {
				tempFrameCounter = 0;
				temp2 = *curPWMvalues; // Save lowest element
				for (temp = 0; temp < MAX_ELEMENTS - 1; temp++) {
					// Move all elements one step down
					curPWMvalues[temp] = curPWMvalues[temp + 1];
				}

				// Put first element on top
				curPWMvalues[MAX_ELEMENTS - 1] = temp2;
			}
			if (framesInThisMode >= (60 * 30)) {
				currentFSFlashMode = BLACK;
			}
			break;
		case COLOR_FADE:
			// Fade out on all odd numbers
			if (tempFrameCounter & 0x01) {
				temp = 1;
				pwmValue_p = curPWMvalues;
				for (nled = 0; nled < MAX_ELEMENTS; nled++) {
					if (*pwmValue_p >= 8) {
						*pwmValue_p -= 8;
						temp = 0;
					} else if (*pwmValue_p > 0) {
						*pwmValue_p -= 1;
						temp = 0;
					}
					pwmValue_p++;
				}

				if (temp) {
					// Now all lights are completely faded out
					tempFrameCounter++; // Switch to an even number
				}
			} else {
				// Fade up one color
				nled = tempFrameCounter / 2; // Index from 0 to 5
				if (nled >= N_COLORS) {
					nled = 0;
					tempFrameCounter = 0;
				}
				temp2 = colorMatching[nled];
				for (nled = 0; nled < 4; nled++) {
					temp = temp2 & 0x000F;
					if (temp < NLEDS) {
						curPWMvalues[temp] += lfsr(5);
						if (curPWMvalues[temp] > 500) {
							tempFrameCounter++;
							break;
						}
					}
					temp2 >>= 4;
				}
			}
			if (framesInThisMode >= (60 * 30)) {
				currentFSFlashMode = BLACK;
			}
			break;
		}
		framesInThisMode++;
		// What follows is expensive !!!
		temp = 0;
		for (pwmValue_p = curPWMvalues; pwmValue_p <= MAX_ADR_curPWMvalues;
			 pwmValue_p++) {
			setPwmValue(temp++, *pwmValue_p);
		}
	} else {
		// Switch glowing mode!
		if (!IBI(flags, FLAG_PWM_IS_ON)) {
			// Only change mode when all LEDs are OFF
			tempFrameCounter = 0;
			framesInThisMode = 0;
			if (newState > COLOR_FADE) {
				newState = BLACK;
			}
			currentFSFlashMode = newState;
			pwmTimerOn();
			rprintf("currentFSFlashMode = %d\n", currentFSFlashMode);
		}
	}
}
