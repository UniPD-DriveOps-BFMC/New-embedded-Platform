/*
 * vl6180.c
 *
 *  Created on: May 21, 2025
 *      Author: Stefano
 */

#include "speaker.h"
#include "main.h"
#include <stdio.h>


void playTone(uint32_t frequency_hz, uint32_t duration_ms) {
	if(frequency_hz == NOTE_REST) {
		htim4.Instance->CCR1 = 0;  // Silence
		HAL_Delay(duration_ms);
		return;
	}

	uint32_t period = (1000000 / frequency_hz) - 1;  // 1MHz timer clock

	htim4.Instance->ARR = period;
	htim4.Instance->CCR1 = period / 2;  // 50% duty cycle
	HAL_Delay(duration_ms);
	htim4.Instance->CCR1 = 0;  // Silence
}

void playMelody(const uint16_t melody[][2], uint16_t notes, uint16_t tempo) {
	for (uint16_t i = 0; i < notes; i++) {
		uint32_t duration = (60000 / tempo) * melody[i][1] / 4;
		playTone(melody[i][0], duration);

		// Short pause between notes (20% of note duration)
		htim4.Instance->CCR1 = 0;
		HAL_Delay(duration / 5);
	}
}

void policeSiren(uint32_t cycles) {
	for (uint32_t i = 0; i < cycles; i++) {
		// Rising pitch
		for (uint32_t freq = 800; freq <= 1500; freq += 10) {
			playTone(freq, 10);
		}
		// Falling pitch
		for (uint32_t freq = 1500; freq >= 800; freq -= 10) {
			playTone(freq, 10);
		}
	}
}

void playClick(uint32_t duration_ms) {
	htim4.Instance->CCR1 = 10;  // Very short pulse
	HAL_Delay(duration_ms);
	htim4.Instance->CCR1 = 0;
}


void playOdeToJoy(){
	static const uint16_t odeToJoy[][2] = {
			{NOTE_E4, 4}, {NOTE_E4, 4}, {NOTE_F4, 4}, {NOTE_G4, 4},
			{NOTE_G4, 4}, {NOTE_F4, 4}, {NOTE_E4, 4}, {NOTE_D4, 4},
			{NOTE_C4, 4}, {NOTE_C4, 4}, {NOTE_D4, 4}, {NOTE_E4, 4},
			{NOTE_E4, 6}, {NOTE_D4, 2}, {NOTE_D4, 2},

			{NOTE_E4, 4}, {NOTE_E4, 4}, {NOTE_F4, 4}, {NOTE_G4, 4},
			{NOTE_G4, 4}, {NOTE_F4, 4}, {NOTE_E4, 4}, {NOTE_D4, 4},
			{NOTE_C4, 4}, {NOTE_C4, 4}, {NOTE_D4, 4}, {NOTE_E4, 4},
			{NOTE_D4, 6}, {NOTE_C4, 2}, {NOTE_C4, 2}
	};
	playMelody(odeToJoy, sizeof(odeToJoy) / sizeof(odeToJoy[0]), 130);
}

void playInTheEnd(){
	static const uint16_t inTheEnd[][2] = {
			{NOTE_DS3, 4}, {NOTE_AS3, 4}, {NOTE_AS3, 4}, {NOTE_FS3, 4},
			{NOTE_F3, 6}, {NOTE_F3, 6}, {NOTE_F3, 6}, {NOTE_F3, 4},
			{NOTE_FS3, 4}, {NOTE_DS3, 4}, {NOTE_FS3, 4}, {NOTE_AS3, 4},
			{NOTE_DS4, 4}, {NOTE_F4, 4}, {NOTE_FS4, 4}, {NOTE_F4, 4}
	};
	playMelody(inTheEnd, sizeof(inTheEnd)/sizeof(inTheEnd[0]), 110);
}
