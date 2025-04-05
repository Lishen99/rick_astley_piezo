/*
 * ============================================================================
 * Project:     Arduino Piezo Player - Never Gonna Give You Up
 * File:        rick_astley_piezo.ino
 * Author:      [Your Name/GitHub Handle]
 * Date:        April 5, 2025
 * Copyright:   Copyright (c) 2025 Lishen Madusha Amaraweera
 * License:     MIT License (see LICENSE file for details)
 *
 * Description: Plays Rick Astley's "Never Gonna Give You Up" on a passive
 * piezo buzzer connected to Arduino Pin D9 (PB1/OC1A).
 * Uses Timer1 in Fast PWM mode (Mode 14) for optimized tone generation.
 * Relies on the Arduino framework for the delay() function.
 *
 * Hardware:    Arduino Uno (or other ATmega328P based board)
 * Passive Piezo Buzzer (Connected to Pin D9 and GND)
 *
 * Based on:    Song transcription and piezo optimization discussions.
 * ============================================================================
 */

// Include standard AVR definitions for registers - useful even in Arduino IDE
// for direct register access, though often implicitly available.
#include <avr/io.h>

// Contains NOTE_ definitions. Required file.
// Source: https://gist.github.com/mikeputnam/2820675
#include "pitches.h"

#define NOTE_0 0 // Frequency definition for Silence/Rest

// Timer1 clock frequency (CPU_FREQ / Prescaler) = 16MHz / 8 = 2MHz
#define BUZZER_CLOCK (16000000UL / 8)

/*
 * tone(uint16_t freq)
 * Configures Timer1 to output a PWM signal of the specified frequency
 * on the OC1A pin (Arduino D9 / PB1).
 * Optimized for piezo buzzers:
 * - Uses accurate frequency calculation (ICR1 = (Clock / Freq) - 1)
 * - Generates approx. 50% duty cycle square wave (OCR1A = ICR1 / 2)
 * - Handles freq=0 (silence) by disconnecting the OC1A pin output.
 */
void tone(uint16_t freq) {
  if (freq == 0) {
    // Silence: Disconnect OC1A (PB1 / Arduino D9) from timer output compare logic
    TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
    return;
  } else {
    // Note Active: Connect OC1A pin in non-inverting PWM mode
    TCCR1A &= ~(1 << COM1A0); // Ensure COM1A0 is 0
    TCCR1A |= (1 << COM1A1);  // Set COM1A1 for non-inverting mode (Clear on Compare Match)
  }

  // Calculate timer TOP value (ICR1) for desired frequency
  uint32_t temp_icr = BUZZER_CLOCK / freq; // Use 32-bit intermediate calculation prevents overflow

  // Check if frequency is valid (avoid ICR1 underflow/wrap-around)
  if (temp_icr > 0) {
    ICR1 = (uint16_t)(temp_icr - 1); // Calculate actual ICR1 and cast back
  } else {
    ICR1 = 0; // Set to highest possible frequency if requested freq is too high
  }

  // Set Compare Match value (OCR1A) for ~50% duty cycle
  // OCR1A = (ICR1 + 1) / 2 - 1; is more precise, but OCR1A = ICR1 / 2; is simpler & often sufficient
  OCR1A = ICR1 / 2;
}

/*
 * ASM Constants (Alternative to using C defines from <avr/io.h>)
 * These define symbolic names for specific hardware register addresses.
 * Note: Using standard C defines (like TCCR1A, PORTB etc.) is generally more portable.
 */
asm("PORTB = 0x05"); // Address of Port B Data Register
asm("DDRB = 0x04");  // Address of Port B Data Direction Register
asm("TCCR1A = 0x80"); // Address of Timer/Counter1 Control Register A
asm("TCCR1B = 0x81"); // Address of Timer/Counter1 Control Register B

/*
 * SETUP: Initializes hardware required for tone generation.
 * Runs once when the Arduino starts or is reset.
 */
void setup() {
  // Configure Timer1 using direct ASM register writes:
  // TCCR1A: Set WGM11=1 (part of Mode 14) initially. COM1A bits handled by tone().
  // TCCR1B: Set WGM13=1, WGM12=1 (Mode 14: Fast PWM, TOP=ICR1), CS11=1 (clk/8 prescaler)
  // Equivalent C:
  // TCCR1A = (1 << WGM11);
  // TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  asm("LDI R16, 0B00000010"); // Only WGM11 set (COM bits will be set in tone())
  asm("STS TCCR1A, R16");
  asm("LDI R16, 0B00011010"); // WGM13, WGM12, CS11 set
  asm("STS TCCR1B, R16");

  // Set Arduino Pin D9 (Port B, Pin 1) as output using ASM
  // Equivalent C: DDRB |= (1 << DDB1);
  asm("SBI DDRB, 1"); // Set PB1 as output
}

/*
 * LOOP: Contains the sequence of notes and delays to play the song.
 * Runs repeatedly after setup() finishes.
 */
void loop() {
  // --- Never Gonna Give You Up - Note Sequence ---
  // Format: tone(NOTE); delay(sound_duration); tone(NOTE_0); delay(silence_duration);
  // Based on Tempo: 110 bpm approx
  // Uses standard Arduino delay() function.
  // Piezo optimizations applied (e.g., slightly shorter high notes).

tone(NOTE_D5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_FS5); delay(670);
tone(NOTE_0); delay(119);  
tone(NOTE_A5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_G5); delay(111);
tone(NOTE_0); delay(20);   
tone(NOTE_FS5); delay(223); 
tone(NOTE_0); delay(40);   
tone(NOTE_D5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(946);
tone(NOTE_0); delay(106);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);  
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);  
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(111);
tone(NOTE_0); delay(20); 
tone(NOTE_D5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_FS5); delay(670); 
tone(NOTE_0); delay(119);  
tone(NOTE_A5); delay(111); 
tone(NOTE_0); delay(20);  
tone(NOTE_G5); delay(111); 
tone(NOTE_0); delay(20);  
tone(NOTE_FS5); delay(223);
tone(NOTE_0); delay(40);  
tone(NOTE_D5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(946);
tone(NOTE_0); delay(106);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_0); delay(526);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_A4); delay(946);
tone(NOTE_0); delay(106);
tone(NOTE_0); delay(526);
tone(NOTE_0); delay(263);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_0); delay(263);
tone(NOTE_A5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_0); delay(526);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_0); delay(263);
tone(NOTE_0); delay(263);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_0); delay(526);
tone(NOTE_0); delay(263);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_FS5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_E5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_0); delay(526);
tone(NOTE_D5); delay(946);
tone(NOTE_0); delay(106);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_FS5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_FS5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_E5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_0); delay(1052);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_0); delay(263);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_FS5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_FS5); delay(334);  
tone(NOTE_0); delay(60);   
tone(NOTE_FS5); delay(334);  
tone(NOTE_0); delay(60);   
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_E5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_E5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_D5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_CS5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(354);
tone(NOTE_0); delay(40);  
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_D5); delay(946);
tone(NOTE_0); delay(106);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_FS5); delay(334);  
tone(NOTE_0); delay(60);   
tone(NOTE_FS5); delay(334);  
tone(NOTE_0); delay(60);   
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_A5); delay(445);  
tone(NOTE_0); delay(81);   
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_CS5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_D5); delay(946);
tone(NOTE_0); delay(106);
tone(NOTE_0); delay(526);
tone(NOTE_0); delay(263);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_0); delay(263);
tone(NOTE_0); delay(263);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_0); delay(526);
tone(NOTE_0); delay(263);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_0); delay(263);
tone(NOTE_A5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_A5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_FS5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_0); delay(263);
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_0); delay(263);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_0); delay(526);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_0); delay(263);
tone(NOTE_0); delay(263);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_FS5); delay(445);  
tone(NOTE_0); delay(81);   
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_D5); delay(946);
tone(NOTE_0); delay(106);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_FS5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_E5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_FS5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_0); delay(789);
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_0); delay(263);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_FS5); delay(223);  
tone(NOTE_0); delay(40);   
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_FS5); delay(334);  
tone(NOTE_0); delay(60);   
tone(NOTE_FS5); delay(334);  
tone(NOTE_0); delay(60);   
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_E5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_E5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_D5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_CS5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(236);  
tone(NOTE_0); delay(27);   
tone(NOTE_B4); delay(354); 
tone(NOTE_0); delay(40);   
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_D5); delay(946);
tone(NOTE_0); delay(106);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_FS5); delay(334);  
tone(NOTE_0); delay(60);   
tone(NOTE_FS5); delay(334);  
tone(NOTE_0); delay(60);   
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_A5); delay(445);  
tone(NOTE_0); delay(81);   
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_CS5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
// Duplicate Section Start
tone(NOTE_E5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_D5); delay(946);
tone(NOTE_0); delay(106);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_FS5); delay(334);  
tone(NOTE_0); delay(60);   
tone(NOTE_FS5); delay(334);  
tone(NOTE_0); delay(60);   
tone(NOTE_E5); delay(710);
tone(NOTE_0); delay(79);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_A5); delay(445);  
tone(NOTE_0); delay(81);   
tone(NOTE_CS5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_D5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_CS5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_A4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_D5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_E5); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_CS5); delay(354);
tone(NOTE_0); delay(40);
tone(NOTE_B4); delay(111); 
tone(NOTE_0); delay(20);   
tone(NOTE_A4); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_A4); delay(236);
tone(NOTE_0); delay(27);
tone(NOTE_E5); delay(473);
tone(NOTE_0); delay(53);
tone(NOTE_D5); delay(946);
tone(NOTE_0); delay(106);

tone(NOTE_0); delay(5000); // Pause before repeating the song

}