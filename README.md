# Arduino Piezo Buzzer - Never Gonna Give You Up

Plays a rendition of Rick Astley's "Never Gonna Give You Up" on a passive piezo buzzer connected to an Arduino Uno (or compatible board). This version uses direct AVR C code and Timer1 for optimized PWM tone generation within the Arduino environment.

## Hardware Required

* Arduino Uno (or other ATmega328P based board)
* Passive Piezo Buzzer

## Wiring

Connect the piezo buzzer:
* Positive (+) pin to Arduino Digital Pin **D9** (PB1 / OC1A)
* Negative (-) pin to Arduino **GND**

## Software Required

* **Arduino IDE:** (Version 1.8.x or 2.x recommended)
* **`pitches.h`:** A header file containing standard note frequency definitions (e.g., `NOTE_C4`, `NOTE_FS5`). You can obtain a standard version from: [https://gist.github.com/mikeputnam/2820675](https://gist.github.com/mikeputnam/2820675). Place this file in the same directory as your `.ino` sketch file.

## How to Use

1.  Install the Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software).
2.  Download or copy the code into a new sketch (`rick_astley_piezo.ino`).
3.  Download `pitches.h` from the link above and save it in the *same folder* as your `.ino` file.
4.  Connect your Arduino Uno board via USB.
5.  In the Arduino IDE, select the correct Board (Arduino Uno) and Port from the Tools menu.
6.  Click the "Upload" button (arrow icon).

The Arduino will reset, and the song should start playing through the piezo buzzer.

## Code Explanation

* **Tone Generation:** Uses Timer1 in Fast PWM mode (Mode 14, TOP=ICR1).
* **Frequency Control:** The `ICR1` register is set based on the desired note frequency (`Freq = 16MHz / 8 / (ICR1 + 1)`).
* **Output:** Arduino Pin D9 (PB1/OC1A) outputs the PWM signal.
* **Duty Cycle:** Optimized for piezo buzzers with an approximately 50% duty cycle (`OCR1A = ICR1 / 2`).
* **Silence:** Rests (`NOTE_0`) are handled cleanly by disconnecting the Timer output compare pin (`COM1A` bits in `TCCR1A`) rather than just setting the duty cycle to 0%.
* **Timing:** Uses the standard Arduino `delay()` function for pauses. Note durations include a small silence gap (typically 10%) for note articulation. High frequency notes have been slightly shortened to reduce potential harshness.

## Customization

* **Tempo:** The tempo is implicitly set by the `delay()` values in the `loop()` function (currently around 110 bpm). To change the tempo, all delay values need to be recalculated based on a new desired tempo and the corresponding note durations.
* **Pin:** Changing the output pin requires modifying the `DDRB` and potentially `PORTB` settings (if using a different port) and likely configuring a different Timer/Output Compare Pin (e.g., OC1B, OC2A/B) along with associated register changes (TCCRxA/B, OCRxA/B). Pin D9 (OC1A) is used here.
* **Song Data:** The note sequence is defined in the `loop()` function.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

* Song: "Never Gonna Give You Up" by Rick Astley
* Note frequency definitions (`pitches.h`) sourced from: [https://gist.github.com/mikeputnam/2820675](https://gist.github.com/mikeputnam/2820675)
* Initial piezo playing concepts inspired by various online Arduino music projects.
