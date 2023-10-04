/**
 * Volodymyr Burdeinyi
 *
 * ATtiny404 @ 4 MHz (internal oscillator; BOD disabled)
 *
 * CC BY 4.0
 * Licensed under a Creative Commons Attribution 4.0 International license: 
 * http://creativecommons.org/licenses/by/4.0/
 */
#include <Arduino.h>

#include <avr/io.h>
#include <avr/sleep.h>

const byte pinSync = 0;
const byte pinA = 3;
const byte pinB = 1;
const byte pinC = 2;
const byte pinD = 4;
const byte pinUnused = 5;
const byte chainPins[] = {pinA, pinB, pinC, pinD};
byte chainLength = sizeof(chainPins);

const byte mainsFrequencyHz = 100;
const uint32_t dutyCycleLengthMicros = 1000000 / mainsFrequencyHz;
const uint32_t cycleEndGapMicros = 3000;

uint32_t stateA = 1;
uint32_t stateB = 1;
uint32_t stateC = 1;
uint32_t stateD = 1;
uint32_t chainPinsState[] = {0, 0, 0, 0};

volatile bool hasTick = false;
volatile uint32_t isrStartAtMicros = 0;
volatile uint32_t _cycleDurationMicros = 0;

// zvs stands for zero voltage switch
void zvsCycle() {
    uint32_t cm = micros();  //current millis

    _cycleDurationMicros = cm - isrStartAtMicros;
    

    for (byte i = 0; i < chainLength; i++) {
        byte currentPin = chainPins[i];
        uint32_t currentPinDuration = chainPinsState[i];

        
        if (_cycleDurationMicros > (dutyCycleLengthMicros - currentPinDuration)) {
            digitalWrite(currentPin, 1);
        }
    }

    if (_cycleDurationMicros < (dutyCycleLengthMicros - cycleEndGapMicros)) {
        zvsCycle();
    }
}

void afterZvsCycle() {
    for (byte i = 0; i < chainLength; i++) {
        byte currentPin = chainPins[i];
        digitalWrite(currentPin, 0);
    }
}


uint8_t const firstProgramNumber = 1;
uint8_t const secondProgramNumber = 2;
uint8_t const thirdProgramNumber = 3;
uint8_t currentProgramNumber = firstProgramNumber;
uint8_t const totalProgramsCount = 3;
void programEndEvent() {
  currentProgramNumber++;

  if (currentProgramNumber > totalProgramsCount) {
    currentProgramNumber = firstProgramNumber;
  }
}


void program5() {
    const uint16_t minBrightness = 0;
    for (byte i = 0; i < chainLength; i = i + 1) {
        chainPinsState[i] = minBrightness;
    }
}

void program4() {
  const uint16_t programDuraionMillis = 20000;
  const uint32_t subProgramDurationMillis = 12000;
  const uint16_t minStepDurationMillis = 400;
  const uint16_t maxStepDurationMillis = 1000;
  static uint16_t stepDurationMillis = maxStepDurationMillis;

  const uint32_t maxBrightness = dutyCycleLengthMicros - 3800;
  const uint16_t minBrightness = 2400;

  static uint32_t programStartAtMillis;
  static uint32_t lastProgramRunMillis;
  static uint32_t subProgramStartMillis;

  static bool init = false;
  if (!init) {
    init = true;
    for (byte i = 0; i < chainLength; i = i + 1) {
        if (i%3) {
            chainPinsState[i] = maxBrightness;
        } else {
            chainPinsState[i] = minBrightness;
        }      
    }

    programStartAtMillis = millis();
    lastProgramRunMillis = millis();
    subProgramStartMillis = millis();
  }

  if ((lastProgramRunMillis + stepDurationMillis) > millis()) {
    return;
  }


  if ((subProgramStartMillis + subProgramDurationMillis) < millis()) {
      static const byte directionForward = 1;
      static const byte directionBackward = 0;
      static byte direction = directionForward;

      if (stepDurationMillis < minStepDurationMillis) {
        direction = directionBackward;
      } else if (stepDurationMillis > maxStepDurationMillis) {
        direction = directionForward;
      }

      if (direction == directionForward) {
        stepDurationMillis = stepDurationMillis / 2;
      } else {
        stepDurationMillis = stepDurationMillis * 2;
      }

      subProgramStartMillis = millis();
  }


  static uint8_t chainPinStateNumberOff = 0;
  for (byte i = 0; i < chainLength; i = i + 1) {
    if (i == chainPinStateNumberOff) {
      chainPinsState[i] = minBrightness;
    } else {
      chainPinsState[i] = maxBrightness;
    }
  }
  chainPinStateNumberOff++;
  if (chainPinStateNumberOff > chainLength) {
      chainPinStateNumberOff = 0;
  }


  if (programStartAtMillis + programDuraionMillis < millis()) {
      init = false;
      
      programEndEvent();
  }

  lastProgramRunMillis = millis();
}

void program3() {
  const uint16_t programDuraionMillis = 20000;
  const uint32_t subProgramDurationMillis = 12000;
  const uint16_t minStepDurationMillis = 400;
  const uint16_t maxStepDurationMillis = 1000;
  static uint16_t stepDurationMillis = maxStepDurationMillis;

  const uint32_t maxBrightness = dutyCycleLengthMicros - 3800;
  const uint16_t minBrightness = 2400;

  static uint32_t programStartAtMillis;
  static uint32_t lastProgramRunMillis;
  static uint32_t subProgramStartMillis;

  static bool init = false;
  if (!init) {
    init = true;
    for (byte i = 0; i < chainLength; i = i + 1) {
        if (i%2) {
            chainPinsState[i] = maxBrightness;
        } else {
            chainPinsState[i] = minBrightness;
        }      
    }

    programStartAtMillis = millis();
    lastProgramRunMillis = millis();
    subProgramStartMillis = millis();
  }

  if ((lastProgramRunMillis + stepDurationMillis) > millis()) {
    return;
  }


  if ((subProgramStartMillis + subProgramDurationMillis) < millis()) {
      static const byte directionForward = 1;
      static const byte directionBackward = 0;
      static byte direction = directionForward;

      if (stepDurationMillis < minStepDurationMillis) {
        direction = directionBackward;
      } else if (stepDurationMillis > maxStepDurationMillis) {
        direction = directionForward;
      }

      if (direction == directionForward) {
        stepDurationMillis = stepDurationMillis / 2;
      } else {
        stepDurationMillis = stepDurationMillis * 2;
      }

      subProgramStartMillis = millis();
  }


  for (byte i = 0; i < chainLength; i = i + 1) {
    if (chainPinsState[i] < maxBrightness) {
      chainPinsState[i] = maxBrightness;
    } else {
      chainPinsState[i] = minBrightness;
    }
  }


  if (programStartAtMillis + programDuraionMillis < millis()) {
      init = false;
      
      programEndEvent();
  }

  lastProgramRunMillis = millis();
}

void program2() {
  const uint16_t programDuraionMillis = 20000;
  const uint16_t subProgramDurationMillis = 12000;
  const uint16_t minStepDurationMillis = 20;
  const uint16_t maxStepDurationMillis = 200;
  static uint16_t stepDurationMillis = maxStepDurationMillis;

  const uint32_t maxBrightness = dutyCycleLengthMicros - 2800;
  const uint32_t minBrightness = 2400;
  const uint32_t incInitial = maxBrightness / ((chainLength) * 12);
  static uint32_t inc = incInitial;
  static int32_t _increments[] = {inc, inc, inc, inc};

  static uint32_t programStartAtMillis;
  static uint32_t lastProgramRunMillis;
  static uint32_t subProgramStartMillis;

  static bool init = false;
  if (!init) {
    init = true;

    for (byte i = 0; i < chainLength; i = i + 1) {
      chainPinsState[i] = i * inc * 12;
    }

    programStartAtMillis = millis();
    lastProgramRunMillis = millis();
    subProgramStartMillis = millis();
  }

  if ((lastProgramRunMillis + stepDurationMillis) > millis()) {
    return;
  }


  if ((subProgramStartMillis + subProgramDurationMillis) < millis()) {
      static const byte directionForward = 1;
      static const byte directionBackward = 0;
      static byte direction = directionForward;

      if (stepDurationMillis < minStepDurationMillis) {
        direction = directionBackward;
      } else if (stepDurationMillis > maxStepDurationMillis) {
        direction = directionForward;
      }

      if (direction == directionForward) {
        stepDurationMillis = stepDurationMillis / 2;
      } else {
        stepDurationMillis = stepDurationMillis * 2;
      }

      subProgramStartMillis = millis();
  }


  for (byte i = 0; i < chainLength; i = i + 1) {
    if (_increments[i] + chainPinsState[i] > maxBrightness) {
      _increments[i] = -inc;
    } else if (_increments[i] + chainPinsState[i] < minBrightness) {
      _increments[i] = inc;
    }

    chainPinsState[i] = chainPinsState[i] + _increments[i];
  }


  if (programStartAtMillis + programDuraionMillis < millis()) {
      init = false;

      programEndEvent();
  }

  lastProgramRunMillis = millis();
}

void setup() {
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(pinC, OUTPUT);
    pinMode(pinD, OUTPUT);
    pinMode(pinUnused, INPUT_PULLUP);
    pinMode(pinSync, INPUT);

//     PORTA.PIN6CTRL = (PORTA.PIN6CTRL & ~PORT_ISC_gm) | PORT_ISC_RISING_gc;
//    PORTA.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_LEVEL_gc;
    PORTA.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_RISING_gc;

    // http://www.gammon.com.au/power
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
}

// @example https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/PinInterrupts.md
ISR(PORTA_PORT_vect) {
    PORTA.INTFLAGS = PORT_INT6_bm;

//   if (flags & 0x04) { // filter event by pin //not needed for current pin
    hasTick = true;
    isrStartAtMicros = micros();
//   }
}

void loop() {
    if (hasTick == true) {
        hasTick = false;

        // program5();
        
        switch (currentProgramNumber) {
            case firstProgramNumber:
                program2();
                break;
            case secondProgramNumber:
                program3();
                break;
            case thirdProgramNumber:
                program4();
                break;
            default:
                break;
        }
        
        zvsCycle();
        afterZvsCycle();

        // low power http://www.technoblogy.com/show?2RA3
        PORTA.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_RISING_gc;
        sleep_cpu();
    }
}
