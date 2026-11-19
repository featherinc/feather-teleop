/**
 * Small wrapper for actual teensy compilation.
 * Makefile also passes -DCOMPILE_FOR_TEENSY via build.flags.defs for .c/.cpp units.
 */
#include "src/teensy.hpp"
#include <Arduino.h>

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    teensy_setup(5000);  // 5 second delay to make sure setial monitor can start up during testing
}

void loop() {
    digitalWrite(LED_BUILTIN, (millis() / 2000u) & 1);
    teensy_loop();
}
