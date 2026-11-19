/**
 * The emulated teensy system used for testing.
 */
#include "../src/teensy.hpp"

int main(void) {
    teensy_setup(0);  // No delay, start immediately
    for (;;) {
        teensy_loop();
    }
    return 0;
}
