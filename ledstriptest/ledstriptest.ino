//Sketch uses 4072 bytes (1%) of program storage space. Maximum is 253952 bytes.
//Global variables use 1500 bytes (18%) of dynamic memory, leaving 6692 bytes for local variables. Maximum is 8192 bytes.

#include <FastLED.h>
#define PIN        6
#define NUMPIXELS  461

CRGB leds[NUMPIXELS];
bool isRed = true;

void setup() {

}

void loop() {
  fastLedUpdate();
}

void fastLedUpdate() {
  FastLED.addLeds<NEOPIXEL, PIN>(leds, NUMPIXELS);
  if (isRed) {
    fill_solid(leds, NUMPIXELS, CRGB::Blue);
  } else {
    fill_solid(leds, NUMPIXELS, CRGB::Red);
  }
  FastLED.show();
  isRed = !isRed;
}
