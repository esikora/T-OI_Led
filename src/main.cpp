/* --------------------------------------------------------------------------------------------
 * Prerequisites:
 * Install USB serial driver: https://github.com/LilyGO/LILYGO-T-OI/blob/master/CH341SER_2.EXE
 * -------------------------------------------------------------------------------------------- */

#include <Arduino.h>

// External library: FastLED, https://github.com/FastLED/FastLED
#include <FastLED.h>

// T-OI Pin "GPIO4" connected to "Data In" of RGB Led module
const byte kPinLedStrip = 4;

// Number of Leds
const byte kNumLeds = 7;

// LED strip controller
CRGB ledStrip_[kNumLeds];

// ADC_MODE(ADC_VCC);

/**
 * Led strip effect: Constant white light e.g. for reading.
 */
void renderConstant(const CRGB& color)
{
    // Update color of each LED 
    for (int ledNr = 0; ledNr < kNumLeds; ledNr++)
    {
        // Update color of the current LED to chosen RGB value
        ledStrip_[ledNr] = color;
    }
}


/**
 * Fade brightness from start value to end value with a defined delay after each step.
 */
void fadeBrightness(uint8_t startBr, uint8_t endBr, uint16_t stepDelay)
{
    int8_t brStep;
    uint8_t numSteps;

    // Determine increment and number of steps
    if (startBr <= endBr)
    {
        brStep = 1;
        numSteps = endBr - startBr + 1;
    }
    else
    {
        brStep = -1;
        numSteps = startBr - endBr + 1;
    }

    uint8_t curBr = startBr;

    // Change the brightness gradually    
    for (uint8_t i = 0; i < numSteps; i++)
    {
        FastLED.setBrightness(curBr);
        FastLED.show();
        delay(stepDelay);
        curBr += brStep;
    }
}


byte mode_ = 1;

unsigned long modeChangeTime_ = 0;


void setup() {
  Serial.begin(115200);

  FastLED.addLeds<NEOPIXEL, kPinLedStrip>(ledStrip_, kNumLeds);
  FastLED.clear();
  FastLED.setBrightness(20);
  FastLED.show();

  modeChangeTime_ = millis();
}


void loop() {
  // uint16_t vcc = ESP.getVcc(); // Unit: mV
  // Serial.println("Hello from LILYGO TTGO T-OI ESP8266");
  // Serial.println(vcc);

  int delayMs = 100;

  if (mode_ == 1)
  {
    if ( random(0, 2) == 1 )
    {
      renderConstant( CRGB(255, 0, 0) );
    }
    else
    {
      renderConstant( CRGB(0, 0, 255) );
    }

    FastLED.setBrightness(0);
    FastLED.show();
    delay(200);

    fadeBrightness(0, 100, 20);
    fadeBrightness(100, 0, 20);

    FastLED.setBrightness(20);
    delay(200);

    mode_ = (mode_ + 1) % 2;
    modeChangeTime_ = millis();
  }
  else
  {
    byte intensity = (byte) random(120, 256);

    renderConstant( CRGB(intensity, intensity / 2, intensity / 3) );
    FastLED.show();

    delayMs = random(50, 120);

    unsigned long curTime = millis();

    if (curTime - modeChangeTime_ > 30000)
    {
      mode_ = (mode_ + 1) % 2;
      modeChangeTime_ = curTime;
    }
  }

  delay( delayMs );
}