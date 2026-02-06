#include "LedManager.h"

void LedManager::begin(int pin, int numLeds) {
    _numLeds = numLeds;
    _leds = new CRGB[_numLeds];

    FastLED.addLeds<WS2812, 23, GRB>(_leds, _numLeds); 
    FastLED.setBrightness(_brightness);

    _currentMode = Startup;
    _startTime = millis();
}

void LedManager::setState(avuloaderState newState) {
    _currentMode = newState;
}

void LedManager::update() {
    handleLeds();
    FastLED.show();
}

void LedManager::handleLeds() {
    switch (_currentMode) {
        case Startup:           effectBlinking(CRGB::Green1, CRGB::Orange, 200, 200, _numLeds / 2); break;
        case Shutdown:          solidEverywhere(CRGB::Magenta3); break;
        case Standby:           solidEverywhere(CRGB::Blue3); break;
        case Operational:       effectBreathing(CRGB::Green2); break;
        case Driving:           solidEverywhere(CRGB::Green2); break;
        case E_Brake:           solidEverywhere(CRGB::Red2); break;
        case Low_power:         solidEverywhere(CRGB::Yellow1); break;
        case battery_empty:     effectBlinking(CRGB::Red1, CRGB::Orange, 500, 500, _numLeds / 2); break;
        case battery_dead:      effectBlinking(CRGB::Red1, CRGB::Black, 200, 800, _numLeds / 2); break;
        case Overheating:       effectBreathing(CRGB::Orange); break;
        case ERR:               effectBreathing(CRGB::Yellow1); break;
        case Linux_boot_ERR:    effectFlash(CRGB::Yellow1, 200); break;
        default:             effectBreathing(CRGB::Amethyst); break;
    }
}

void LedManager::effectBreathing(CRGB color) {
    // compute a smooth 0..255 breathing value
    float raw = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
    uint8_t breath = (uint8_t)constrain((int)raw, 0, 255);

    // set overall FastLED brightness to the breathing
    uint8_t combined = scale8(breath, _brightness);
    FastLED.setBrightness(combined);

    // Fill every LED with the given color
    fill_solid(_leds, _numLeds, color);
}

void LedManager::effectFlash(CRGB color, int interval) {
    CRGB current = (millis() / interval) % 2 == 0 ? color : CRGB::Black;
    FastLED.setBrightness(_brightness);
    fill_solid(_leds, _numLeds, current);
}

void LedManager::solidEverywhere(CRGB color) {
    FastLED.setBrightness(_brightness);
    fill_solid(_leds, _numLeds, color);
}

void LedManager::effectBlinking(CRGB color1, CRGB color2, int onDuration, int offDuration, int numLeds) {
    unsigned long cycleTime = onDuration + offDuration;
    unsigned long timeInCycle = millis() % cycleTime;
    FastLED.setBrightness(_brightness);
    bool firstPhase = (timeInCycle < onDuration);
    //First half
    fill_solid(&_leds[0], numLeds, firstPhase ? color1 : color2);
    //Second half
    fill_solid(&_leds[numLeds], numLeds, firstPhase ? color2 : color1);
}