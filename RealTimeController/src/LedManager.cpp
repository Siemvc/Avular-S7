#include "LedManager.h"

void LedManager::begin(int pin, int numLeds) {
    _numLeds = numLeds;
    _leds = new CRGB[_numLeds];

    FastLED.addLeds<WS2812, 23, GRB>(_leds, _numLeds); 
    
    FastLED.setBrightness(50);

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
        case Startup:        effectFlash(CRGB::Green2, 300); break;
        case Shutdown:       effectBreathing(CRGB::Magenta3); break;
        case Standby:        effectBreathing(CRGB::Blue3); break;
        case Operational:    effectBreathing(CRGB::Green2); break;
        case Driving:        solidEverywhere(CRGB::Green2); break;
        case E_Brake:        solidEverywhere(CRGB::Red2); break;
        case Low_power:      solidEverywhere(CRGB::Yellow1); break;
        case battery_empty:  effectBlinking(CRGB::Red1, CRGB::Orange, 500, 500, _numLeds / 2); break;
        case battery_dead:   effectBlinking(CRGB::Red1, CRGB::Black, 200, 800, _numLeds / 2); break;
        case Overheating:       effectBreathing(CRGB::Orange); break;
        case ERR:            effectBreathing(CRGB::Yellow1); break;
        case Linux_boot_ERR: effectFlash(CRGB::Yellow1, 200); break;
        default:             effectBreathing(CRGB::Amethyst); break;
    }
}

void LedManager::effectBreathing(CRGB color) {
    float breath = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0;
    for(int i=0; i < _numLeds; i++) {
        _leds[i] = color;
        _leds[i].fadeToBlackBy(255 - breath);
    }
}

void LedManager::effectFlash(CRGB color, int interval) {
    CRGB current = (millis() / interval) % 2 == 0 ? color : CRGB::Black;
    fill_solid(_leds, _numLeds, current);
}

void LedManager::solidEverywhere(CRGB color) {
    fill_solid(_leds, _numLeds, color);
}

void LedManager::effectBlinking(CRGB color1, CRGB color2, int onDuration, int offDuration, int numLeds) {
    unsigned long cycleTime = onDuration + offDuration;
    unsigned long timeInCycle = millis() % cycleTime;

    bool firstPhase = (timeInCycle < onDuration);

    CRGB firstHalf = firstPhase ? color1 : color2;
    fill_solid(&_leds[0], numLeds, firstHalf);
    
    CRGB secondHalf = firstPhase ? color2 : color1;
    fill_solid(&_leds[numLeds], numLeds, secondHalf);
}