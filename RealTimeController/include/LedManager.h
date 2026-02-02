#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <FastLED.h>

enum avuloaderState {
  Startup,
  Shutdown,
  Standby,
  Operational,
  Driving,
  E_Brake,
  Low_power,
  battery_empty,
  battery_dead,
  Overheating,
  ERR,
  Linux_boot_ERR,
  Shutdown,
  IDLE
};

class LedManager {
public:
    LedManager() : _numLeds(0), _leds(nullptr) {}
    
    void begin(int pin, int numLeds);
    void update();
    void setState(avuloaderState newState);

private:
    int _numLeds;
    CRGB* _leds; 
    avuloaderState _currentMode = Startup;
    unsigned long _startTime;
    void handleLeds();
    void effectBreathing(CRGB color);
    void effectFlash(CRGB color, int interval);
    void solidEverywhere(CRGB color);
    void effectBlinking(CRGB color1, CRGB color2, int onDuration, int offDuration, int numLeds);
};

#endif