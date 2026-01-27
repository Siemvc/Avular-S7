//FIXME: Check common ground !!!!!!
#include <FastLED.h>

const int ledPin = 23;       
const int numLeds = 8;      
const int ledsPerHoek = 2;
int brightness = 50;       

#define ledType    WS2812
#define colorOrder GRB

CRGB leds[numLeds];

// Sections of RGB strip
const int Corner_Left_front = 0;
const int Corner_RIGHT_front = 1;
const int Corner_RIGHT_Back = 2;
const int Corner_Left_Back = 3;

// We define the CATEGORY of states here
enum avuloaderState {
  Startup,
  Shutdown,
  Standby,
  Operational,
  Driving,
  E_Brake,
  Low_power,
  ERR,
  Linux_boot_ERR,
  IDLE
};

// We create the actual VARIABLE here (renamed to currentMode to avoid the error)
avuloaderState currentMode = Startup;

void setup() {
  delay(2000); // Safety delay for startup

  FastLED.addLeds<ledType, ledPin, colorOrder>(leds, numLeds);
  FastLED.setBrightness(brightness);
  unsigned long startTime = millis();
  while(millis()-startTime < 5000){
    EffectSlowFlash(CRGB::Green2);
    FastLED.show();
  }
  currentMode = Standby;
}

void loop() {
  HandleLeds(); // Update LEDs to current state
  FastLED.show();
  
  // Simulate demo cycle every 5 seconds
  EVERY_N_SECONDS(5) {
    NextDemoState();
  }
}
void HandleLeds() {
  switch (currentMode) {
    case Shutdown:
      EffectBreathing(CRGB::Magenta3);
      break;
    case Standby:
      EffectBreathing(CRGB::Blue3);
      break;
    case Operational:
      EffectBreathing(CRGB::Green2);
      break;
    case Driving:
      Solid_everywhere(CRGB::Green2);
      break;
    case E_Brake:
      Solid_everywhere(CRGB::Red2);
      break;
    case Low_power:
      Solid_everywhere(CRGB::Yellow1);
      break;
    case ERR:
      EffectBreathing(CRGB::Yellow1);
      break;
    case Linux_boot_ERR:
      EffectFlash(CRGB::Yellow1);
      break;
    default:
      EffectBreathing(CRGB::Amethyst);
      break;
  }
}
void NextDemoState() {
  switch (currentMode) {
    case Shutdown: currentMode = Standby; break;
    case Standby: currentMode = Operational; break;
    case Operational: currentMode = Driving; break;
    case Driving: currentMode = E_Brake; break;
    case E_Brake: currentMode = Low_power; break;
    case Low_power: currentMode = ERR; break;
    case ERR: currentMode = Linux_boot_ERR; break; 
    case Linux_boot_ERR: currentMode = Shutdown; break;
    default: currentMode = IDLE; break;
  }
}

// --- HELPER FUNCTIONS ---

void SetCorner(int cornerIndex, CRGB color) {
  int startLed = cornerIndex * ledsPerHoek;
  for(int i = 0; i < ledsPerHoek; i++) {
    leds[startLed + i] = color;
  }
}

void SetHeadlightsTaillights() {
  SetCorner(Corner_Left_front, CRGB::White);
  SetCorner(Corner_RIGHT_front, CRGB::White);
  SetCorner(Corner_Left_Back, CRGB::Red);
  SetCorner(Corner_RIGHT_Back, CRGB::Red);
}

void EffectBreathing(CRGB color) {
  float breath = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0;
  for(int i=0; i < numLeds; i++) {
    if (i < 4 * ledsPerHoek) {
      leds[i] = color;
      leds[i].fadeToBlackBy(255 - breath);
    } else {
      leds[i] = CRGB::Black;
    }
  }
}

void Solid_everywhere(CRGB color){
  SetCorner(Corner_Left_front, color);
  SetCorner(Corner_RIGHT_front, color);
  SetCorner(Corner_Left_Back, color);
  SetCorner(Corner_RIGHT_Back, color);
}


void EffectPolice() {
  int state = (millis() / 100) % 2;
  if(state == 0) {
    SetCorner(Corner_Left_front, CRGB::Blue);
    SetCorner(Corner_RIGHT_Back, CRGB::Blue);
    SetCorner(Corner_RIGHT_front, CRGB::Red);
    SetCorner(Corner_Left_Back, CRGB::Red);
  } else {
    SetCorner(Corner_Left_front, CRGB::Red);
    SetCorner(Corner_RIGHT_Back, CRGB::Red);
    SetCorner(Corner_RIGHT_front, CRGB::Blue);
    SetCorner(Corner_Left_Back, CRGB::Blue);
  }
}

void EffectFlash(CRGB flash_colour) {
  CRGB color = (millis() / 200) % 2 == 0 ? flash_colour : CRGB::Black;
  fill_solid(leds, numLeds, color); 
}

void EffectSlowFlash(CRGB flash_colour) {
  CRGB color = (millis() / 300) % 2 == 0 ? flash_colour : CRGB::Black;
  fill_solid(leds, numLeds, color); 
}

void EffectStartup() {
  for(int j = 0; j <= 3 ; j++){
    for(int i=0; i<4; i++) {
      SetCorner(i, CRGB::Green);
      FastLED.show();
      delay(300);
      SetCorner(i, CRGB::Black);
    }
  }
  fill_solid(leds, 8, CRGB::Green);
  FastLED.show();
  delay(500);
  fill_solid(leds, 8, CRGB::Black);
}

