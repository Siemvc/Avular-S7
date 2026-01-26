#include <FastLED.h>

const int ledPin = 6;       
const int numLeds = 60;      
const int ledsPerHoek = 2;
int brightness = 255;       

#define ledType    WS2812
#define colorOrder GRB

CRGB leds[numLeds];

//Sections of RGB strip
const int Corner_Left_front = 0;
const int Corner_RIGHT_front = 1;
const int Corner_RIGHT_Back = 2;
const int Corner_Left_Back = 3;

enum avuloaderState {
  BOOTING,
  IDLE,
  DRIVING,
  TURNING_LEFT,
  TURNING_RIGHT,
  ERROR,
  OBSTACLE
};

//avuloaderState currentState = BOOTING;

void setup() {
  delay(2000); // Safety delay for startup

  // FastLED setup
  FastLED.addLeds<ledType, ledPin, colorOrder>(leds, numLeds);
  FastLED.setBrightness(brightness);

  EffectBootup();
  currentState = IDLE;
}

void HandleLeds() {
  switch (currentState) {
    case IDLE:
      EffectBreathing(CRGB::Cyan); // Breathing effect in Cyan
      break;
    case DRIVING:
      SetHeadlightsTaillights();   // White front, Red back
      break;
    case TURNING_LEFT:
      EffectBlinker(Corner_Left_front, Corner_Left_Back); // Knipperlicht links
      break;
    case TURNING_RIGHT:
      EffectBlinker(Corner_RIGHT_front, Corner_RIGHT_Back); // Knipperlicht rechts
      break;
    case OBSTACLE:
      EffectPolice(); // Politie stroboscoop
      break;
    case ERROR:
      EffectFlashRed(); // Alles rood knipperen
      break;
  }
}

void SetCorner(int cornerIndex, CRGB color) {
  int startLed = cornerIndex * ledsPerHoek;
  for(int i = 0; i < ledsPerHoek; i++) {
    leds[startLed + i] = color;
  }
}

void EffectBlinker(int cornerFront, int cornerBack) {

  CRGB color = (millis() / 350) % 2 == 0 ? CRGB::Orange : CRGB::Black;   // blinking at 2Hz
  
  // Zet de andere hoeken uit of op 'dim'
  fill_solid(leds, numLeds, CRGB::Black); 
  
  SetCorner(cornerFront, color);
  SetCorner(cornerBack, color);
}

void EffectBreathing(CRGB color) {
  float breath = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0; // use sin for smooth breathingx
  for(int i=0; i < numLeds; i++) {
    if (i < 4 * ledsPerHoek) {
      leds[i] = color;
      leds[i].fadeToBlackBy(255 - breath);
    } else {
      leds[i] = CRGB::Black;
    }
  }
}

void EffectFlashRed() {
  CRGB color = (millis() / 100) % 2 == 0 ? CRGB::Red : CRGB::Black;
  fill_solid(leds, 12, color); // Turn on first 12 LEDs
}

void NextDemoState() { //demo test for functionality
  switch (currentState) {
    case IDLE: currentState = DRIVING; break;
    case DRIVING: currentState = TURNING_LEFT; break;
    case TURNING_LEFT: currentState = TURNING_RIGHT; break;
    case TURNING_RIGHT: currentState = OBSTACLE; break;
    case OBSTACLE: currentState = ERROR; break;
    case ERROR: currentState = IDLE; break;
  }
}

void loop() {
  HandleLeds(); // Update LEDs to current state
  FastLED.show();
  
  // Simulate demo cycle every 5 seconds
  EVERY_N_SECONDS(5) {
    NextDemoState();
  }
}
// void SetHeadlightsTaillights() {
//   SetCorner(Corner_Left_front, CRGB::White);
//   SetCorner(Corner_RIGHT_front, CRGB::White);
//   SetCorner(Corner_Left_Back, CRGB::Red);
//   SetCorner(Corner_RIGHT_Back, CRGB::Red);
// }

// void EffectBootup() {
//   for(int i=0; i<4; i++) {
//     SetCorner(i, CRGB::Green);
//     FastLED.show();
//     delay(500);
//     SetCorner(i, CRGB::Black);
//   }
//   fill_solid(leds, 12, CRGB::Green);
//   FastLED.show();
//   delay(500);
//   fill_solid(leds, 12, CRGB::Black);
// }

// void EffectPolice() {
//   // Stroboscoop effect: Blue/Red changing
//   int state = (millis() / 100) % 2;
//   if(state == 0) {
//     SetCorner(Corner_Left_front, CRGB::Blue);
//     SetCorner(Corner_RIGHT_Back, CRGB::Blue);
//     SetCorner(Corner_RIGHT_front, CRGB::Red);
//     SetCorner(Corner_Left_Back, CRGB::Red);
//   } else {
//     SetCorner(Corner_Left_front, CRGB::Red);
//     SetCorner(Corner_RIGHT_Back, CRGB::Red);
//     SetCorner(Corner_RIGHT_front, CRGB::Blue);
//     SetCorner(Corner_Left_Back, CRGB::Blue);
//   }
// }