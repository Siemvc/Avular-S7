#include <FastLED.h>

const int ledPin = 6;       
const int numLeds = 60;       // Je hebt 60 LEDs
const int ledsPerHoek = 2;
int brightness = 255;       // Helderheid (0-255)

#define ledType    WS2812
#define colorOrder GRB

CRGB leds[numLeds];

//Gedifineerde secties van rgb strip
const int hoekLV = 0;
const int hoekRV = 1;
const int hoekRA = 2;
const int hoekLA = 3;

enum avuloaderState {
  BOOTING,
  IDLE,
  DRIVING,
  TURNING_LEFT,
  TURNING_RIGHT,
  ERROR,
  OBSTACLE
};

avuloaderState currentState = BOOTING;

void setup() {
  delay(2000); // Veiligheidspauze bij opstarten

  // FastLED setup
  FastLED.addLeds<ledType, ledPin, colorOrder>(leds, numLeds);
  FastLED.setBrightness(brightness);

  EffectBootup();
  currentState = IDLE;
}

void HandleLeds() {
  switch (currentState) {
    case IDLE:
      EffectBreathing(CRGB::Cyan); // Rustig ademen
      break;
    case DRIVING:
      SetHeadlightsTaillights();   // Wit voor, Rood achter
      break;
    case TURNING_LEFT:
      EffectBlinker(hoekLV, hoekLA); // Knipperlicht links
      break;
    case TURNING_RIGHT:
      EffectBlinker(hoekRV, hoekRA); // Knipperlicht rechts
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

void SetHeadlightsTaillights() {
  SetCorner(hoekLV, CRGB::White);
  SetCorner(hoekRV, CRGB::White);
  SetCorner(hoekLA, CRGB::Red);
  SetCorner(hoekRA, CRGB::Red);
}

void EffectBlinker(int cornerFront, int cornerBack) {
  // Knipperen op 2 Hz (elke 500ms)
  CRGB color = (millis() / 350) % 2 == 0 ? CRGB::Orange : CRGB::Black;
  
  // Zet de andere hoeken uit of op 'dim'
  fill_solid(leds, numLeds, CRGB::Black); 
  
  SetCorner(cornerFront, color);
  SetCorner(cornerBack, color);
}

void EffectBreathing(CRGB color) {
  // Gebruik een sinusgolf voor soepel ademen
  float breath = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0;
  
  for(int i=0; i < numLeds; i++) {
    // We gebruiken 'i' niet, want alles ademt tegelijk
    // Je kunt hier controleren of 'i' binnen de hoek-leds valt
    if (i < 4 * ledsPerHoek) {
      leds[i] = color;
      leds[i].fadeToBlackBy(255 - breath);
    } else {
      leds[i] = CRGB::Black;
    }
  }
}

void EffectPolice() {
  // Stroboscoop effect: Blauw/Rood wisselen heel snel
  int state = (millis() / 100) % 2;
  if(state == 0) {
    SetCorner(hoekLV, CRGB::Blue);
    SetCorner(hoekRA, CRGB::Blue);
    SetCorner(hoekRV, CRGB::Red);
    SetCorner(hoekLA, CRGB::Red);
  } else {
    SetCorner(hoekLV, CRGB::Red);
    SetCorner(hoekRA, CRGB::Red);
    SetCorner(hoekRV, CRGB::Blue);
    SetCorner(hoekLA, CRGB::Blue);
  }
}

void EffectFlashRed() {
  CRGB color = (millis() / 100) % 2 == 0 ? CRGB::Red : CRGB::Black;
  fill_solid(leds, 12, color); // Zet alleen de eerste 12 leds aan
}

void EffectBootup() {
  // Knight rider style "vullen"
  for(int i=0; i<4; i++) {
    SetCorner(i, CRGB::Green);
    FastLED.show();
    delay(500);
    SetCorner(i, CRGB::Black);
  }
  fill_solid(leds, 12, CRGB::Green);
  FastLED.show();
  delay(500);
  fill_solid(leds, 12, CRGB::Black);
}

// Simpele demo switcher
void NextDemoState() {
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
  HandleLeds(); // Update de LEDs op basis van de huidige status
  FastLED.show();
  
  // Simulatie demo (verwijder dit in je echte rover code)
  EVERY_N_SECONDS(5) {
    NextDemoState();
  }
}