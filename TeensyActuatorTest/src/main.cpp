#include <Arduino.h>

// Pinnen
const int PIN_MOTOR_A = 29;   // IN1
const int PIN_MOTOR_B = 28;   // IN2
const int PIN_POTMETER = 27; // Potmeter

// P-Controller Instellingen
float Kp = 2.4;           // "Agressiviteit": Hoger = sneller reageren, maar kan gaan trillen
int deadband = 20;        // Marge: Als we binnen X punten zijn, vinden we het goed (voorkomt trillen)
int min_pwm = 60;         // Minimale kracht die de motor nodig heeft om te bewegen (tegen wrijving)
int max_pwm = 245;        // Maximale snelheid (0-255)

// Variabelen
int targetPosition = -1;  // Waar willen we heen? (-1 betekent nog geen doel)
int currentPosition = 0;  // Waar zijn we nu?

void setup() {
  Serial.begin(115200);
  
  // Pinnen instellen
  pinMode(PIN_MOTOR_A, OUTPUT);
  pinMode(PIN_MOTOR_B, OUTPUT);
  pinMode(PIN_POTMETER, INPUT);

  // Teensy ADC resolutie instellen (standaard 10 bit: 0-1023)
  analogReadResolution(10); 

  Serial.println("--- Avuloader P-Controller Start ---");
  Serial.println("Typ een getal tussen 50 en 950 om de actuator te bewegen.");
  
  // Lees de startpositie
  currentPosition = analogRead(PIN_POTMETER);
  targetPosition = currentPosition; // Blijf staan waar je bent bij opstarten
}

// ---------------------------------------------------------------------------
// MOTOR AANSTURING FUNCTIE
// ---------------------------------------------------------------------------
void stuurMotor(int snelheid) {
  // Snelheid is een getal tussen -255 en 255
  
  // Begrens de input voor de zekerheid
  snelheid = constrain(snelheid, -245, 245);

  if (snelheid == 0) {
    // STOP
    digitalWrite(PIN_MOTOR_A, LOW);
    digitalWrite(PIN_MOTOR_B, LOW);
  }
  else if (snelheid < 0) {
    // NAAR BINNEN (Retract)
    // Jouw logica: Pin A Hoog, Pin B Laag
    // We gebruiken de absolute waarde van snelheid voor PWM (dus van -200 maken we 200)
    analogWrite(PIN_MOTOR_A, abs(snelheid)); 
    digitalWrite(PIN_MOTOR_B, LOW);
  }
  else if (snelheid > 0) {
    // NAAR BUITEN (Extend)
    // Jouw logica: Andersom -> Pin A Laag, Pin B Hoog
    digitalWrite(PIN_MOTOR_A, LOW);
    analogWrite(PIN_MOTOR_B, abs(snelheid));
  }
}


void loop() {
  // 1. Check of er een nieuw doel is via USB
  if (Serial.available() > 0) {
    int input = Serial.parseInt();
    // Filter ongeldige inputs (newlines etc)
    if (input > 10 && input < 786) {
      targetPosition = input;
      Serial.print("Nieuw Doel Ontvangen: ");
      Serial.println(targetPosition);
    }
    else{Serial.println("Out of range!");}
  }

  // 2. Lees Huidige Positie
  currentPosition = analogRead(PIN_POTMETER);

  // 3. Bereken de Fout (Error)
  int error = targetPosition - currentPosition;

  // 4. Bepaal Actie (P-Control)
  int pwmOutput = 0;

  // Zit je binnen de marge (deadband)? Doe dan niks (anders blijft hij zoemen)
  if (abs(error) < deadband) {
    pwmOutput = 0;
  } 
  else {
    // P-Regeling: Output = Fout * Versterking
    pwmOutput = error * Kp;

    // Toevoegen van 'Feedforward' wrijvingcompensatie (Minimale kracht)
    if (pwmOutput > 0) pwmOutput += min_pwm;
    if (pwmOutput < 0) pwmOutput -= min_pwm;

    // Begrenzen op max snelheid van de motor
    pwmOutput = constrain(pwmOutput, -max_pwm, max_pwm);
  }

  // 5. Stuur de motor aan
  // LET OP: Hier bepalen we de richting van de regeling.
  // Aanname: "Naar buiten" (positieve PWM) zorgt voor een HOGERE potmeter waarde.
  // Als je actuator de verkeerde kant op schiet, zet hier een minteken voor pwmOutput!
  stuurMotor(pwmOutput);

  // 6. Debugging (Elke 200ms printen)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.printf("Pos: %d -> Doel: %d | Error: %d | PWM: %d\n", currentPosition, targetPosition, error, pwmOutput);
    lastPrint = millis();
  }
  
  delay(50); // Korte pauze voor stabiliteit
}