const int IN1A = D5;
const int IN2A = D6;
const int EN_A = D7;
const int IN1B = D2;
const int IN2B = D3;
const int EN_B = D4;



void setup() {
  Serial.begin(115200);
  Serial.println("NodeMCU Actuator Control Gestart");
  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(EN_A, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2B, OUTPUT);
  pinMode(EN_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(EN_A, HIGH);

}

void loop() {
  Serial.println("Vooruit");
  digitalWrite(LED_BUILTIN, HIGH); 
  analogWrite(IN1A, 50);
  digitalWrite(IN2A, LOW);
  delay(1000);

  Serial.println("Stop");
  digitalWrite(LED_BUILTIN, LOW); 
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, LOW);
  delay(1000);
  Serial.println("Achteruit");
  digitalWrite(LED_BUILTIN, HIGH); 
  digitalWrite(IN1A, LOW);
  analogWrite(IN2A, 50);
  delay(1000);

  Serial.println("Stop");
  digitalWrite(LED_BUILTIN, LOW); 
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, LOW);
  delay(1000);

}
