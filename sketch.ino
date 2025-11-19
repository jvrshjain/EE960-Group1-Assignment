// Smart Home Simulation - Arduino Uno
// Sensors: DHT22 (pin 2), PIR (pin 3), HC-SR04 (TRIG 4, ECHO 5)
// LCD: RS 7, E 8, D4 9, D5 10, D6 11, D7 12
// LEDs: temperature LED A0, PIR LED A1, ultrasonic LED A2
#include <DHT.h>
#include <LiquidCrystal.h>

// ----- Sensor Pins -----
#define DHTPIN 2
#define DHTTYPE DHT22

#define PIR_PIN 3
#define TRIG_PIN 4
#define ECHO_PIN 5

// LEDs
#define LED_TEMP A0
#define LED_PIR  A1
#define LED_DIST A2

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
DHT dht(DHTPIN, DHTTYPE);

// Student-decided thresholds
float TEMP_THRESHOLD = 28.0;   // Temperature LED ON above this
int DIST_THRESHOLD = 20;       // Distance LED ON below this

// PIR timing control
bool pir_active = false;
unsigned long pir_trigger_time = 0;
unsigned long pir_inhibit_time = 0;



void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  dht.begin();
  lcd.begin(16, 2);

  pinMode(PIR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_TEMP, OUTPUT);
  pinMode(LED_PIR, OUTPUT);
  pinMode(LED_DIST, OUTPUT);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Smart Home Ready");
  delay(1000);
  lcd.clear();
}

void loop() {
  // put your main code here, to run repeatedly:
// ---------- 1. TEMPERATURE ----------
  float t = dht.readTemperature();

  if (!isnan(t)) {
    if (t >= TEMP_THRESHOLD) digitalWrite(LED_TEMP, HIGH);
    else digitalWrite(LED_TEMP, LOW);

    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" C");
  }

  // ---------- 2. PIR MOTION WITH TIMING ----------
  int pir_raw = digitalRead(PIR_PIN);
  unsigned long now = millis();

  if (!pir_active) {
    if (pir_raw == HIGH && now > pir_inhibit_time) {
      pir_active = true;
      pir_trigger_time = now;
      Serial.println("Motion Detected!");
      digitalWrite(LED_PIR, HIGH);
    }
  } else {
    // Keep motion HIGH for 5 seconds
    if (now - pir_trigger_time >= 5000) {
      pir_active = false;
      digitalWrite(LED_PIR, LOW);
      Serial.println("Motion Ended.");
      pir_inhibit_time = now + 1200; // inhibit for 1.2s
    }
  }

  // ---------- 3. ULTRASONIC DISTANCE ----------
  long distance = getDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 0 && distance < DIST_THRESHOLD)
    digitalWrite(LED_DIST, HIGH);
  else
    digitalWrite(LED_DIST, LOW);

  // ---------- LCD DISPLAY ----------
  lcd.setCursor(0,0);
  lcd.print("T:");
  lcd.print(t,1); 
  lcd.print((char)223);
  lcd.print("C ");

  lcd.print("D:");
  lcd.print(distance);

  lcd.setCursor(0,1);
  lcd.print("PIR:");
  lcd.print(pir_active ? "YES " : "NO  ");

  delay(500);
}

// ----------- Ultrasonic Function -----------
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration / 58;

}
