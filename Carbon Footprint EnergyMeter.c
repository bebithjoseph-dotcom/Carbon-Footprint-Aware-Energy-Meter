/************ BLYNK CONFIG ************/
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL3kNhEHnAq"
#define BLYNK_TEMPLATE_NAME "Carbon Footprint Aware Energy Meter"
#define BLYNK_AUTH_TOKEN "HBACQstGLNXFPzbmKACPGFvrVFaE6s_k"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

/************ WIFI ************/
char ssid[] = "IOT";
char pass[] = "12345678";

/************ LCD ************/
LiquidCrystal_I2C lcd(0x27, 16, 2);

/************ PINS ************/
#define CURRENT_SENSOR 34
#define VOLTAGE_SENSOR 35
#define SDA_PIN 21
#define SCL_PIN 22

#define GREEN_LED  25
#define YELLOW_LED 26
#define RED_LED    27
#define WHITE_LED  19
#define BUZZER     14
#define RELAY_PIN  18

#define DHTPIN     4
#define DHTTYPE    DHT11

#define OVERLOAD_POWER 60.0   // watts

DHT dht(DHTPIN, DHTTYPE);

/************ CONSTANTS ************/
const float emissionFactor = 0.82;   // kg CO2 / kWh
const float CURRENT_NOISE = 0.02;

float VOLTAGE_CAL = 230.0;
float CURRENT_CAL = 1.0;
float tariff = 6.5;

/************ VARIABLES ************/
float voltageRMS = 0;
float currentRMS = 0;
float powerW = 0;
float powerFactor = 1.0;   // assumed unity
float energykWh = 0;
float carbon = 0;
float temperatureC = 0;
float electricityCost = 0;

String carbonRating = "";

unsigned long lastEnergyTime = 0;
unsigned long lcdTimer = 0;
int lcdPage = 0;

/************ TRUE RMS FUNCTION ************/
float readRMS(uint8_t pin) {
  const int samples = 800;
  long sum = 0, sumSq = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(120);
  }

  int offset = sum / samples;

  for (int i = 0; i < samples; i++) {
    int v = analogRead(pin) - offset;
    sumSq += (long)v * v;
    delayMicroseconds(120);
  }

  float rmsADC = sqrt(sumSq / (float)samples);
  return (rmsADC * 3.3) / 4095.0;
}
/*************Relaycontrolblynk****************/
bool manualRelayControl = false;  // Phone control active?
bool relayState = true;           // Relay ON/OFF
/****************Smart Relay*******************/
 BLYNK_WRITE(V8) {
  relayState = param.asInt();   // 1 = ON, 0 = OFF

  if (relayState) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }
}



/************ SETUP ************/
void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(WHITE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH); // relay ON by default

  analogReadResolution(12);
  analogSetPinAttenuation(CURRENT_SENSOR, ADC_11db);
  analogSetPinAttenuation(VOLTAGE_SENSOR, ADC_11db);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Carbon Meter");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(2000);
  lcd.clear();

  dht.begin();
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  lastEnergyTime = millis();
}

/************ LOOP ************/
void loop() {
  Blynk.run();

  /************ SENSOR READ ************/
  float vRaw = readRMS(VOLTAGE_SENSOR);
  float iRaw = readRMS(CURRENT_SENSOR);

  voltageRMS = vRaw * VOLTAGE_CAL;
  currentRMS = iRaw * CURRENT_CAL;

  if (currentRMS < CURRENT_NOISE) {
    currentRMS = 0;
    powerW = 0;
  } else {
    powerW = voltageRMS * currentRMS;
  }

  /************ ENERGY ************/
  unsigned long now = millis();
  float hours = (now - lastEnergyTime) / 3600000.0;
  energykWh += (powerW / 1000.0) * hours;
  lastEnergyTime = now;

  /************ CARBON ************/
  carbon = energykWh * emissionFactor;

  if (carbon <= 0.003) carbonRating = "A (Excellent)";
  else if (carbon <= 0.007) carbonRating = "B (Good)";
  else if (carbon <= 0.014) carbonRating = "C (Moderate)";
  else if (carbon <= 0.023) carbonRating = "D (High)";
  else carbonRating = "E (Very High)";

  electricityCost = energykWh * tariff;

  /************ DHT ************/
  float t = dht.readTemperature();
  if (!isnan(t)) temperatureC = t;

  /************ BLYNK ************/
  Blynk.virtualWrite(V0, powerW);
  Blynk.virtualWrite(V1, voltageRMS);
  Blynk.virtualWrite(V2, currentRMS);
  Blynk.virtualWrite(V3, temperatureC);
  Blynk.virtualWrite(V4, energykWh);
  Blynk.virtualWrite(V5, electricityCost);
  Blynk.virtualWrite(V6, carbonRating);
  Blynk.virtualWrite(V7, carbon);

  /************ OVERLOAD + TEMP PROTECTION ************/
  static bool overloadSent = false;

if (powerW > OVERLOAD_POWER || temperatureC > 60) {
  digitalWrite(RELAY_PIN, LOW);
  relayState = 0;              // sync phone state
  digitalWrite(BUZZER, HIGH);
}


    if (!overloadSent) {
      Blynk.logEvent("overload_alert", "⚠️ Overload / Over temperature!");
      overloadSent = true;
    }
  else {
  digitalWrite(BUZZER, LOW);
  overloadSent = false;
}


  /************ LCD DISPLAY ************/
  if (millis() - lcdTimer > 2000) {
    lcdTimer = millis();
    lcdPage = (lcdPage + 1) % 6;
    lcd.clear();
  }

  if (lcdPage == 0) {
    lcd.setCursor(0,0);
    lcd.print("Volt:");
    lcd.print(voltageRMS,1);
    lcd.print("V");

    lcd.setCursor(0,1);
    lcd.print("Amp:");
    lcd.print(currentRMS,2);
    lcd.print("A");
  }
  else if (lcdPage == 1) {
    lcd.setCursor(0,0);
    lcd.print("Power:");
    lcd.print(powerW,1);
    lcd.print("W");

    lcd.setCursor(0,1);
    lcd.print("PF:1.00");
  }
  else if (lcdPage == 2) {
     lcd.setCursor(0,0);
    lcd.print("Energy:");
    lcd.print(energykWh,4);
    lcd.print("J");
    lcd.setCursor(0,1);
    lcd.print("Cost:");
    lcd.print(electricityCost,2);
    lcd.print("Rs");
  }
  else if (lcdPage == 3) {
    lcd.setCursor(0,0);
    lcd.print("Carbon:");
    lcd.setCursor(5,1);
    lcd.print(carbon,3);
    lcd.print("kg");
  }
  else if (lcdPage == 4) 
   {lcd.setCursor(0,0);
    lcd.print("Temperature:");
    lcd.setCursor(8,1);
    lcd.print(temperatureC,1);
    lcd.print("C");
  }
  else if (lcdPage == 5) {
    lcd.setCursor(0,0);
    lcd.print("Carbon Rating");
    lcd.setCursor(0,1);
    lcd.print(carbonRating);
  }

  /************ LED INDICATION ************/
  static bool blink;
  blink = !blink;

  digitalWrite(WHITE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);

  if (powerW < 2) digitalWrite(WHITE_LED, HIGH);
  else if (powerW <= 13) digitalWrite(GREEN_LED, blink);
  else if (powerW <= 25) digitalWrite(YELLOW_LED, blink);
  else digitalWrite(RED_LED, blink);

  delay(300);
}