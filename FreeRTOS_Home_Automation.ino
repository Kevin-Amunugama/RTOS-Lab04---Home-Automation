
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <FreeRTOS.h>


#define BLYNK_PRINT Serial

static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;

//Function Prototypes
void gassensor();
void DHT11sensor();
void pirsensor();
void ultrasonic();
void coreOpp01 ();
void coreOpp02 ();

//Initialize the LCD display
LiquidCrystal_I2C lcd(0x27, 16, 2);
static const int led_pin1 = LED_BUILTIN;

//WiFi Credentials
char auth[] = "IRCy0RzL1OPyFT4exUaTx3b-Xsy_Nlnw";
char ssid[] = "iPhone 13";
char pass[] = "asdfghjkl";

//Pin Connections
DHT dht(19, DHT11);
BlynkTimer timer;
bool pirbutton = 0;
#define Buzzer 23
#define MQ2 34
#define trig 18
#define echo 5
#define PIR 4
#define relay1 2
#define relay2 15



BLYNK_WRITE(V0) {
  pirbutton = param.asInt();
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(Buzzer, OUTPUT);
  pinMode(PIR, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);

  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  dht.begin();

  lcd.setCursor(0, 0);
  lcd.print("Home Automation");
  lcd.setCursor(4, 1);
  lcd.print("System");
  delay(4000);
  lcd.clear();

  xTaskCreatePinnedToCore(coreOpp01,
                          "Opp1",
                          5120,
                          NULL,
                          1,
                          &task_1,
                          app_cpu);


  xTaskCreatePinnedToCore(coreOpp02,
                          "Opp2",
                          5120,
                          NULL,
                          2,
                          &task_2,
                          pro_cpu);

  vTaskDelete(NULL);


}

void coreOpp01 (void *parameters){
  timer.setInterval(100L, gassensor);
  timer.setInterval(100L, DHT11sensor);
}

void coreOpp02 (void *parameters){
  timer.setInterval(100L, pirsensor);
  timer.setInterval(100L, ultrasonic);
}

//Get the MQ2 sensor values
void gassensor() {
  int value = analogRead(MQ2);
  Serial.println(value);
  value = map(value, 0, 1024, 0, 100);
  if (value <= 55) {
    digitalWrite(Buzzer, LOW);
  } else if (value > 55) {
    Blynk.notify("Warning! Gas leak detected");
    digitalWrite(Buzzer, HIGH);
  }
  Blynk.virtualWrite(V1, value);
  lcd.setCursor(0, 0);
  lcd.print("G:");
  lcd.print(" ");
  lcd.print(value);

}

//Get the DHT11 sensor values
void DHT11sensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Blynk.virtualWrite(V2, t);
  Blynk.virtualWrite(V3, h);

  lcd.setCursor(8, 0);
  lcd.print("T:");
  lcd.print(t);

  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(h);

}

//Get the PIR sensor values
void pirsensor() {
  bool value = digitalRead(PIR);
  if (pirbutton == 1) {
    if (value == 0) {
      digitalWrite(Buzzer, LOW);
    } else if (value == 1) {
      Blynk.notify("Warning! Please check your security system");
      digitalWrite(Buzzer, HIGH);
    }
  }
}

//Get the ultrasonic sensor values
void ultrasonic() {
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long t = pulseIn(echo, HIGH);
  long cm = t / 29 / 2;
  Blynk.virtualWrite(V4, cm);

  lcd.setCursor(8, 1);
  lcd.print("W:");
  lcd.print(cm);
  lcd.print("  ");
}

//Get buttons values
BLYNK_WRITE(V5) {
 bool RelayOne = param.asInt();
  if (RelayOne == 1) {
    digitalWrite(relay1, LOW);
  } else {
    digitalWrite(relay1, HIGH);
  }
}

//Get buttons values
BLYNK_WRITE(V6) {
 bool RelayTwo = param.asInt();
  if (RelayTwo == 1) {
    digitalWrite(relay2, LOW);
  } else {
    digitalWrite(relay2, HIGH);
  }
}

void loop() {
  Blynk.run();//Run the Blynk library
  timer.run();//Run the Blynk timer

}
