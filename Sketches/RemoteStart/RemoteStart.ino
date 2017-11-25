#include "pins_arduino.h"
int POWER_PIN = A1;
int LOCK_BTN_PIN = A0;

#define PIN_ON HIGH
#define PIN_OFF LOW

//HIGH = Turn relay on
//LOW = Turn relay off
void setup() {
  pinMode(POWER_PIN, OUTPUT);
  pinMode(LOCK_BTN_PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Toyota GSM Remote Starter - by James Tyra");

  //set all pins to low
  digitalWrite(POWER_PIN,LOW);
  digitalWrite(LOCK_BTN_PIN, LOW);
  if (pin_status(LOCK_BTN_PIN) == HIGH) {
    Serial.println("LOCK_BTN_PIN already HIGH!");
  }
}

void loop() {
  Serial.println("Startng Car!");
  start_car();
  Serial.println("DOne!");
  delay(30000);
}


void start_car() {
  //TODO: Ensure Engine RPM == 0 before trying to start
  power_on_key_fob();
  //ok push set .5, .5 5.0
  push_key_fob_btn(LOCK_BTN_PIN,600);
  push_key_fob_btn(LOCK_BTN_PIN,600);
  push_key_fob_btn(LOCK_BTN_PIN,3000);
  
  power_off_key_fob();
}
void power_on_key_fob() {
  Serial.println("Power on KEY FOB");
  digitalWrite(POWER_PIN, HIGH);
  delay(2000);
}

void power_off_key_fob() {
  Serial.println("Power OFF Key FOB");
  digitalWrite(POWER_PIN, LOW);
  delay(2000);  
}

void push_key_fob_btn(int btn, int msDuration) {
  if (pin_status(btn) == HIGH) {
    Serial.println("Not pushing, pin already HIGH!");
    Serial.println(btn);
    return; 
  }
  Serial.println("START PUSH!");
  //set pin high
  digitalWrite(btn,HIGH);
  //wait 
  delay(msDuration);
  //set low
  digitalWrite(btn,LOW);
  //delay 1 sec
  delay(500);
  Serial.println("END PUSH!");
}


int pin_status(int pin) {
  int resl;
  resl = digitalRead(pin);
  return (pin == HIGH) ? HIGH : LOW;
  
}

