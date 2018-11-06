#include <Arduino.h>
#include "config.h"




class RSTART {
  public:
    void start_car() {
      //TODO: Ensure Engine RPM == 0 before trying to start
      power_on_key_fob();
      //ok push set .5, .5 5.0
      push_key_fob_btn(LOCK_BTN_PIN, 600);
      push_key_fob_btn(LOCK_BTN_PIN, 600);
      push_key_fob_btn(LOCK_BTN_PIN, 3000);

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
  private:
    void push_key_fob_btn(int btn, int msDuration) {
      if (pin_status(btn) == HIGH) {
        Serial.println("Not pushing, pin already HIGH!");
        Serial.println(btn);
        return;
      }
      Serial.println("START PUSH!");
      //set pin high
      digitalWrite(btn, HIGH);
      //wait
      delay(msDuration);
      //set low
      digitalWrite(btn, LOW);
      //delay 1 sec
      delay(500);
      Serial.println("END PUSH!");
    }


    int pin_status(int pin) {
      int resl;
      resl = digitalRead(pin);
      return (pin == HIGH) ? HIGH : LOW;

    }
};

