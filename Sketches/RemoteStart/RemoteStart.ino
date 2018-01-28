/******************************************************************************
  Distributed under BSD license
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
******************************************************************************/


#include <Arduino.h>
#include <FreematicsONE.h>
#include "config.h"


// device states
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_SLEEPING 0x20
#define STATE_CONNECTED 0x40
#define STATE_ALL_GOOD 0x40

static const byte pids[] = {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_THROTTLE};
static int pidData[sizeof(pids)] = {0};
static char vin[20] = {0};
static uint16_t connCount = 0;
bool modemPower = false;

int POWER_PIN = A1;
int LOCK_BTN_PIN = A0;

#define PIN_ON HIGH
#define PIN_OFF LOW

//HIGH = Turn relay on
//LOW = Turn relay off

typedef enum {
  GPRS_DISABLED = 0,
  GPRS_READY,
  GPRS_HAVE_DATA,
  GPRS_HTTP_CONNECTING,
  GPRS_HTTP_CONNECTED,
  GPRS_HTTP_RECEIVING,
  GPRS_HTTP_ERROR,
} GPRS_STATES;

typedef enum {
  HTTP_GET = 0,
  HTTP_POST,
} HTTP_METHOD;

typedef struct {
  float lat;
  float lon;
  uint8_t year; /* year past 2000, e.g. 15 for 2015 */
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} GSM_LOCATION;

class COBDGSM : public COBDSPI {
  public:
    COBDGSM(): gprsState(GPRS_DISABLED), connErrors(0) {
      buffer[0] = 0;
    }
    void toggleGSM()
    {
      setTarget(TARGET_OBD);
      sendCommand("ATGSMPWR\r", buffer, sizeof(buffer));
    }
    bool netInit()
    {
      for (byte n = 0; n < 10; n++) {
        // try turning on module
        
        xbTogglePower();
        sleep(3000);
        // discard any stale data
        xbPurge();
        for (byte m = 0; m < 3; m++) {
          if (netSendCommand("AT+CGMM\r",3000,"SIMCOM_SIM5360A") == true) {
            modemPower = true;
            return true;
          }
          sleep(1000);
        }
      }
      return false;
    }
    bool netSetup(const char* apn, bool only3G = false)
    {
      uint32_t t = millis();
      bool success = false;
      netSendCommand("ATE0\r");
      netSendCommand("AT+CNAOP=2\r");
      if (only3G) netSendCommand("AT+CNMP=14\r"); // use WCDMA only
      do {
        do {
          Serial.print('.');
          sleep(2000);
          success = netSendCommand("AT+CPSI?\r", 1000, "Online");
          if (success) {
            if (!strstr_P(buffer, PSTR("NO SERVICE")))
              break;
            success = false;
          } else {
            if (strstr_P(buffer, PSTR("Off"))) break;
          }
        } while (millis() - t < 60000);
        if (!success) break;

        t = millis();
        do {
          success = netSendCommand("AT+CREG?\r", 2000, "+CREG: 0,1");
          sleep(2000);
        } while (!success && millis() - t < 60000);
        if (!success) break;

        do {
          success = netSendCommand("AT+CGREG?\r",2000, "+CGREG: 0,1");
          sleep(2000);
        } while (!success && millis() - t < 60000);
        if (!success) break;

        do {
          sprintf_P(buffer, PSTR("AT+CGSOCKCONT=1,\"IP\",\"%s\"\r"), apn);
          success = netSendCommand(buffer, 9000);
          sleep(2000);
        } while (!success && millis() - t < 60000);
        if (!success) break;

        success = netSendCommand("AT+CSOCKSETPN=1\r");
        sleep(2000);
      } while(0);
      if (!success) {
        Serial.println("ERROR in netsetup()!");
        Serial.println(buffer);
      }
      gprsState = GPRS_HTTP_CONNECTING;
      return success;
    }
    const char* getIP()
    {
      uint32_t t = millis();
      char *ip = 0;
      do {
        if (netSendCommand("AT+IPADDR\r", 5000, "\r\nOK\r\n", true)) {
          char *p = strstr(buffer, "+IPADDR:");
          if (p) {
            ip = p + 9;
            if (*ip != '0') {
              break;
            }
          }
        }
        sleep(500);
        ip = 0;
      } while (millis() - t < 15000);
      return ip;
    }
    int getSignal()
    {
        if (netSendCommand("AT+CSQ\r", 500)) {
            char *p = strchr(buffer, ':');
            if (p) {
              p += 2;
              int db = atoi(p);
              return (db*2)-113;
            }
        }
        return -1;
    }
    char* getOperatorName()
    {
        // display operator name
        if (netSendCommand("AT+COPS?\r") == 1) {
            char *p = strstr(buffer, ",\"");
            if (p) {
                p += 2;
                char *s = strchr(p, '\"');
                if (s) *s = 0;
                return p;
            }
        }
        return 0;
    }
    void httpUninit()
    {
      if (netSendCommand("AT+CHTTPSCLSE\r",3000, "OK") == false) {
        gprsState = GPRS_HTTP_ERROR;
        return false;
      }
      if (netSendCommand("AT+CHTTPSSTOP\r",3000, "OK") == false) {
        gprsState = GPRS_HTTP_ERROR;
        return false;    
      }
      gprsState = GPRS_READY;
      return true;
    }

    bool httpInit()
    {
      if (netSendCommand("AT+CHTTPSSTART\r",20000, "OK") == false) {
        return false;
      }
      gprsState = GPRS_READY;
      return true;
    }
    void httpConnect()
    {
      // 0 for GET, 1 for POST
      char cmd[17];
      //sprintf_P(buffer, PSTR("AT+CHTTPSOPSE=\"%s\",%s,1\r"),SERVER_HOST,SERVER_PORT);
      sprintf_P(buffer, PSTR("AT+CHTTPACT=\"%s\",%d\r"),SERVER_HOST,SERVER_PORT);
      
      //if(netSendCommand(buffer,40000,"OK") == false) {
      //  return false;
     //}
      xbWrite(buffer);
      gprsState = GPRS_HTTP_CONNECTED;
      return true;
    }
    bool httpRead()
    {
      //if (sendGSMCommand("AT+HTTPREAD\r", MAX_CONN_TIME) && strstr(buffer, "+HTTPREAD:")) {
      //  gprsState = GPRS_HAVE_DATA;
      //  return true;
      //} else {
      //  Serial.println("READ ERROR");
      //  Serial.println(buffer);
      //  gprsState = GPRS_HTTP_ERROR;
      //  return false;
      //}
    }
    bool getLocation(GSM_LOCATION* loc)
    {
      /*if (sendGSMCommand("AT+CIPGSMLOC=1,1\r", 1000)) do {
          char *p;
          if (!(p = strchr(buffer, ':'))) break;
          if (!(p = strchr(p, ','))) break;
          loc->lon = atof(++p);
          if (!(p = strchr(p, ','))) break;
          loc->lat = atof(++p);
          if (!(p = strchr(p, ','))) break;
          loc->year = atoi(++p) - 2000;
          if (!(p = strchr(p, '/'))) break;
          loc->month = atoi(++p);
          if (!(p = strchr(p, '/'))) break;
          loc->day = atoi(++p);
          if (!(p = strchr(p, ','))) break;
          loc->hour = atoi(++p);
          if (!(p = strchr(p, ':'))) break;
          loc->minute = atoi(++p);
          if (!(p = strchr(p, ':'))) break;
          loc->second = atoi(++p);
          return true;
        } while (0);*/
      return false;
    }
    byte checkbuffer(const char* expected, unsigned int timeout = 2000)
    {
      byte ret = xbReceive(buffer, sizeof(buffer), 0, &expected, 1) != 0;
      if (ret == 0) {
        // timeout
        return (millis() - checkTimer < timeout) ? 0 : 2;
      } else {
        return ret;
      }
    }
    bool netSendCommand(const char* cmd, unsigned int timeout = 5000, const char* expected = "OK", bool terminated = false)
    {
      if (cmd) {
        xbWrite(cmd);
      }
      sleep(50);
      buffer[0] = 0;
      byte ret = xbReceive(buffer, sizeof(buffer), timeout, &expected, 1);
      ret = strstr(buffer,expected);
      Serial.println(ret);
      Serial.println(buffer);
      if (ret > 0) {
        Serial.println("STRSTR TRUE!");
        return true;
      }
      else {
        return false;
      }
      if (ret) {
        if (terminated) {
          char *p = strstr(buffer, expected);
          if (p) *p = 0;
        }
        return true;
      } else {
        return false;
      }
    }
    char buffer[192];
    byte bytesRecv;
    uint32_t checkTimer;
    byte gprsState;
    byte connErrors;
};



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

class CGPRSRemoteStarter : public COBDGSM
{
  public:
    void setup()
    {
      state = 0;
      pinMode(POWER_PIN, OUTPUT);
      pinMode(LOCK_BTN_PIN, OUTPUT);
      delay(500);
      digitalWrite(POWER_PIN, LOW);
      digitalWrite(LOCK_BTN_PIN, LOW);
      Serial.begin(115200);

      // this will init SPI communication
      begin();
      Serial.println("Toyota GSM Remote Starter - by James Tyra");
#ifdef DO_OBD
      // connect to OBD port
      Serial.print("#OBD..");
      for (;;) {
        Serial.print('.');
        if (init()) {
          state |= STATE_OBD_READY;
          break;
        }
        sleepSec(10);
      } //do this until initialized

      // display OBD adapter version
      if (state & STATE_OBD_READY) {
        Serial.println("OK");
      } else {
        Serial.println("NO");
      }

      // retrieve VIN
      if (getVIN(buffer, sizeof(buffer))) {
        snprintf_P(vin, sizeof(vin), PSTR("%s"), buffer);
        Serial.print("#VIN:");
        Serial.println(vin);
      }

#endif

    }


    void loop()
    {

      //Setup things
#ifdef DO_OBD
      // connect to OBD port
      Serial.print("#OBD..");
      for (uint32_t t = millis(); millis() - t < OBD_CONN_TIMEOUT; ) {
        Serial.print('.');
        if (init()) {
          state |= STATE_OBD_READY;
          break;
        }
      }

#endif

      Serial.print("#GSM...");
      if (netInit()) {
        Serial.println("OK");
      } else {
        Serial.println(buffer);
      }

      Serial.print("#GPRS(APN:");
      Serial.print(APN);
      Serial.print(")...");
      if (netSetup(APN)) {
        Serial.println("OK");
      } else {
        Serial.println(buffer);
      }

      int csq = getSignal();
      if (csq > 0) {
        Serial.print("#SIGNAL:");
        Serial.println(csq);
        gprsState = GPRS_READY;
      }

      // do HTTP request


      if (gprsState != GPRS_READY) {
        Serial.println("GPRS Not Ready!");
      } 
      
      Serial.println("Get REMOTE CMD");
      //Ok find out if we have a command to process
      //we have 90 seconds to do this, then go back to sleep
      //once gprs_state == GPRS_HAVE_DATA, we have the command
      for (uint32_t t = millis(); millis() - t < GSM_GET_CMD_TIMEOUT; ) {
        Serial.print('*');
        sleep(getRemoteCommand());
        if (gprsState == GPRS_HAVE_DATA) {
          //do something
          Serial.println("CMD:");
          Serial.println(buffer);
          gprsState = GPRS_READY;
        }
      }

      //TODO: Enter Lowest power state Possible
      //TODO: SLEEP
      toggleGSM(); // turn off GSM power
      sleep(500);
      state &= ~(STATE_OBD_READY | STATE_GPS_READY | STATE_MEMS_READY);
      state |= STATE_SLEEPING;
      Serial.println("Sleeping");
      enterLowPowerMode();
      sleepSec(90);
      leaveLowPowerMode();

    }
  private:
    void generateCarStartedURL()
    {
      // URL format: http://server-url.com/<SECRET_KEY>/STARTED/<RPM>/<SPEED>/<ENGINE_LOAD>/<COOLANT_TEMP>/<INTAKE_PRESSURE>/<THROTTLE_POSITION>/<FUEL_RATE>/<GPS_LAT>/<GPS_LNG>
      //get location via GSM
      GSM_LOCATION loc;
      getLocation(&loc);
      // generate URL

      //snprintf_P(buffer, sizeof(buffer), PSTR("AT+HTTPPARA=\"URL\",\"%s/%s/STARTED/%u/%u/%u/%d/0/%u/0/%ld/%ld\"\r"),
      //           SERVER_URL, SECRET_KEY, pidData[0], pidData[1], pidData[2], pidData[3], pidData[4], loc.lat, loc.lon);
      //Serial.println(buffer);
      Serial.print(loc.lat);
      Serial.print(' ');
      Serial.println(loc.lon);
    }
    void generateGetCmdPath()
    {
      // format: GET /<SERVER_PATH>/GETCMD/<SECRET_KEY>/ HTTP/1.1\r\nHost: <SERVER_HOST>\r\n\r\n\x1A
      // generate URL
      sprintf_P(buffer, PSTR("GET %s/GETCMD/%s/ HTTP/1.1\r\nHost: %s Content-Length: 0\r\n\r\n\x1A"),
                 SERVER_PATH, SECRET_KEY,SERVER_HOST);
      //snprintf_P(buffer, sizeof(buffer), PSTR("AT+CHTTPACT=\"moxone.me\",%s\r"),
      //           SERVER_PORT);
      Serial.println(buffer);
    }
 

    //returns amount of time to sleep
    int getRemoteCommand()
    {
      //generateGetCmdURL();
      //httpInit();
      sleep(5000);
      httpConnect();
      sleep(5000);
      //ok send data
      char sendz[30];
      //generateGetCmdPath();
      //snprintf_P(sendz, 30, PSTR("AT+CHTTPSSEND=%d\r"),strlen(buffer));
      //Serial.println(sendz);
      //must perform raw read/write here
      //xbWrite(sendz);
      //sleep(5000);
      //xbPurge();
      //char resl[1];
      //xbRead(resl,1,30000);
      //Serial.println(resl);
      //if(resl != '>') {
      //  Serial.println("Wrong Data Received!");
      //}
      //sleep(1500);
      generateGetCmdPath();
      netSendCommand(buffer, 45000,"OK");
      sleep(2000); 
      return 0;
    }


    byte state;
    RSTART starter;
};


CGPRSRemoteStarter rstart;

void setup()
{
  rstart.setup();
}

void loop()
{
  rstart.loop();
}



