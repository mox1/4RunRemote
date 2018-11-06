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

//Yes I know this isn't standard, but it logically separates things
//and makes it more readable.
//#include "Rstart.cpp"
//#include "Sim5360.cpp"



static const byte pids[] = {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_THROTTLE};
static int pidData[sizeof(pids)] = {0};
static char vin[20] = {0};
static uint16_t connCount = 0;
bool modemPower = false;




//HIGH = Turn relay on
//LOW = Turn relay off


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


class CSIM5360 : public COBDSPI {
  public:
    CSIM5360() { buffer[0] = 0; }
    bool netInit()
    {
      for (byte n = 0; n < 3; n++) {
        // try turning on module
        xbTogglePower();
        sleep(3000);
        // discard any stale data
        xbPurge();
        for (byte m = 0; m < 3; m++) {
          if (netSendCommand("AT\r"))
            return true;
        }
      }
      return false;
    }
    bool netSetup(const char* apn, bool only3G = false)
    {
      uint32_t t = millis();
      bool success = false;
      netSendCommand("ATE0\r");
      if (only3G) netSendCommand("AT+CNMP=14\r"); // use WCDMA only
      do {
        do {
          Serial.print('.');
          delay(3000);
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
          success = netSendCommand("AT+CREG?\r", 5000, "+CREG: 0,1");
        } while (!success && millis() - t < 30000);
        if (!success) break;

        do {
          success = netSendCommand("AT+CGREG?\r",1000, "+CGREG: 0,1");
        } while (!success && millis() - t < 30000);
        if (!success) break;

        do {
          sprintf_P(buffer, PSTR("AT+CGSOCKCONT=1,\"IP\",\"%s\"\r"), apn);
          success = netSendCommand(buffer);
        } while (!success && millis() - t < 30000);
        if (!success) break;

        success = netSendCommand("AT+CSOCKSETPN=1\r");
        if (!success) break;

        success = netSendCommand("AT+CIPMODE=0\r");
        if (!success) break;

        netSendCommand("AT+NETOPEN\r");
        delay(5000);
      } while(0);
      if (!success) Serial.println(buffer);
      return success;
    }
    const char* getIP()
    {
      uint32_t t = millis();
      char *ip = 0;
      do {
        if (netSendCommand("AT+IPADDR\r", 5000, "+IPADDR:")) {
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
      } while (millis() - t < 60000);
      return ip;
    }
    int getSignal()
    {
        if (netSendCommand("AT+CSQ\r", 500)) {
            char *p = strchr(buffer, ':');
            if (p) {
              p += 2;
              int db = atoi(p) * 10;
              p = strchr(p, '.');
              if (p) db += *(p + 1) - '0';
              return db;
            }
        }
        return -1;
    }
    bool getOperatorName()
    {
        // display operator name
        if (netSendCommand("AT+COPS?\r") == 1) {
            char *p = strstr(buffer, ",\"");
            if (p) {
                p += 2;
                char *s = strchr(p, '\"');
                if (s) *s = 0;
                strcpy(buffer, p);
                return true;
            }
        }
        return false;
    }
    bool httpOpen()
    {
        return netSendCommand("AT+CHTTPSSTART\r", 3000);
    }
    void httpClose()
    {
      netSendCommand("AT+CHTTPSCLSE\r");
    }
    bool httpConnect(char *host, unsigned int port)
    {
        sprintf_P(buffer, PSTR("AT+CHTTPSOPSE=\"%s\",%u,1\r"), host, port);
        //Serial.println(buffer);
        return netSendCommand(buffer, MAX_CONN_TIME);
    }
    unsigned int genHttpHeader(HTTP_METHOD method, char *host, const char* path, bool keepAlive, const char* payload, int payloadSize)
    {
        // generate HTTP header
        char *p = buffer;
        p += sprintf_P(p, PSTR("%s %s HTTP/1.1\r\nUser-Agent: ONE\r\nHost: %s\r\nConnection: %s\r\n"),
          method == HTTP_GET ? "GET" : "POST", path, host, keepAlive ? "keep-alive" : "close");
        if (method == HTTP_POST) {
          p += sprintf_P(p, PSTR("Content-length: %u\r\n"), payloadSize);
        }
        p += sprintf_P(p, PSTR("\r\n\r"));
        return (unsigned int)(p - buffer);
    }
    bool httpSend(HTTP_METHOD method, char *host, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0)
    {
      unsigned int headerSize = genHttpHeader(method, host, path, keepAlive, payload, payloadSize);
      // issue HTTP send command
      sprintf_P(buffer, PSTR("AT+CHTTPSSEND=%u\r"), headerSize + payloadSize);
      if (!netSendCommand(buffer, 100, ">")) {
        Serial.println(buffer);
        Serial.println("Connection closed");
      }
      // send HTTP header
      genHttpHeader(method, host, path, keepAlive, payload, payloadSize);
      xbWrite(buffer);
      // send POST payload if any
      if (payload) xbWrite(payload);
      buffer[0] = 0;
      if (netSendCommand("AT+CHTTPSSEND\r")) {
        checkTimer = millis();
        return true;
      } else {
        Serial.println(buffer);
        return false;
      }
    }
    int httpReceive(char** payload)
    {
        int received = 0;
        // wait for RECV EVENT
        checkbuffer("RECV EVENT", MAX_CONN_TIME);
        /*
          +CHTTPSRECV:XX\r\n
          [XX bytes from server]\r\n
          \r\n+CHTTPSRECV: 0\r\n
        */
        if (netSendCommand("AT+CHTTPSRECV=384\r", MAX_CONN_TIME, "+CHTTPSRECV: 0", true)) {
          char *p = strstr(buffer, "+CHTTPSRECV:");
          if (p) {
            p = strchr(p, ',');
            if (p) {
              received = atoi(p + 1);
              if (payload) {
                char *q = strchr(p, '\n');
                *payload = q ? (q + 1) : p;
              }
            }
          }
        }
        return received;
    }
    byte checkbuffer(const char* expected, unsigned int timeout = 2000)
    {
      // check if expected string is in reception buffer
      if (strstr(buffer, expected)) {
        return 1;
      }
      // if not, receive a chunk of data from xBee module and look for expected string
      byte ret = xbReceive(buffer, sizeof(buffer), timeout, &expected, 1) != 0;
      if (ret == 0) {
        // timeout
        return (millis() - checkTimer < timeout) ? 0 : 2;
      } else {
        return ret;
      }
    }
    bool netSendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "\r\nOK", bool terminated = false)
    {
      if (cmd) {
        xbWrite(cmd);
      }
      buffer[0] = 0;
      byte ret = xbReceive(buffer, sizeof(buffer), timeout, &expected, 1);
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
    char buffer[384];
private:
    uint32_t checkTimer;
};
CSIM5360 sim;
byte netState = NET_DISCONNECTED;
byte errors = 0;


RSTART starter;


class CGPRSRemoteStarter {
  public:
    void doSetup() {
      //key fob pin setup
      pinMode(POWER_PIN, OUTPUT);
      pinMode(LOCK_BTN_PIN, OUTPUT);
      delay(500);
      digitalWrite(POWER_PIN, LOW);
      digitalWrite(LOCK_BTN_PIN, LOW);
      Serial.begin(115200);
      delay(500);
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
        sim.sleepSec(10);
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

    } //end setup
    void doLoop() {

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

      Serial.println("Get REMOTE CMD");
      int cmd = getRemoteCommand();
      //cmd < 1 == error, don't process it
      if (cmd < 1) {
        goLowPower();
        return;
      }
      //Process command
      switch (cmd) {
        case 1:
          Serial.println("Success, but nothing to do!");
          break;
        case 2:
          starter.start_car();
          break;
        default:
          break;
      }
      //sleep
      goLowPower();
    }//end loop
  private:

    void goLowPower(int seconds = 180) {
      //TODO: Enter Lowest power state Possible
      //TODO: SLEEP
      sim.xbTogglePower();
      delay(2000);
      // discard any stale data
      sim.xbPurge();

      delay(500);
      Serial.println("Entering Low Power Mode");
      sim.enterLowPowerMode();
      sim.deepSleepSec(seconds);
      sim.leaveLowPowerMode();
    }

    //returns amount of time to sleep
    int getRemoteCommand(int sendcmd = 0) {
      //this will setup the xbee 5360 module from scratch
      //then send HTTP GET request
      //then power xbee module back down
      // Step #1 this will init SPI communication
      sim.begin();
      sim.xbBegin(XBEE_BAUDRATE);
      // Step #2 initialize SIM5360 xBee module (if present)
      for (;;) {
        Serial.print("Init SIM5360...");
        if (sim.netInit()) {
          Serial.println("OK");
          break;
        } else {
          Serial.println("Failed Step 2 (init 5360)");
          return -2;
        }
      }
      //Step 3 - setup sim network
      Serial.print("Connecting network");
      if (sim.netSetup(APN, false)) {
        Serial.println("OK");
      } else {
        Serial.println("Failed Step 3 (init cell network)");
        return -3;
      }
      //(OPTIONAL) Step 4 - get various network info
      if (sim.getOperatorName()) {
        Serial.print("Operator:");
        Serial.println(sim.buffer);
      }

      Serial.print("Obtaining IP address...");
      const char *ip = sim.getIP();
      if (ip) {
        Serial.print(ip);
      } else {
        Serial.println("failed");
      }

      int signal = sim.getSignal();
      if (signal > 0) {
        Serial.print("CSQ:");
        Serial.print((float)signal / 10, 1);
        Serial.println("dB");
      }
      //Step 5 - Do HTTP connection
      Serial.print("Step 5 - Init HTTP...");
      if (sim.httpOpen()) {
        Serial.println("OK");
      } else {
        Serial.println("NO");
      }
      //Step 6 - connect to HTTP server
      Serial.println("Step 6 - connecting to HTTP server.");
      sim.xbPurge();
      if (!sim.httpConnect(HTTP_SERVER_HOST, HTTP_SERVER_PORT)) {
        Serial.println("Error connecting ");
        Serial.println(sim.buffer);
        return -6;
        return;
      }
      //Step 7 - Send HTTP request
      Serial.print("Sending HTTP request...");
      if (!sim.httpSend(HTTP_GET, HTTP_SERVER_HOST, SERVER_PATH, true)) {
        Serial.println("Failed Step 7");
        sim.httpClose();
        return -7;
      } else {
        Serial.println("OK");
      }
      //STep 8 - Read HTTP response
      Serial.print("Receiving...");
      char *payload;
      if (sim.httpReceive(&payload)) {
        Serial.println("OK");
        Serial.println("-----HTTP RESPONSE-----");
        
        Serial.println(payload);
        Serial.println("-----------------------");
        netState = NET_CONNECTED;
        errors = 0;
      } else {
        Serial.println("failed");
        errors++;
      }
      return 0;
    }



};
CGPRSRemoteStarter rstart;

void setup()
{
  rstart.doSetup();
}

void loop()
{
  rstart.doLoop();
}
