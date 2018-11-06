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
#include "config.h"

//Yes I know this isn't standard, but it logically separates things
//and makes it more readable.
#include "Rstart.cpp"
#include "Sim5360.cpp"



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
      while(sim.xbTogglePower() != false) {
        delay(3000);
        Serial.println("togglePower");
      }
      delay(3000);
      sim.xbTogglePower();
      delay(3000);
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
      delay(5000);
      //(OPTIONAL) Step 4 - get various network info
      //if (sim.getOperatorName()) {
      //  Serial.print("Operator:");
      //  Serial.println(sim.buffer);
      //}

      //Serial.print("Obtaining IP address...");
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
      //Serial.print("Step 5 - Init HTTP...");
      if (sim.httpOpen()) {
        Serial.println("OK");
      } else {
        Serial.println("NO");
      }
      //Step 6 - connect to HTTP server
      //Serial.println("Step 6 - connecting to HTTP server.");
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
        //Serial.println("-----HTTP RESPONSE-----");
        
        Serial.println(payload);
        //Serial.println("-----------------------");
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
