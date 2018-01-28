#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED



/**************************************
* GPRS/network settings
**************************************/
#define XBEE_BAUDRATE 115200
// change APN to your carrier's setting
#define APN "your.apn"
// change SERVER_SECRET to something else!
#define SECRET_KEY "secret_sauce"
#define SERVER_URL "your.server.com/path/to/python/app"
#define USE_GSM_LOCATION 1
// maximum consecutive errors before resetting
#define MAX_CONN_ERRORS 5
// maximum allowed connecting time
#define MAX_CONN_TIME 20000 /* ms */


#endif // CONFIG_H_INCLUDED
