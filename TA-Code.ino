/**
   TRAUMA ALARMA MAIN CODE

   ENGR-152 Product design project
   Cole Weidkamp, Garrett Cooper, Matthew Martin, Marco Scoma

   Created 3/23/19 by Matthew

   "Code does not come from winning. Your struggles develop your code. When you go through crashes and decide not to surrender, that is programming."
*/

#include <Adafruit_FONA.h>
#include <TinyGPS.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

// FONA Pin definitions
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7


SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

TinyGPS gps;
SoftwareSerial gpsSS(4, 3);

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// Project variables, no concern for most people
int debugMode = 2;

//Timer variables
unsigned long fonaTimer = 0;

// FONA buffers for dealing with SMS stuff
char replybuffer[255];

/**************************************
   MAIN CODE BLOC. DOES THE MAIN STUFF
 **************************************/
void setup()
{
  // Begin serial communications
  Serial.begin(115200);
  //fonaSerial->begin(4800);
  gpsSS.begin(9600);
  
  /**
  // Print SIM card IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0)
  {
    debugLog("d", "Found SIM card IMEI.");
    Serial.println(imei);
  }
  else
  {
    debugLog("e", "SIM card error. No IMEI Found.");
  }
  **/
  debugLog('d', F("Setup completed."));
}

void loop()
{
  // Update variables and stuff BEFORE any periodic actions should happen

  // Look for data on the serial bus, and parse it
  if (Serial.available()) 
  {
    parseCommand(Serial.readString());
  }
  
  periodicActions(); // Things that should happen on every loop of the processor
  /**
  // Forces the program to wait 5 seconds after bootup to start doing stuff
  // Important because the MPU gives wacky values for a bit on first startup
  if(millis() > 5000) 
  {
    periodicActions(); // Things that should happen on every loop of the processor
  }
   */
}


// Actions scheduled to run at certain times, all the millis stuff goes here
void periodicActions()
{
  /**
  // Check FONA for new SMS every two seconds
  if (millis() - fonaTimer > 2000)
  {
    int8_t smsnum = fona.getNumSMS();
    if (smsnum > 0)
    {

      debugLog("d", "You've got mail!");
      parseAllSMS();
    }
    fonaTimer = millis();
  }
  **/
  // Constantly encode GPS data, so that we might get a full sentence when needed
  if (gpsSS.available())
  {
    char c = gpsSS.read();
    gps.encode(c);
  }
}


/********************************************************************
   SOVIET CODE BLOC. FUNCTION OWNED BY EVERYBODY, LIKE TRUE COMMUNISM.
 ********************************************************************/

// Grabs all SMS found onm the SIM card and passes them to the command parser
// Deletes them after parsing
void parseAllSMS()
{
  int8_t smsnum = fona.getNumSMS();
  uint16_t smslen;
  int8_t smsn;
  for (int i = 1; i < (smsnum + 1); i++)
  {
    debugLog('d', "SMS Number " + i);
    fona.readSMS(i, replybuffer, 250, &smslen);
    debugLog('d', replybuffer);
    parseCommand(String(replybuffer));
    fona.deleteSMS(i);
    debugLog('w', F("Message has been deleted."));
  }
}

// Structure to hold the GPS coordinates
// Makes it easy to pass around, like some sort of hot potato, or deformed kick ball
struct coordinates
{
  float lat;
  float lon;
};

// Reads GPS Data and returns the latitude and longitude
struct coordinates getGPS()
{
  struct coordinates coord;
  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age); // TODO: Remove age thing?
  //Serial.print("LAT=");
  //Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6); 
  //Serial.print(" LON=");
  //Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  coord.lat = flat;
  coord.lon = flon;
  // TODO: Remove print statements
  return coord;
}

// Convert a coordinates structure to a google maps link (string) for transmission
String toMapsLink(struct coordinates coord)
{
  String mapLink;
  String latString; latString = String(coord.lat, 6);
  String lonString; lonString = String(coord.lon, 6);
  mapLink = "http://www.google.com/maps/place/" + latString + "," + lonString;
  return mapLink;
}

// A helper function to manage logging to the serial console
// Has 3 log levels: Debug (2, highest), Warn (1, medium), and Error (0, lowest)
// Always prints errors, and the logging level can be increased to reveal more information
void debugLog(char level, String message)
{
  if (level == 'e')
  {
    Serial.print(F("[ERROR] "));
    Serial.println(message);
  }

  if (level == 'w' && debugMode >= 1)
  {
    Serial.print(F("[WARNING] "));
    Serial.println(message);
  }

  if (level == 'd' && debugMode >= 2)
  {
    Serial.print(F("[DEBUG] "));
    Serial.println(message);
  }
}

// A helper function to extract commands from abritrary inputs
// Used to get commands from SMS and serial; reusable!
void parseCommand(String command)
{
  debugLog('d', "Parsing command: " + command);
  if(command == "foo")
  {
    debugLog('d', F("Bar"));
  }
  else if(command == "location")
  {
    debugLog('d', toMapsLink(getGPS()));
  }
  else if(command == "dMode0")
  {
    debugLog('w', F("Setting debug mode to 0..."));
    debugMode = 0;
  }
  else if(command == "dMode1")
  {
    debugLog('w', F("Setting debug mode to 1..."));
    debugMode = 1;
  }
  else if(command == "dMode2")
  {
    debugLog('w', F("Setting debug mode to 2..."));
    debugMode = 2;
  }
  else
  {
    debugLog('e', "Command: " + command + " Not Recognized");
  }
}
