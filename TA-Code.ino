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
#include "MPU6050.h"
#include "Wire.h"
#include <SoftwareSerial.h>

/*************************
   Configuration variables
 *************************/
int gAlarmThreshold = 18; // Threshold for a High-G event
const short gpsCounterDefault = 2000; // Number of invalid GPS sentences to accept before we throw an error.
#define phoneNumber "+19492917419"

// FONA Pin definitions
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

// MPU Pin definitions (FEATHER)
#define MPU_SLC 3
#define MPU_SDA 2

// Status LED Pin definitions
#define BATT_STAT_PIN 10
#define CELL_STAT_PIN 11
#define GPS_STAT_PIN 12

// FONA Serial declarations
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// GPS Serial declarations
TinyGPS gps;
#define gpsSS Serial1

MPU6050 mpu;

// Project variables, no concern for most people
int debugMode = 2; // Level of logging that should be sent to console by default
bool validGPS = false; // FLAG. Used to see if the GPS is giving us real data
short gpsCounter = gpsCounterDefault; // Counts down with the number of invalid sentences encoded by the GPS. Throws an error when it hits zero.
int highGCounter = 0; // Tracks the number of High-G events detected in a row. When this exceeds a certain ammount, an alert is triggered
bool prevGAlarm = false; // Tells if the acceleration vector was previously in an alarm state
bool highGAlarm = false; // If a high-G condition is detected, and an alert should be sent

//Timer variables
unsigned long smsTimer = 0;
unsigned long battTimer = 0;
unsigned long gpsTimer = 0;
unsigned long cellServiceTimer = 0;
unsigned long mpuMicroTimer = 0;
unsigned long highGAlarmTimer = 0;

// FONA buffers for dealing with SMS stuff
char replybuffer[255];

// MPU Acceleration Variables
int16_t gx, gy, gz;

// Structure to hold the GPS coordinates
// Makes it easy to pass around, like some sort of digital hot potato, or deformed kick ball
struct coordinates
{
  float lat;
  float lon;
};


/**************************************
   MAIN CODE BLOC. DOES THE MAIN STUFF
 **************************************/
void setup()
{
  // Pin modes
  pinMode(BATT_STAT_PIN, OUTPUT);
  pinMode(CELL_STAT_PIN, OUTPUT);
  pinMode(GPS_STAT_PIN, OUTPUT);

  // Begin serial communications
  Serial.begin(9600);
  fonaSerial->begin(4800);
  fona.begin(*fonaSerial);
  gpsSS.begin(9600);

  // Join the wire bus for the MPU
  Wire.begin();

  // Print SIM card IMEI number.
  char imei[16] = {}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0)
  {
    debugLog('d', "Found SIM card IMEI.");
    Serial.println(imei);
  }
  else
  {
    debugLog('e', "SIM card error. No IMEI Found.");
  }

  /*
     MPU Setup
  */
  mpu.initialize();

  // Gyro offsets
  mpu.setXGyroOffset(-970);
  mpu.setYGyroOffset(-27);
  mpu.setZGyroOffset(206);
  //mpu.setZAccelOffset(1447);

  debugLog('d', F("Setup completed."));
}

void loop()
{
  // Look for data on the serial bus, and parse it
  if (Serial.available())
  {
    parseCommand(Serial.readString());
  }

  // Forces the program to wait 5 seconds after bootup to start doing stuff
  // Important because the MPU gives wacky values for a bit on first startup
  if (millis() > 0) // TODO: Change back to 5 seconds
  {
    periodicActions(); // Things that shouldn't happen on every loop of the processor

    // Constantly encode GPS data, so that we might get a full sentence when needed
    if (gpsSS.available())
    {
      char c = gpsSS.read();
      //Serial.println(c); // Uncomment to see GPS data flowing in
      if (gps.encode(c)) // Valid GPS sentence has been encoded
      {
        validGPS = true;
        gpsCounter = gpsCounterDefault; // Reset the GPS counter
      }
      else // Valid GPS sentence not encoded
      {
        gpsCounter--;
        if (gpsCounter <= 0)
        {
          validGPS = false;
          gpsCounter = 0; // Set it to zero, because why not?
          //debugLog('w', F("Did not receive good GPS data in awhile!"));
        }
      }
    }
  }

}

// Actions scheduled to run at certain times, all the millis stuff goes here
void periodicActions()
{
  
  // Check FONA for new SMS every half second
  if (millis() - smsTimer > 500)
  {
    int8_t smsnum = fona.getNumSMS();
    if (smsnum > 0)
    {
      debugLog('d', "You've got mail!");
      parseAllSMS();
    }
    
    smsTimer = millis();
  }

  // Check acceleration from MPU every 100 microseconds
  if (micros() - mpuMicroTimer > 50)
  {
    // Get updated acceleration from the MPU.
    mpu.getRotation(&gx, &gy, &gz); // (Yes I know it looks like gyro, but trust me it's acceleration)
    
      /*
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.print(gz); Serial.print("\t");
      Serial.print(vectorMag()); Serial.print("\t");
      Serial.print(millis()); Serial.println("\t");
      */

    // For every sample that rises above the alarm threshold, increment counter
    double magnitude = vectorMag();
    //Serial.println(magnitude);
    if (magnitude > gAlarmThreshold)
    {
      highGCounter++;
      prevGAlarm = true;
    }
    else // Potential falling edge of an acceleration event
    {
      // Check to see if the number of samples that went above threshold is within alarm limits
      if (prevGAlarm && highGCounter > 3 && highGCounter < 15)
      {
        Serial.println("=========IMPACT===========");
        highGAlarm = true; // Set flag to send an alarm notification
      }
      highGCounter = 0; // Reset counter
    }
    
    mpuMicroTimer = micros();
  }

  // Check if a High-G alert should be sent every half second
  // This eliminates potential duplicate notifications from one impact event
  // 'Aftershocks' are common, where two peaks will happen in one event, and trigger multiple notifications
  if (millis() - highGAlarmTimer > 500)
  {
    if (highGAlarm)
    {
      Serial.println("=======ALARM=========");
      digitalWrite(BATT_STAT_PIN, HIGH); // Turns on the battery status light, warning of an impact
      char impactMessage[] = "Impact detected!";
      fona.sendSMS(phoneNumber, impactMessage);

      // Get the maps link and convert to char array
      String mapsString = toMapsLink(getGPS());
      char mapsLink[56];
      mapsString.toCharArray(mapsLink, 56);

      fona.sendSMS(phoneNumber, mapsLink); // Send a maps link with current location
      highGAlarm = false;
    }
    
    highGAlarmTimer = millis();
  }

  // Check battery status every five seconds
  if (millis() - battTimer > 5000)
  {
    uint16_t vbat;
    // From Adafruit FONA example code
    if (!fona.getBattPercent(&vbat))
    {
      debugLog('w', F("Failed to read battery voltage"));
      digitalWrite(BATT_STAT_PIN, HIGH);
    }
    else
    {
      if (vbat < 20)
      {
        digitalWrite(BATT_STAT_PIN, HIGH);
        debugLog('w', F("Battery less than 20%!")); // TODO: Remove console spam
      }
      else
      {
        digitalWrite(BATT_STAT_PIN, LOW);
        debugLog('d', "Battery OK");
      }
    }
    
    battTimer = millis();
  }

  // Check network status every second
  if (millis() - cellServiceTimer > 1000)
  {

    /*
       0 : Not registered
       1 : Registered (home)
       2 : Not registered (searching)
       3 : Denied
       4 : Unknown
       5 : Registered roaming
    */

    // From Adafruit FONA example code
    uint8_t n = fona.getNetworkStatus();
    uint8_t s = fona.getRSSI();
    if (n == 1 || n == 5)
    {
      //digitalWrite(CELL_STAT_PIN, HIGH);
      analogWrite(CELL_STAT_PIN, map(s, 2, 31, 0, 255)); // Maps cell signal level to brightness on LED
      debugLog('d', F("Cell service OK"));
      Serial.println(s);
    }
    else
    {
      digitalWrite(CELL_STAT_PIN, LOW);
      debugLog('w', F("No cell service."));
    }
    cellServiceTimer = millis();
  }

  // Check GPS status every two seconds
  if (millis() - gpsTimer > 2000)
  {
    if (validGPS)
    {
      debugLog('d', F("GPS OK"));
      digitalWrite(GPS_STAT_PIN, HIGH);
    }
    else
    {
      digitalWrite(GPS_STAT_PIN, LOW);
      debugLog('w', F("No valid GPS data"));
    }
    
    gpsTimer = millis();
  }

} // END PERIODIC ACTIONS

/********************************************************************
   SOVIET CODE BLOC. FUNCTION OWNED BY EVERYBODY, LIKE TRUE COMMUNISM.
 ********************************************************************/

// Grabs all SMS found on the SIM card and passes them to the command parser
// Deletes them after parsing
void parseAllSMS()
{
  int8_t smsnum = fona.getNumSMS(); // Gets number of SMS's stored on the SIM card
  uint16_t smslen;
  int8_t smsn;
  for (int i = 1; i < (smsnum + 1); i++)
  {
    debugLog('d', "SMS Number " + i);
    fona.readSMS(i, replybuffer, 250, &smslen);
    debugLog('d', replybuffer);
    parseCommand(String(replybuffer)); // Try to parse the received SMS as a command
    fona.deleteSMS(i);
    debugLog('w', F("Message has been deleted."));
  }
}

// Reads GPS Data and returns the latitude and longitude
struct coordinates getGPS()
{
  struct coordinates coord;
  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age); // TODO: Remove age thing?
  coord.lat = flat;
  coord.lon = flon;
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

// A utility function to calculate the magnitude of a vector
// Takes the gx,gy,gz variables, converts to g's from milli g's
// Calculates resultant vector and returns it, yo.
double vectorMag()
{
  double sqx = (gx / 2048) * (gx / 2048);
  double sqy = (gy / 2048) * (gy / 2048);
  double sqz = (gz / 2048) * (gz / 2048);
  double asqx = abs(sqx);
  double asqy = abs(sqy);
  double asqz = abs(sqz);
  double magnitude = sqrt(asqx + asqy + asqz); // TODO: Is this going to cause memory problems??
  return magnitude;
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
  if (command == "foo")
  {
    char barArray[] = "bar";
    fona.sendSMS(phoneNumber, barArray);
    debugLog('d', F("Bar"));
  }
  else if (command == "location")
  {
    // Get the maps link and convert to char array
    String mapsString = toMapsLink(getGPS());
    char mapsLink[56];
    mapsString.toCharArray(mapsLink, 56);
    debugLog('d', mapsLink);
    fona.sendSMS(phoneNumber, mapsLink); // Send a maps link with current location
  }
  else if (command == "dMode0")
  {
    debugLog('w', F("Setting debug mode to 0..."));
    debugMode = 0;
  }
  else if (command == "dMode1")
  {
    debugLog('w', F("Setting debug mode to 1..."));
    debugMode = 1;
  }
  else if (command == "dMode2")
  {
    debugLog('w', F("Setting debug mode to 2..."));
    debugMode = 2;
  }
  else
  {
    debugLog('e', "Command: " + command + " Not Recognized");
  }
}
// My life is a lie.
