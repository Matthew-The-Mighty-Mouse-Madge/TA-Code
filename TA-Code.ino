/**
   TRAUMA ALARMA MAIN CODE

   ENGR-152 Product design project
   Cole Weidkamp, Garrett Cooper, Matthew Martin, Marco Scoma

   Created 3/23/19 by Matthew

   "Code does not come from winning. Your struggles develop your code. When you go through crashes and decide not to surrender, that is programming."
*/

#include <Adafruit_FONA.h>
//#include <TinyGPS.h>
//#include <I2Cdev.h>
//#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SoftwareSerial.h>


// FONA Pin definitions
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

// MPU Pin definitions (FEATHER)
#define INTERRUPT_PIN 2
#define MPU_SLC 10
#define MPU_SDA 11

// GPS Pin definitions (FEATHER)
#define GPS_TX 6
#define GPS_RX A5

// Status LED Pin definitions
#define BATT_STAT_PIN 5
#define CELL_STAT_PIN 13
#define GPS_STAT_PIN 12

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

/*
  TinyGPS gps;
  SoftwareSerial gpsSS(4, 3); // Standard UNO config
  //SoftwareSerial gpsSS(GPS_RX, GPS_TX); // Use on feather


  MPU6050 mpu;
*/
// Project variables, no concern for most people
int debugMode = 2;
bool doCheckAccel = false; // FLAG. Tells the program if it should be checking the acceleration


//Timer variables
unsigned long smsTimer = 0;
unsigned long batteryTimer = 0;
unsigned long gpsTimer = 0;
unsigned long cellServiceTimer = 0;

// FONA buffers for dealing with SMS stuff
char replybuffer[255];
/*
  // MPU control/status vars
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  // MPU orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat aa;         // [x, y, z]            accel sensor measurements
  VectorFloat aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  //VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector

  // ================================================================
  // ===               INTERRUPT DETECTION ROUTINE                ===
  // ================================================================
  volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
  void dmpDataReady() // Executed when interrupt detected. Done automatically by processor
  {
  mpuInterrupt = true;
  }

*/
/*************************
   Configuration variables
 *************************/
int gAlarmThreshold = 18; // Threshold for a High-G event
#define phoneNumber "+19492917419"

/**************************************
   MAIN CODE BLOC. DOES THE MAIN STUFF
 **************************************/
void setup()
{
  // Just a few variables, don't mind them
  //uint8_t devStatus; // status after each device operation (0 = success, !0 = error)

  // Pin modes
  pinMode(13, OUTPUT);
  pinMode(BATT_STAT_PIN, OUTPUT);
  pinMode(CELL_STAT_PIN, OUTPUT);
  pinMode(GPS_STAT_PIN, OUTPUT);
  //pinMode(INTERRUPT_PIN, INPUT);

  // Begin serial communications
  Serial.begin(9600);
  fonaSerial->begin(4800);
  fona.begin(*fonaSerial);
  //gpsSS.begin(9600);


  // Join the wire bus
  //Wire.begin();
  //Wire.setClock(400000);

  // Print SIM card IMEI number.
  char imei[16] = {}; // MUST use a 16 character buffer for IMEI!
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

  /*
     MPU Setup
  */
  /*
    mpu.initialize();
    debugLog('d', F("Initializing DMP."));
    devStatus = mpu.dmpInitialize(); // Initialize the DMP, and get its status

    // Gyro offsets, scaled for min sensitivity
     mpu.setXGyroOffset(-949);
    mpu.setYGyroOffset(-29);
    mpu.setZGyroOffset(188);
    mpu.setZAccelOffset(1447);

    // Ensure initialization worked (returns 0 if so)
    // Copied from MPU6050 library example code
    if (devStatus == 0)
    {
     mpu.setDMPEnabled(true);

     // enable Arduino interrupt detection
     attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
     mpuIntStatus = mpu.getIntStatus();

     // get expected DMP packet size for later comparison
     packetSize = mpu.dmpGetFIFOPacketSize();
     debugLog('d', F("DMP Initialization success!"));
    }
    else
    {
     // ERROR!
     // 1 = initial memory load failed
     // 2 = DMP configuration updates failed
     // (if it's going to break, usually the code will be 1)
     Serial.print(F("DMP Initialization failed (code "));
     Serial.print(devStatus);
     Serial.println(F(")"));
     debugLog('e', F("DMP Initialization failed. Code: "));
     debugLog('e', devStatus);
    }
  */

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

  periodicActions(); // Things that shouldn't happen on every loop of the processor
  /**
    // Forces the program to wait 5 seconds after bootup to start doing stuff
    // Important because the MPU gives wacky values for a bit on first startup
    if(millis() > 5000)
    {
      periodicActions(); // Things that shouldn't happen on every loop of the processor

      // Constantly encode GPS data, so that we might get a full sentence when needed
      if (gpsSS.available())
      {
        char c = gpsSS.read();
        gps.encode(c);
      }

      // Check to see if an MPU interrupt is detected, and if the FIFO buffer is full
      if (mpuInterrupt && !fifoCount < packetSize)
      {
        // reset interrupt flag and get INT_STATUS byte from the MPU
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus(); // Get interrupt status directly from the MPU. Should be true
        doCheckAccel = true; // We should check the acceleration
      }

      // Check for a High-G event
      if((aaReal.getMagnitude()/1000) > gAlarmThreshold)
      {
        debugLog('w', F("High-G event detected!"));
        // TODO: Trigger an SMS Alert
      }

      // Controls checking of acceleration from MPU. Based on flag variable, controlled by interrupt and FIFO size
      if(doCheckAccel)
      { 
      checkAccel();
      }
      
    }
  */
}

// Actions scheduled to run at certain times, all the millis stuff goes here
void periodicActions()
{
  // Check FONA for new SMS every two seconds
  if (millis() - smsTimer > 2000)
  {
    int8_t smsnum = fona.getNumSMS();
    if (smsnum > 0)
    {

      debugLog("d", "You've got mail!");
      parseAllSMS();
    }
    smsTimer = millis();
  }

  // Check battery status every five seconds
  if(millis() - battTimer > 5000)
  {
    uint16_t vbat;
    // From Adafruit FONA example code
    if (!fona.getBattPercent(&vbat)) 
    {
      debugLog('w', F("Failed to read battery voltage"));
    } 
    else 
    {
      Serial.print(F("VPct = ")); Serial.print(vbat); Serial.println(F("%"));
      // Do other stuff with voltage here
    }
  }

  // Check network status every second
  if(millis() - cellServiceTimer > 1000)
  {
    /*
     * 0 : Not registered
     * 1 : Registered (home)
     * 2 : Not registered (searching)
     * 3 : Denied
     * 4 : Unknown
     * 5 : Registered roaming
     */
    // From Adafruit FONA example code
    uint8_t n = fona.getNetworkStatus();
    if(n == 1 || n == 5)
    {
      digitalWrite(CELL_STATUS_PIN, HIGH);
    }
    else
    {
      digitalWrite(CELL_STATUS_PIN, LOW);
    }
  }

  // Check GPS status every two seconds
  if(millis() - gpsTimer > 2000)
  {
    // Do stuff here
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

void sendSMS(char sendto[], char message[])
{
  fona.sendSMS(phoneNumber, message);
}
// A utility function to get data from the MPU. Called after an interrupt is processed
// Checks for FIFO overflow, and if there's sufficient data reads it into the aaReal Struct
// Stores real acceleration, adjusted to remove gravity
/*
  void checkAccel()
  {
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    debugLog('w', F("FIFO Overflow!"));
  }
  else if (!fifoCount < packetSize) // If there's enough data in the FIFO buffer to be read
  {
    // Actually read the packet from the FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display real acceleration, adjusted to remove gravity
    // TODO: Can we do without the Quaternion?
    mpu.dmpGetQuaternion(&q, fifoBuffer); // I have no idea what this does, and at this point I'm too afraid to ask.
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); // Stores real acceleration into the aaReal struct
          Serial.print("areal\t");
          Serial.print(aaReal.x);
          Serial.print("\t");
          Serial.print(aaReal.y);
          Serial.print("\t");
          Serial.print(aaReal.z);
          Serial.print("\t");

    // If the buffer size is less than a packet, we can reset and wait for another interrupt
    if (fifoCount < packetSize)
    {
      doCheckAccel = false; // Reset check variable
    }
  }
  }

  // Structure to hold the GPS coordinates
  // Makes it easy to pass around, like some sort of digital hot potato, or deformed kick ball
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
*/
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
    digitalWrite(13, HIGH);
    //sendSMS(phoneNumber, 'b'); // TODO: Fix this!
    fona.sendSMS(phoneNumber, "bar");
    debugLog('d', F("Bar"));
  }
  else if (command == "location")
  {
    //debugLog('d', toMapsLink(getGPS()));
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
