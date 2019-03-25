/**
 * TRAUMA ALARMA MAIN CODE
 * 
 * ENGR-152 Product design project
 * Cole Weidkamp, Garrett Cooper, Matthew Martin, Marco Scoma
 * 
 * Created 3/23/19 by Matthew
 * 
 * "Code does not come from winning. Your struggles develop your code. When you go through crashes and decide not to surrender, that is programming."
 * "The worst thing I can be is a civil engineer. I hate that."
 * "Windows is for babies. When you grow up you have to use Linux."
 */

#include <Adafruit_FONA.h>
#include <TinyGPS.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Project variables, no concern for most people
int debugMode = 0; 


/**
 * MAIN CODE BLOC. DOES THE MAIN STUFF
 */
void setup() 
{
  Serial.begin(38400);
}

void loop() 
{
  if(Serial.read())
  {
    serialCommands(Serial.read());
  }

}


/**
 * SOVIET CODE BLOC. FUNCTION OWNED BY EVERYBODY, LIKE TRUE COMMUNISM.
 */

 // A helper function to manage logging to the serial console
 // Has 3 log levels, Debug (2, lowest), Warn (1, medium), and Error (0, highest)
 void debugLog(String level, String message)
 {
    if(level == "e")
    {
      Serial.print("[ERROR] ");
      Serial.println(message);
    }

    if(level == "w" && debugMode >= 1)
    {
      Serial.print("[WARNING] ");
      Serial.println(message);
    }

    if(level == "d" && debugMode >=2)
    {
      Serial.print("[DEBUG] ");
      Serial.println(message);
    }
 }

 // A helper function to extract commands from serial inputs
 // Mostly for developer purposes, not used at the moment
 void serialCommands(String command)
 {
    // Do some stuff here
 }
