/**
 * TRAUMA ALARMA TEST CODE
 * 
 * Playground for the GPS sensor
 * 
 * Created 3/23/19 by Matthew
 *  
 * "The worst thing I can be is a civil engineer. I hate that."
 */
 
#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial ss(4, 3);
 
void setup() 
{
  Serial.begin(38400);
  ss.begin(9600);
  pinMode(6, INPUT);
}

void loop() 
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  /*
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available()) // While there's bytes to read
    {
      char c = ss.read(); 
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
      {
        float flat, flon;
        unsigned long age;
        gps.f_get_position(&flat, &flon, &age);
        Serial.print("LAT=");
        Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
        Serial.print(" LON=");
        Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      }
    }
  }
  */

  if(ss.available())
  {
    char c = ss.read();
    gps.encode(c);
  }
  
  if(digitalRead(5))
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  }
/*
  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    //Serial.print(" SAT=");
    //Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    //Serial.print(" PREC=");
    //Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  */
  /*
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
  Serial.println("** No characters received from GPS: check wiring **");
  */
}
