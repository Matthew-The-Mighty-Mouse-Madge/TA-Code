/**
 * TRAUMA ALARMA TEST CODE
 * 
 * Playground for the FONA's text capabilities
 * 
 * 
 * Created 3/24/19 by Matthew
 *  
 * "The worst thing I can be is a civil engineer. I hate that."
 */
 
#include "Adafruit_FONA.h"

#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

// this is a large buffer for replies
char replybuffer[255];

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

void setup() 
{
  while (!Serial);
  pinMode(13, OUTPUT);

  Serial.begin(115200);
  Serial.println(F("FONA SMS caller ID test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  // make it slow so its easy to read!
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  Serial.println(F("FONA is OK"));

  // Print SIM card IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }

  fonaSerial->print("AT+CNMI=2,1\r\n");  //set up the FONA to send a +CMTI notification when an SMS is received

  Serial.println("FONA Ready");
}

  
char fonaNotificationBuffer[64];          //for notifications from the FONA
char smsBuffer[250];
unsigned long oldTime = 0;

void loop() 
{
  if(millis() - oldTime > 2000)
  {
    int8_t smsnum = fona.getNumSMS();
    Serial.println(smsnum);
    if(smsnum > 0)
    {
      //int8_t smsnum = fona.getNumSMS();
      uint16_t smslen;
      int8_t smsn;
      Serial.println("You've got mail!");
      for(int i = 1; i < (smsnum + 1); i++)
      {
        Serial.print("SMS Number ");
      
        Serial.println(i);
        fona.readSMS(i, replybuffer, 250, &smslen);
        Serial.println(replybuffer);
        fona.deleteSMS(i);
        Serial.print("Message ");
        Serial.print(i);
        Serial.println(" has been deleted.");
      }
    }
    oldTime = millis();
  }
}


void checkSMS()
{
  char* bufPtr = fonaNotificationBuffer;    //handy buffer pointer
  int slot = 0;            //this will be the slot number of the SMS
    int charCount = 0;
    //Read the notification into fonaInBuffer
    do  
    {
      *bufPtr = fona.read();
      Serial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaNotificationBuffer)-1)));
    
    //Add a terminal NULL to the notification string
    *bufPtr = 0;

    //Scan the notification string for an SMS received notification.
    //  If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(fonaNotificationBuffer, "+CMTI: " FONA_PREF_SMS_STORAGE ",%d", &slot)) //Scan the string for a slot number
    {
      Serial.print("slot: "); Serial.println(slot);
      
      char callerIDbuffer[32];  //we'll store the SMS sender number in here
      
      // Retrieve SMS sender address/phone number.
      if (!fona.getSMSSender(slot, callerIDbuffer, 31)) 
      {
        Serial.println("Didn't find SMS message in slot!");
      }
      
      Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);
      // Retrieve SMS value.
      uint16_t smslen;
      if (fona.readSMS(slot, smsBuffer, 250, &smslen)) // pass in buffer and max len!
      { 
        Serial.println(smsBuffer);
      }

      /**
      //Send back an automatic response
      Serial.println("Sending reponse...");
      if (!fona.sendSMS(callerIDbuffer, "Hey, I got your text!")) 
      {
        Serial.println(F("Failed"));
      } else 
      {
        Serial.println(F("Sent!"));
      }
      **/
      
      digitalWrite(13, HIGH);
      
      // delete the original msg after it is processed
      //   otherwise, we will fill up all the slots
      //   and then we won't be able to receive SMS anymore
      if (fona.deleteSMS(slot)) 
      {
        Serial.println(F("Message deleted!"));
      } 
      else 
      {
        Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
        fona.print(F("AT+CMGD=?\r\n"));
      }
    }
}

void sendSMS(char sendto[21], char message[141])
{
  fona.sendSMS(sendto, message);
}
