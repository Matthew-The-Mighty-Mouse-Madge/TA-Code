// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define LED_PIN 13
bool blinkState = false;

unsigned long microTimer = 0;
bool prevGAlarm = false;
int counter = 0;

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)

  Wire.begin();


  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values

  Serial.println("Updating internal sensor offsets...");
  // -76	-2359	1688	0	0	0
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  accelgyro.setXGyroOffset(-861);
  accelgyro.setYGyroOffset(-36);
  accelgyro.setZGyroOffset(180);
  accelgyro.setXAccelOffset(-3097);
  accelgyro.setYAccelOffset(-417);
  accelgyro.setZAccelOffset(4421);
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");


  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  // read raw accel/gyro measurements from device
  //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available

  //accelgyro.getRotation(&gx, &gy, &gz);

  // display tab-separated accel/gyro x/y/z values

  //Serial.print(gx); Serial.print("\t");
  //Serial.print(gy); Serial.print("\t");
  //Serial.println(gz);

  if (micros() - microTimer > 50)
  {
    accelgyro.getRotation(&gx, &gy, &gz);
    
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.print(vectorMag()); Serial.print("\t");
    Serial.print(millis()); Serial.println("\t");
    

    if (vectorMag() > 23)
    {
      counter++;
      prevGAlarm = true;
      //Serial.println("HIGH G");
    }
    else
    {
      if (prevGAlarm && counter > 3 && counter < 15)
      {
        Serial.println("=========ALARM===========");
      }
      counter = 0;
    }

    microTimer = 0;
  }

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

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
