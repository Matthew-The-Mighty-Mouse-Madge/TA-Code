/**
   TRAUMA ALARMA TEST CODE

   Playground for the MPU6050 motion sensor, getting G-alarms to work
   Based heavily (copied almost) from Jeff Rowberg's MPU6050 example code

   Created 3/23/19 by Matthew

   "The worst thing I can be is a civil engineer. I hate that."
*/
#include <I2Cdev.h>
//#include <MPU6050.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

//Create the MPU object
MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
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

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
  // Just a few variables, don't mind them
  uint8_t devStatus; // status after each device operation (0 = success, !0 = error)

  // Join the wire bus
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Gyro offsets, scaled for min sensitivity
  mpu.setXGyroOffset(-949);
  mpu.setYGyroOffset(-29);
  mpu.setZGyroOffset(188);
  mpu.setZAccelOffset(1447);

  // Ensure initialization worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));


    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
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
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
bool doCheckAccel = false; // Should we check for accel data from MPU?
bool highG = false; // Has a high-G event been detected?
int gAlarmThreshold = 18;
void loop()
{
  /**
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
    if (mpuInterrupt && fifoCount < packetSize)
    {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }

    } **/
  
  if (mpuInterrupt && !fifoCount < packetSize) // An interrupt has been processed and the fifo is full
  {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus(); // Get interrupt status directly from the MPU. Should be true
    doCheckAccel = true; // We should check the acceleration
  }

  if (doCheckAccel) // Let's check the acceleration
  {
    checkAccel();
  }
  Serial.println(aaReal.getMagnitude());
  
  if((aaReal.getMagnitude()/1000) > gAlarmThreshold)
  {
    highG = true;
  }
/*
  if(highG)
  {
    Serial.println("[ALERT] Impact detected!");
    highG = false; // Reset the flag
  }
  */
}

// After an interrupt is detected, get the actual data from the accelerometer
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
    Serial.println(F("FIFO overflow!"));
  }
  else if (!fifoCount < packetSize) // If there's enough data in the FIFO buffer to be read
  {
    // Actually read the packet from the FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer); // TODO: Can we do without the Quaternion?
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); // Stores real acceleration into the aaReal struct
    /*
          Serial.print("areal\t");
          Serial.print(aaReal.x);
          Serial.print("\t");
          Serial.print(aaReal.y);
          Serial.print("\t");
          Serial.print(aaReal.z);
          Serial.print("\t");
    */
    //Serial.print("Magnitude\t");
    //Serial.println(vectorMag());
    //debugMag();
    // If the buffer is less than a packet, we can reset and wait for another interrupt
    if (fifoCount < packetSize)
    {
      doCheckAccel = false; // Reset check variable
    }
  }
}


/* DEPRECATED
  // A utility function that calculates the magnitude of an acceleration vector
  // References aaReal, a global struct that holds the latest acceleration data
  double vectorMag()
  {
  double sqx = sq(aaReal.x);
  double sqy = sq(aaReal.y);
  double sqz = sq(aaReal.z);
  double asqx = abs(sqx);
  double asqy = abs(sqy);
  double asqz = abs(sqz);
  double magnitude = sqrt(asqx+asqy+asqz); // TODO: Is this going to cause memory problems??
  return magnitude;
  }

  void debugMag()
  {
  double sqx = sq(aaReal.x);
  double sqy = sq(aaReal.y);
  double sqz = sq(aaReal.z);
  double asqx = abs(sqx);
  double asqy = abs(sqy);
  double asqz = abs(sqz);
  double sum = asqx + asqy + asqz;
  double root = sqrt(sum);
  Serial.print("Square magnitudes\t");
  Serial.print(sqx);
  Serial.print("\t");
  Serial.print(sqy);
  Serial.print("\t");
  Serial.print(sqz);
  Serial.print("\t");
  Serial.print(sum);
  Serial.print("\t");
  Serial.println(root);
  }
*/
