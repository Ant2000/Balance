#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define LED_PIN 13
float a,b,c;
bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady() 
{
    mpuInterrupt = true;
}
void setup() 
{
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24;
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(-25);
  mpu.setYGyroOffset(11);
  mpu.setZGyroOffset(-7);
  mpu.setXAccelOffset(-3495);
  mpu.setYAccelOffset(-654);
  mpu.setZAccelOffset(1048);
  if (devStatus == 0) {
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else 
  {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}
void pullen()
{
  digitalWrite(13,HIGH);
  digitalWrite(12,LOW);
}
void rellen()
{
  digitalWrite(13,LOW);
  digitalWrite(12,HIGH);
}
void pullbth()
{
  digitalWrite(10,HIGH);
  digitalWrite(11,LOW);
}
void relbth()
{
  digitalWrite(10,LOW);
  digitalWrite(11,HIGH);
}
void haltlen()
{
  digitalWrite(13,LOW);
  digitalWrite(12,LOW);
}
void haltbth()
{
  digitalWrite(11,LOW);
  digitalWrite(10,LOW);
}
void loop() 
{
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
  {
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02) 
  {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      a=ypr[0] * 180/M_PI;
      b=ypr[1] * 180/M_PI;
      c=ypr[2] * 180/M_PI;
      Serial.print("ypr\t");
      Serial.print(a);
      Serial.print("\t");
      Serial.print(b);
      Serial.print("\t");
      Serial.println(c);
  }
  if(c<0.00)
  {
    rellen();
  }
  else if(c>1.50)
  {
    pullen();
  }
  else
  {
    haltlen();
  }
  if(b<17.50)
  {
    relbth();
  }
  else if(b>19.00)
  {
    pullbth();
  }
  else
  {
    haltbth();
  }
}
