//#define CALIBRATE
#define DEBUG

#define UNO

#define USE_MPU6050 // Download MPU6050 by Electronic Cats from Arduino library manager
//#define USE_MPU9250

#define RATE_HZ 200

#ifdef USE_MPU6050
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
#define INTERRUPT_PIN 2
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
#endif

#ifdef USE_MPU9250
#include "MPU9250.h"
MPU9250 mpu;
#endif


// hatire structure
struct  {
  int16_t  Begin  ;   // begin
  uint16_t Cpt ;      // Frame Number or Error code
  float    gyro[3];   // [Y, P, R] Yaw, Pitch, Roll
  float    acc[3];    // [x, y, z] Displacements
  int16_t  End ;      // end
} hat;


long loopTimer = 0;
int sleepTime = 1000/RATE_HZ;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  Serial.begin(115200);
  Wire.begin();
  // header frame for hatire
  hat.Begin=0xAAAA;
  // Frame Number or Error code
  hat.Cpt=0;
  // footer frame for hatire
  hat.End=0x5555; 
  while (!Serial);
  delay(2000);

  #ifdef USE_MPU6050
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
  #else
  mpu.setup();
  #endif

  //mpu.setAccBias(+78.75, 144.67, -201.21);  //x, y, z   
  //mpu.setGyroBias(-0.5, 2.3, -1.2);     //x, y, z
  //mpu.setMagBias(+104.97, +57.91, -418.89); //x, y, z
  //mpu.setMagScale(+1.24, +0.94, +0.89);     //x, y, z
  digitalWrite(LED_BUILTIN,HIGH);
}

void loop() {
#ifdef CALIBRATE
  calibrate();
#else
  #ifdef USE_MPU6050
  updateMPU6050();
  #else
  updateMPU9250();
    #ifdef DEBUG
  debugMode();
    #endif
  #endif
#endif
  
  
}

void calibrate() {
  #ifdef USE_9250
  Serial.println("calibration mode");
  delay(5000);
  Serial.println("Calibrating...");
  // calibrate anytime you want to
  
  mpu.calibrate();

  mpu.printCalibration();
  Serial.println("Calibrating done!");
  delay(5000);
  #endif
}

#ifdef USE_MPU9250
void updateMPU9250() {
  if ( loopTimer < millis()) {
    loopTimer = millis() + sleepTime;
    
    mpu.update();
   
    hat.gyro[0]=mpu.getYaw();
    hat.gyro[1]=mpu.getPitch();
    hat.gyro[2]=mpu.getRoll();
    
    // Send HAT Frame to PC
    Serial.write((byte*)&hat,30);
    hat.Cpt++;
    if (hat.Cpt>999) {
      hat.Cpt=0;
    }
  }
}

void debugMode() {
  mpu.print();
  Serial.print("Yaw:");
  Serial.print(mpu.getYaw());
  Serial.print("\tPitch:");
  Serial.print(mpu.getPitch());
  Serial.print("\tRoll:");
  Serial.println(mpu.getRoll()); 
}
#endif

#ifdef USE_MPU6050
void updateMPU6050() {
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop 
      fifoCount = mpu.getFIFOCount();
    }
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize){
      //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
  // check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
    Serial.println(F("FIFO overflow!"));
  
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

    // read a packet from FIFO
    while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }
 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    hat.gyro[0]=ypr[0] * 180/M_PI;
    hat.gyro[1]=ypr[1] * 180/M_PI;
    hat.gyro[2]=ypr[2] * 180/M_PI;
    
    // Send HAT Frame to PC
    Serial.write((byte*)&hat,30);
    hat.Cpt++;
    if (hat.Cpt>999) {
      hat.Cpt=0;
    }
  }
}
#endif
