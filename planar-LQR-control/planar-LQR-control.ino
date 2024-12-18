// --- Software to balance a 2 wheel mobile robot using an LQR control system, only moving forwards and backwrads (planar) --- //

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// setup sensor, for SPI mode we need a CS pin and reset
#define BNO08X_CS 10
#define BNO08X_INT 9 
#define BNO08X_RESET -1

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

// angular position variables
struct euler_t {
  float pitch;
} ypr;


// control system constants
const float K[] = {-0.141, -0.165, -15.9, -1.32}; // LQR control matrix, K, gains (taken from simulation)
const float sensorError = 0;  // IMU might be assembled at a slight angle
const float speedCalc = 255 / 0.8; // scale the motor torque to speed
const int minSpeed = 26;  // minimum PWM value

// control system variables
unsigned long time, previousTime;
float torque;   // desired motor torque
int speed;  // PWM duty cycle

// encoder constants
const float wheelPosCalc = 748 / 6.283; // constant used to turn encoder pulses to radians
const float wheelVelCalc = wheelPosCalc / 1000; // constant used to turn encoder pulses to rad/s

// encoder variables
int countPos=0, relativePos = 0, absolutePos = 0; // wheel position in encoder pulses
int currentDirection=2, previousDirection=-2; // wheel direction of rotation
float wheelPos, wheelVel;

// function prototypes
void updateMotors();
void findMotorDirection();
void encoderState();
void quaternionToEuler(float, float, float, float, euler_t*, bool);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t*, euler_t*, bool);



void setup(){
  // set motor driver pins to write
  DDRE = 0x08; // D5 (ENB)
  DDRJ = 0x03; // D14 (IN1), D15 (IN2)
  DDRH = 0x0B; // D6 (ENA), D16 (IN3), D17 (IN4)

  // setup PWM for timers 3 and 4 - WGM set to fast PWM, COM set to normal operation, CS is 7.8kHz
  TCCR3A = 0x81; 
  TCCR3B = 0x0B; 
  TCCR4A = 0x81;  
  TCCR4B = 0x0B;  

  // interrupts used to track encoder pulses (use rising edges)
  attachInterrupt(digitalPinToInterrupt(2), encoderState, RISING); 
  attachInterrupt(digitalPinToInterrupt(3), encoderState, RISING); 

  // initialize sensor 
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  while (!bno08x.begin_I2C()) delay(10);  // try to initialize sensor
  while (! bno08x.enableReport(reportType, reportIntervalUs)) delay(10);
}


void loop(){
  // find pitch angle
  do{
    if (bno08x.getSensorEvent(&sensorValue)) {
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
          break;
      }
    }
  }
  while ((ypr.pitch == 0) || (sensorValue.un.gyroscope.y == 0)); // wait until data is recieved

  // capture data
  time = millis();
  relativePos = countPos;
  countPos = 0;

  // check if motors have changed direction, then update absolute position
  findMotorDirection();

  // calculate wheel pos and speed
  wheelPos = float(absolutePos) / wheelPosCalc;
  wheelVel = float(relativePos) / (float(time - previousTime) * wheelVelCalc);

  // calculate speed (PWM duty cycle) based on LQR control law
  torque = 0*K[0] * wheelPos + 0*K[1] * wheelVel + K[2] * (ypr.pitch-sensorError) + 0*K[3] * sensorValue.un.gyroscope.y;
  speed = torque * speedCalc;

  // set new speed value and change motor direction if required
  updateMotors();

  // update variables
  previousDirection = currentDirection; 
  previousTime = time;
}



void updateMotors(){
  if (speed > minSpeed){
    currentDirection = -1;
    PORTJ = 0x03;  // D14 & D15 output high
    PORTH = 0x00;  // D16 & D17 output low
    if (speed < 255){
      OCR3A = speed;   // D5 PWM value
      OCR4A = speed;   // D6 PWM value
    }
    else {
      OCR3A = 0xFF;
      OCR4A = 0xFF;
    }
  }
  else if (speed < -minSpeed) {
    currentDirection = 1;
    PORTJ = 0x00;
    PORTH = 0x03;    
    if (speed > -255){
      OCR3A = -speed;
      OCR4A = -speed;
    }
    else {
      OCR3A = 0xFF;
      OCR4A = 0xFF;
    }
  }
  else{ // not enough torque to drive the motors
    PORTJ = 0x00;  
    PORTH = 0x00;  
    OCR3A = 0;
    OCR4A = 0;
  }
}


// check if motors have changed direction
void findMotorDirection(){
  if (previousDirection == currentDirection) {
    if (currentDirection == 1){
      absolutePos += relativePos;
    }
   else{
      absolutePos -= relativePos;
      relativePos = -relativePos;
    }
  }
  else{
    relativePos = 0;
  }
}


// interrupt, increments variable on the encoder's rising edges
void encoderState(){
  countPos++;
}


// pitch angle function
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
}


// convert from quaternion to euler angles
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}