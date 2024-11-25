// --- Software to balance a segway robot using an LQR control system --- //

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// setup sensor
// for SPI mode, we need a CS pin and reset
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

  // interrupts used to track encoder pulses (rising edges)
  attachInterrupt(digitalPinToInterrupt(2), encoderState, RISING); 
  attachInterrupt(digitalPinToInterrupt(3), encoderState, RISING); 

  // initialize sensor 
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  while (!bno08x.begin_I2C()) delay(10);  // try to initialize sensor
  while (! bno08x.enableReport(reportType, reportIntervalUs)) delay(10);
}

// pitch angle functions
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}



// control system settings
const int kP = 100;//1400;  // PID coefficients
const float kI = 0;//10
const float kD = 0;//300; 
float sensorError = 0;  
int minSpeed = 26;  

// control system variables
float desiredPitch = 0; // angle variable
float currentError, totalError = 0, previousError = 0, errorRate = 0; // error variables, in radians
unsigned long time, previousTime = 999999; // time variables, previousTime needs to start off large so D control initialy has little effect
int speed;

// encoder variables
int countPos=0, relativePos = 0, absolutePos = 0; // wheel position in encoder pulses
int currentDirection, previousDirection; // wheel direction of rotation
float wheelPos, wheelVel;
float wheelPosCalc = 748 / 6.283; // constant used to turn encoder pulses to radians
float wheelVelCalc = wheelPosCalc / 1000; // constant used to turn encoder pulses to rad/s

void encoderState(){
  countPos++;
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
  while (ypr.pitch == 0);


  // capture data
  time = millis();
  relativePos = countPos;
  countPos = 0;
  
  // calculate speed
  currentError = desiredPitch - ypr.pitch + sensorError; 
  totalError += currentError;
  errorRate = (currentError - previousError) / (time - previousTime);
  speed = (kP * currentError) + (kI * totalError) + (kD * errorRate);



  // check if motors have changed direction, then update absolute position
  if (previousDirection == currentDirection) {
    if (currentDirection == 1){
      absolutePos += relativePos;
    }
   else{
      absolutePos -= relativePos;
    }
  }
  else{
    relativePos = 0;
  }

  // calculate wheel pos and speed
  wheelPos = float(absolutePos) / wheelPosCalc;
  wheelVel = float(relativePos) / (float(time - previousTime) * wheelVelCalc);

  // set new speed value
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

  // reset variable
  previousDirection = currentDirection; 
  previousError = currentError;
  previousTime = time;


  Serial.println(wheelPos);
  Serial.println(wheelVel);

  /*Serial.print("Integrated Gyro - x: ");
  Serial.print(sensorValue.un.gyroscope.x);
  Serial.print(" y: ");
  Serial.print(sensorValue.un.gyroscope.y);
  Serial.print(" z: ");
  Serial.println(sensorValue.un.gyroscope.z);*/

  //Serial.println(currentError);

  /*count++;
  if (count >=10000){
    Serial.println(timers-millis());
    count=0;
  }*/

}
