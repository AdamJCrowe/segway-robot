// --- Software to balance a 2 wheel mobile robot using a PID control system --- //

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



// control system inputs
const int kP = 1000;  // PID coefficients
const float kI = 10; 
const float kD = 100; 
float sensorError = -0.02;  
int minSpeed = 25 ;  

// control system variables
float desiredPitch = 0; // angle variable
float error, totalError = 0, previousError = 0; // error variables, in radians
unsigned long time, previousTime, timeElapsed = 999999; // time variables, elapsedTime needs to be large so D control initialy has little effect
int speed;



// update motor speed and direction
void updateMotors(){
    if (speed > minSpeed){
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

  // find new speed (PWM duty cycle)
  error = desiredPitch - ypr.pitch + sensorError; 
  totalError += error;
  speed = (kP * error) + (kI * totalError) + (kD * (error - previousError) / timeElapsed);

  // set new speed value and change motor direction if required
  updateMotors();
  
  // update errors and times
  previousError = error;
  time = millis();
  timeElapsed = time - previousTime;
  previousTime = time;
}
