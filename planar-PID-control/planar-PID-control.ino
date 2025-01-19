// --- Software to balance a 2 wheel mobile robot using an LQR control system, only moving forwards and backwrads (planar) --- //

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// BNO85 IMU setup
#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportTypePitch = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

// angular position and speed variables
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

// PID control system constants
const float Kpid[] = {-1400, -7.3, -15}; // PID gains for body angle
const int backlashFix = 30; // value to reduce the effect of gearbox backlash, by making the motors start ahead of time
float sensorError = 0.098;  // IMU might be assembled offset (rad)
const int minSpeed = 50;  // minimum PWM value
const float loopTime = 1.5 / 1000; // software loop time (s)

const bool home = false; // the system will attempt to move back to the starting position if true
const float posControlMaxDist = 0.5 * (2 / 0.121); // if the wheels move past this point, the system won't lean out anymore (first number is distance in m)
const float posControlMinDist = 0.01 * (2 / 0.121); // if the wheels move within this point, the system won't lean out anymore (first number is distance in m)
const float posControlMaxPitch = 0.08; // maximum pitch angle the system can lean out by (rad)
const float posControlMinPitch = 0.015; // minimum pitch angle the system can lean out by (rad)
const float posControlGradient = (posControlMaxPitch - posControlMinPitch) / (posControlMaxDist - posControlMinDist); // where y=mx+c this is m
const float posControlIntercept = posControlMaxPitch - (posControlGradient * posControlMaxDist);  // where y=mx+c this is c

// control system variables
unsigned long time, previousTime;
float pidOutput, torqueA, torqueB;   // initial PID torque, desired torque for motor A and B
int pwmA=0, pwmB=0;  // PWM duty cycle for motor A and B
float currentPitch, previousPitch = 0, currentVel, integralPitch; // variables for body angle/velocity
float wheelPosAverage; // used for finding the angle to lean out at

// encoder constants
const float wheelPosCalc = 6.283 / 748; // constant used to turn encoder pulses to radians
const float wheelVelCalc = wheelPosCalc / loopTime; // constant used to turn encoder pulses per loop to rad/s

// encoder variables
int countIncrementsA = 0, relativeIncrementsA = 0, absoluteIncrementsA = 0; // wheel A position in encoder pulses
float wheelPosA = 0, wheelVelA = 0; // wheel A position and velocity in rad/s
int countIncrementsB=0, relativeIncrementsB = 0, absoluteIncrementsB = 0; // wheel B position in encoder pulses
float wheelPosB = 0, wheelVelB = 0; // wheel B position and velocity in rad/s
int currentDirectionA = 2, previousDirectionA = -2; // wheel A direction of rotation
int currentDirectionB = 2, previousDirectionB = -2; // wheel B direction of rotation


// function prototypes
void updateMotors();
void pidResponse();
void findWheelState();
void encoderStateA();
void encoderStateB();
void quaternionToEuler(float, float, float, float, euler_t*, bool);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t*, euler_t*, bool);



void setup(){
  // set motor driver pins to write
  DDRE = 0x08; // D5 (ENB)
  DDRJ = 0x03; // D14 (IN1), D15 (IN2)
  DDRH = 0x0B; // D6 (ENA), D16 (IN3), D17 (IN4)

  // setup PWM for timers 3 and 4 - WGM set to fast PWM, COM set to normal operation, CS is 7.8kHz
  TCCR3A = 0x83; 
  TCCR3B = 0x0C; 
  TCCR4A = 0x83;  
  TCCR4B = 0x0C;  

  // interrupts used to track encoder pulses (use rising edges)
  attachInterrupt(digitalPinToInterrupt(18), encoderStateA, RISING); 
  attachInterrupt(digitalPinToInterrupt(19), encoderStateA, RISING); 
  attachInterrupt(digitalPinToInterrupt(2), encoderStateB, RISING); 
  attachInterrupt(digitalPinToInterrupt(3), encoderStateB, RISING); 

  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause until serial console opens
  delay(2000);

  // start communiciation with the IMU
  while (!bno08x.begin_I2C()) delay(10);  
  while (! bno08x.enableReport(reportTypePitch, reportIntervalUs)) delay(10);
  
  Serial.println("System started correctly");
}


void loop(){
  // find pitch angle
  do{
    if (bno08x.getSensorEvent(&sensorValue)) {
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
    }
  }
  while (ypr.pitch == 0); // wait until data is recieved

  // capture current data
  relativeIncrementsA = countIncrementsA;
  countIncrementsA = 0;
  relativeIncrementsB = countIncrementsB;
  countIncrementsB = 0;

  findWheelState(); // check if motors have changed direction, then update absolute position/velocity

  pidResponse(); // calculate the PID response of the system based on the target pitch angle (0 is making the system balance)

  pwmA = pidOutput;
  pwmB = pidOutput;
  
  updateMotors(); // account for backlash and set new motor speed and change motor direction if required

  // update variables
  previousDirectionA = currentDirectionA; 
  previousDirectionB = currentDirectionB; 
  previousPitch = currentPitch;
}



// check if motors have changed direction
void findWheelState(){
  if (previousDirectionA == currentDirectionA) {
    if (currentDirectionA == 1){ // wheel moving forwards and has not changed direction
      absoluteIncrementsA += relativeIncrementsA;
    }
   else{ // wheel moving backwards and has not changed direction
      absoluteIncrementsA -= relativeIncrementsA;
      relativeIncrementsA *= -1;
    }
  }
  else{ // wheel has changed direction - assume wheel hasn't rotated this software loop
    relativeIncrementsA = 0;
  }

  if (previousDirectionB == currentDirectionB) { // repeat for wheel B
    if (currentDirectionB == 1){
      absoluteIncrementsB += relativeIncrementsB;
    }
    else{
      absoluteIncrementsB -= relativeIncrementsB;
      relativeIncrementsB *= -1;
    }
  }
  else{
    relativeIncrementsB = 0;
  }

  // calculate position and speed for wheel A and B
  wheelPosA = float(absoluteIncrementsA) * wheelPosCalc;
  wheelVelA = float(relativeIncrementsA) * wheelVelCalc;
  wheelPosB = float(absoluteIncrementsB) * wheelPosCalc;
  wheelVelB = float(relativeIncrementsB) * wheelVelCalc;
}


// calculate the PID response of the system based on the target pitch angle (0 is making the system balance)
void pidResponse(){
  currentPitch = ypr.pitch - sensorError;

  if (home == true){ // move the system back to the home position if required
    wheelPosAverage = (wheelPosA + wheelPosB) / 2;

    // system response (min step, ramp, max step) due to wheel position error
    if (wheelPosAverage > posControlMaxDist){ // maximum and positive step response
      currentPitch -= posControlMaxPitch;
    }
    else if (wheelPosAverage < -posControlMaxDist){ 
      currentPitch += posControlMaxPitch;
    }
    else if ((wheelPosAverage < posControlMinDist) && (wheelPosAverage > -posControlMinDist)){ 
      if (wheelPosAverage > 0){ // minimum and positive step response
        currentPitch -= posControlMinPitch;
      }
      else{
        currentPitch += posControlMinPitch;
      }
    }
    else{ // ramp response
      currentPitch -= posControlGradient * wheelPosAverage + posControlIntercept;
    }
  }

  // PID state variables
  currentVel = (currentPitch - previousPitch) / loopTime;
  integralPitch += currentPitch;

  // calculate
  pidOutput = Kpid[0]*currentPitch +  Kpid[1]*integralPitch + Kpid[2]*currentVel;
}


// account for backlash and set motor duty cycle / direction 
void updateMotors(){
  // account for backlash in the gearbox by actuating the motors ahead of time
  if (pwmA > 0){
    pwmA += backlashFix;
  }
  else {
    pwmA -= backlashFix;
  }
  if (pwmB > 0){
    pwmB += backlashFix;
  }
  else {
    pwmB -= backlashFix;
  }

  // update motor A's PWM duty cycle based on pwmA
  if (pwmA >= minSpeed){ // set motor A to forwards
    currentDirectionA = 1;
    PORTJ = 0x01; // D14 output high and D16 output low
    pwmA += backlashFix; // account for gearbox backlash
    if (pwmA < 0x03FF){
      OCR4A = pwmA;   // D6 PWM value
    }
    else { //max motor speed
      OCR4A = 0x03FF;
    }
  }
  else if (pwmA <= -minSpeed) { // set motor A to reverse
    currentDirectionA = -1;
    PORTJ = 0x02; // D14 output low and D16 output high 
    pwmA -= backlashFix;
    if (pwmA > -0x03FF){
      OCR4A = -pwmA;   
    }
    else { 
      OCR4A = 0x03FF;
    }
  }
  else{ // not enough torque to drive the motors
    PORTJ = 0x00; // D14 output low and D16 output low
    OCR4A = 0;   
  }

  if (pwmB >= minSpeed){
    currentDirectionB = 1;
    PORTH = 0x01; // D15 output high and D17 output low
    pwmB += backlashFix;
    if (pwmB < 0x03FF){
      OCR3A = pwmB;   // D5 PWM value
    }
    else {
      OCR3A = 0x03FF;
    }
  }
  else if (pwmB <= -minSpeed) {
    currentDirectionB = -1;
    PORTH = 0x02; // D15 output low and D17 output high
    pwmB -= backlashFix;
    if (pwmB > -0x03FF){
      OCR3A = -pwmB;
    }
    else {
      OCR3A = 0x03FF;
    }
  }
  else{ // not enough torque to drive the motors
    PORTH = 0x00; // D15 output low and D17 output low
    OCR3A = 0;
  }
}


// interrupts - increments variable on the encoder's rising edges
void encoderStateA(){
  countIncrementsA++;
}
void encoderStateB(){
  countIncrementsB++;
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