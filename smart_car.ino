#include <AFMotor.h>
AF_DCMotor motorlu(4);
AF_DCMotor motorld(1);
AF_DCMotor motorrd(2);
AF_DCMotor motorru(3);

#include <SoftwareSerial.h>

SoftwareSerial mySerial(7, 8); // RX, TX  

// Connect HM10      Arduino Uno

//     Pin 1/TXD          Pin 7

//     Pin 2/RXD          Pin 8

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init w as successful
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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

int trig = 51, echo = 50;
unsigned int duration, distance;
// Values for joistic
  int forward_int = 1;
  int back_int = -1;
  int stop_int = 0;
  int right_int = 2;
  int left_int = -2;
  bool if_on_car = false;
  int current_side = 0;
  int bluetooth_value;
  int rht, lft;
  
// Values for accelerometer
// angles
  float yaw_0;
  float dyaw_0 = 0.0;
  

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
volatile int mpuOverflows = 0;
void dmpDataReady() {
  if(mpuInterrupt)
    ++mpuOverflows;
  mpuInterrupt = true;
}

float get_angle();
int check_side();

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {   
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    //while (!Serial); 

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // turn on motor
    motorlu.setSpeed(200);
    motorld.setSpeed(200);
    motorru.setSpeed(200);
    motorrd.setSpeed(200);
    motorlu.run(RELEASE);
    motorld.run(RELEASE);
    motorru.run(RELEASE);
    motorrd.run(RELEASE);

    while (Serial.available() && Serial.read()); // empty buffer
    while (Serial.available() && Serial.read()); // empty buffer again

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    // wait for 30 seconds
  
    delay(30000);
    
    yaw_0 = get_angle();

    pinMode (trig, OUTPUT);
    pinMode (echo, INPUT);
    //mySerial.begin(115200);
    Serial.println('1');
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void forward();
void right();
void left();
void stap();
void move_right_90(float an_pre);
void move_left_90(float an_pre);
int get_bluetooth();

void loop(){
    
  bluetooth_value = get_bluetooth();
  yaw_0 = get_angle();
    
  if (bluetooth_value == forward_int){
      if_on_car = true;
  }
  else if (bluetooth_value == stop_int)
  {
      if_on_car = false;
  }
    
  //Serial.println(current_side);
  if (if_on_car){
    current_side = check_side();
    if (current_side == forward_int){
    forward();  
    }
    else if (current_side == left_int){
      move_left_90(yaw_0);
    }
    else if (current_side == right_int){
      move_right_90(yaw_0);
    }
  }
  else {
    stap();
  }
}


int get_bluetooth(){
  char c == 'a';  

  if (mySerial.available()) {
    c = mySerial.read();
    if (c == '1')
    {
      return forward_int;
    }
    else if (c == '0')
    {
      return stop_int;
    }
    else 
    {
      return right_int;
    }
  }
}


int check_side(){
  // return left_int if left
  // return forward_int if forward
  // return right_int if right
  Serial.println('a');
 
  // Make with sonar
  digitalWrite(trig, HIGH);
  delayMicroseconds (10);
  digitalWrite(trig, LOW);
  delayMicroseconds (1);
  duration = pulseIn(echo, HIGH);
  Serial.println(duration);
  if (duration < 37000){
    distance = duration * 0.034/2;
    Serial.println(distance);
    if (distance > 20){
      return forward_int;
    }
  }
  rht = analogRead(A0);
  lft = analogRead(A1);
  Serial.print("L:");
  Serial.print(lft);
  Serial.print(" R:");
  Serial.print(rht);
  if (rht < lft) {
    Serial.println(" | R");
    return right_int;
  } 
  else {
    Serial.println(" | L");
    return left_int;
  }
}


float get_angle() {

    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();

    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    
    return ypr[0] * 180/M_PI;
}


void move_right_90(float an_pre){
  float an, dan = 0.0;
  right();  
  while (1 == 1){
    an = get_angle();
    //Serial.println(an);
    dan += an - an_pre;
    Serial.print("Ang:");
    Serial.println(dan);
    if ((dan >= 90.0) || (dan <= -90.0)){
      break;
    }
    an_pre = an;
  }
  forward();
}


void move_left_90(float an_pre){
  float an, dan = 0.0;
  left();  
  while (1 == 1){
    an = get_angle();
    dan += an - an_pre;
    if ((dan > 90.0) || (dan < -90.0)){
      break;
    }
    an_pre = an;
  }
  forward();
}


void forward(){
  motorlu.setSpeed(200);
  motorld.setSpeed(200);
  motorru.setSpeed(200);
  motorrd.setSpeed(200);
  motorlu.run(FORWARD);
  motorld.run(FORWARD);
  motorrd.run(FORWARD);
  motorru.run(FORWARD);
}

void right() {
  motorlu.setSpeed(200);
  motorld.setSpeed(200);
  motorru.setSpeed(200);
  motorrd.setSpeed(200);
  motorlu.run(FORWARD);
  motorld.run(FORWARD);
  motorrd.run(BACKWARD);
  motorru.run(BACKWARD);
}

void left() {
  motorlu.setSpeed(200);
  motorld.setSpeed(200);
  motorru.setSpeed(200);
  motorrd.setSpeed(200);
  motorlu.run(BACKWARD);
  motorld.run(BACKWARD);
  motorrd.run(FORWARD);
  motorru.run(FORWARD);
}
 
 void stap() {
  motorlu.setSpeed(200);
  motorld.setSpeed(200);
  motorru.setSpeed(200);
  motorrd.setSpeed(200);
  motorlu.run(RELEASE);
  motorld.run(RELEASE);
  motorru.run(RELEASE);
  motorrd.run(RELEASE);
}
m
