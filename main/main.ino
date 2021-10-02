#include <AFMotor.h>      //add Adafruit Motor Shield library
#include <NewPing.h>      //add Ultrasonic sensor library
#include <IRremote.h>

//I screwed up A3 pin 
#define TRIG_PIN A3 // Pin A0 on the Motor Drive Shield soldered to the ultrasonic sensor
#define ECHO_PIN A1 // Pin A1 on the Motor Drive Shield soldered to the ultrasonic sensor
#define REMOTE_PIN A2 // Pin for remote receiver
#define SERVO_PIN 10
#define MAX_DISTANCE 300 // sets maximum useable sensor measuring distance to 300cm
#define MAX_SPEED 160 // sets speed of DC traction motors to 150/250 or about 70% of full speed - to get power drain down.
#define MAX_SPEED_OFFSET 40 // this sets offset to allow for differences between the two DC traction motors
#define COLL_DIST 30 // sets distance at which robot stops and reverses to 30cm
#define TURN_DIST COLL_DIST+20 // sets distance at which robot veers away from object
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // sets up sensor library to use the correct pins to measure distance.


//remote controller ir value in integer 
#define BUTTON_1 69
#define BUTTON_2 70
#define BUTTON_3 71
#define BUTTON_4 68
#define BUTTON_5 64
#define BUTTON_6 67
#define BUTTON_7 7
#define BUTTON_8 21
#define BUTTON_9 9
#define BUTTON_USERMODE 22
#define BUTTON_AUTOMODE 13
#define BUTTON_UP 24
#define BUTTON_LEFT 8
#define BUTTON_RIGHT 90
#define BUTTON_DOWN 82
#define BUTTON_OK 28
#define MAX_SPEED 200


AF_DCMotor leftMotor1(1, MOTOR12_1KHZ); // create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor leftMotor2(2, MOTOR12_1KHZ); // create motor #2, using M2 output, set to 1kHz PWM frequency
AF_DCMotor rightMotor1(3, MOTOR34_1KHZ);// create motor #3, using M3 output, set to 1kHz PWM frequency
AF_DCMotor rightMotor2(4, MOTOR34_1KHZ);// create motor #4, using M4 output, set to 1kHz PWM frequency

String control_mode = "USERMODE";
int user_control_direction = 0;
int speed = 150;
const int default_speed = 150;

int leftDistance, rightDistance; //distances on either side
int curDist = 0;
String motorSet = "";
int speedSet = 0;

int angle;
int pwm;

const int FORWARD_DEG = 70;
const int RIGHT_DEG = 16;
const int LEFT_DEG = 124;

class PrevDistances{
  public:
    PrevDistances() : prev_distances({MAX_DISTANCE+100,MAX_DISTANCE+100}), ptr(0){}
    void insert(unsigned int a_distance){
      prev_distances[ptr] = a_distance;
      ptr = (ptr+1) % 2;
    }
    bool isStuck(unsigned int a_distance){
      if (abs(prev_distances[0] - a_distance) > 5){
        return false;
      }
      if (abs(prev_distances[1] - a_distance) > 5){
        return false;
      }
      return true;
    }
  private:
    int prev_distances[2];
    int ptr;
};

PrevDistances prev_left;
PrevDistances prev_right;
PrevDistances prev_forward;


void setup() {
  // put your setup code here, to run once:
  pinMode(SERVO_PIN,OUTPUT);
  Serial.begin(9600);
  IrReceiver.begin(REMOTE_PIN,false);
  movePulse(FORWARD_DEG);
}

void loop() {
  changeMode();
  if (control_mode == "AUTOMODE"){
    autoModeWork();
  } else {
    userModeWork();
    delay(200);
  }
}

// get IR signal and change control_mode
void changeMode(){
  if(IrReceiver.decode()){
    if(IrReceiver.decodedIRData.command == BUTTON_USERMODE){
      control_mode = "USERMODE";
      //stops everything
    } else if (IrReceiver.decodedIRData.command == BUTTON_AUTOMODE){
      control_mode = "AUTOMODE";
    } else {
        if (control_mode == "AUTOMODE"){
          control_mode = "USERMODE";
        } else { 
          user_control_direction = IrReceiver.decodedIRData.command;
          // do usermode.
        }
    }
    Serial.println("Change mode"); 
    IrReceiver.resume();
  } else {
    if (control_mode == "USERMODE"){
      moveStop();
    }
    user_control_direction = 0;
  }
  
}

void moveStop(){
  leftMotor1.run(RELEASE); 
  leftMotor2.run(RELEASE); 
  rightMotor1.run(RELEASE); 
  rightMotor2.run(RELEASE);
  leftMotor1.setSpeed(0);
  leftMotor2.setSpeed(0);
  rightMotor1.setSpeed(0); 
  rightMotor2.setSpeed(0);
}

void autoModeWork(){
  moveStop();
  changeMode(); //to change mode after this work done
  movePulse(FORWARD_DEG);
  curDist = readPing();
  if ( (curDist < COLL_DIST)){ 
    changePath();
    moveStop();
  } else {
    moveForward();
  }
  unsigned long travel_time = curDist * 8;
  delayWithIR(travel_time); // travel time decreases as distance from object decreases
  readPing(); // reset ping
  isStuck();
}

void isStuck(){
  if (prev_forward.isStuck(curDist)){
    for(int i = 0 ; i < 3 ; ++i){
      search();
    if(prev_left.isStuck(leftDistance && prev_right.isStuck(rightDistance))){
      stuckExit();
      return;
  }
    prev_left.insert(leftDistance);
    prev_right.insert(rightDistance);
  }
  } else {
    prev_forward.insert(curDist);
  }
}

void stuckExit(){
      unsigned long t = millis();
      switch(t % 7){
        case 0:
          turnLeft();
          turnLeft();
          break;
        case 1:
          turnRight();
          turnRight();
          break;
        case 2:
          moveBackward();
          break;
        case 3:
          moveForward();
          turnLeft();
          break;
        case 4:
          moveForward();
          turnRight();
          break;
        case 5:
          moveBackward();
          turnLeft();
          break;
        case 6:
          moveBackward();
          turnRight();
          break;
        default:
          break;
      }
}

int readPing() { // read the ultrasonic sensor distance
  delayWithIR(100);
  unsigned int uS = sonar.ping();
  int cm = uS/US_ROUNDTRIP_CM;
  return cm;
}

void changePath() {
    search();
    compareDistance();
}

void search(){
    moveStop();   // stop forward movement
    movePulse(RIGHT_DEG);  // check distance to the right
    delayWithIR(500);
    rightDistance = readPing(); //set left distance
    readPing();
    movePulse(LEFT_DEG); // left
    delayWithIR(500);
    leftDistance = readPing(); //set right distance
    delayWithIR(500);
    movePulse(FORWARD_DEG);
}

void movePulse(int x){
    int del=(7*x)+500;
    for (int pulseCounter=0; pulseCounter<=50; pulseCounter++){
        changeMode(); //to change mode after this work done
        digitalWrite(SERVO_PIN,HIGH);
        delayMicroseconds(del);
        digitalWrite(SERVO_PIN,LOW);
        delayWithIR(20); // between pulses
    }
}

void compareDistance()   // find the longest distance
{
  if (leftDistance>rightDistance && leftDistance >= COLL_DIST) //if left is less obstructed 
  {
    turnLeft();
  }
  else if (rightDistance>leftDistance && rightDistance >= COLL_DIST) //if right is less obstructed
  {
    turnRight();
  }
  else //if they are equally obstructed
  {
    turnAround();
  }
}

// gives delay according to @param time, while continously getting IR signal from user
void delayWithIR(unsigned long time){
    unsigned long c = millis();
    unsigned long prev_mill = c;
    while (c < prev_mill + (time/2.5)){
      changeMode();
      c = millis();
    }
}

void moveForward() {
    motorSet = "FORWARD";
    leftMotor1.run(FORWARD);      // turn it on going forward
    leftMotor2.run(FORWARD);      // turn it on going forward
    rightMotor1.run(FORWARD);     // turn it on going forward
    rightMotor2.run(FORWARD);     // turn it on going forward
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    changeMode(); //to change mode after this work done
    leftMotor1.setSpeed(speedSet);
    leftMotor2.setSpeed(speedSet);
    rightMotor1.setSpeed(speedSet); 
    rightMotor2.setSpeed(speedSet);
    delayWithIR(5);
  }
}


//-------------------------------------------------------------------------------------------------------------------------------------
void moveBackward() {
    motorSet = "BACKWARD";
    leftMotor1.run(BACKWARD);     // turn it on going backward
    leftMotor2.run(BACKWARD);     // turn it on going backward
    rightMotor1.run(BACKWARD);    // turn it on going backward
    rightMotor2.run(BACKWARD);    // turn it on going backward
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    changeMode(); //to change mode after this work done
    leftMotor1.setSpeed(speedSet);
    leftMotor2.setSpeed(speedSet);
    rightMotor1.setSpeed(speedSet); 
    rightMotor2.setSpeed(speedSet); 
    delayWithIR(5);
  }
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnRight() {
  motorSet = "RIGHT";
  leftMotor1.run(FORWARD);      // turn motor 1 forward
  leftMotor2.run(FORWARD);      // turn motor 2 forward
  rightMotor1.run(BACKWARD);    // turn motor 3 backward
  rightMotor2.run(BACKWARD);    // turn motor 4 backward
  rightMotor1.setSpeed(speedSet+MAX_SPEED_OFFSET);      
  rightMotor2.setSpeed(speedSet+MAX_SPEED_OFFSET);     
  delayWithIR(1000);
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);      // set both motors back to forward
  leftMotor2.run(FORWARD);
  rightMotor1.run(FORWARD);
  rightMotor2.run(FORWARD);      
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnLeft() {
  motorSet = "LEFT";
  leftMotor1.run(BACKWARD);      // turn motor 1 backward
  leftMotor2.run(BACKWARD);      // turn motor 2 backward
  rightMotor1.run(FORWARD);     // turn motor 3 forward
  rightMotor2.run(FORWARD);     // turn motor 4 forward
  leftMotor1.setSpeed(speedSet+MAX_SPEED_OFFSET);     
  leftMotor2.setSpeed(speedSet+MAX_SPEED_OFFSET);    
  delayWithIR(1000);
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);      // turn it on going forward
  leftMotor2.run(FORWARD);      // turn it on going forward
  rightMotor1.run(FORWARD);     // turn it on going forward
  rightMotor2.run(FORWARD);     // turn it on going forward
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnAround() {
  motorSet = "RIGHT";
  leftMotor1.run(FORWARD);      // turn motor 1 forward
  leftMotor2.run(FORWARD);      // turn motor 2 forward
  rightMotor1.run(BACKWARD);    // turn motor 3 backward
  rightMotor2.run(BACKWARD);    // turn motor 4 backward
  rightMotor1.setSpeed(speedSet+MAX_SPEED_OFFSET);      
  rightMotor2.setSpeed(speedSet+MAX_SPEED_OFFSET);
  delayWithIR(3200);
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);      // set both motors back to forward
  leftMotor2.run(FORWARD);
  rightMotor1.run(FORWARD);
  rightMotor2.run(FORWARD);      
}  

//automode


void userModeWork(){
  switch (user_control_direction) {
    case BUTTON_1:
        search();
        break;
    case BUTTON_2:
        leftMotor1.run(FORWARD);      // turn it on going forward
        leftMotor2.run(FORWARD);      // turn it on going forward
        rightMotor1.run(FORWARD);     // turn it on going forward
        rightMotor2.run(FORWARD);     // turn it on going 
        leftMotor1.setSpeed(speed);
        leftMotor2.setSpeed(speed);
        rightMotor1.setSpeed(speed); 
        rightMotor2.setSpeed(speed);
        break;
    case BUTTON_3:
        turnLeft();
        break;
    case BUTTON_4:
        leftMotor1.run(FORWARD);      // turn motor 1 forward
        leftMotor2.run(FORWARD);      // turn motor 2 forward
        rightMotor1.run(FORWARD);    // turn motor 3 backward
        rightMotor2.run(FORWARD);    // turn motor 4 backward
        rightMotor1.setSpeed(speed);      
        rightMotor2.setSpeed(speed);   
        break;
    case BUTTON_5:
        break;
    case BUTTON_6:
          
        leftMotor1.run(FORWARD);      // turn motor 1 forward
        leftMotor2.run(FORWARD);      // turn motor 2 forward
        rightMotor1.run(FORWARD);    // turn motor 3 backward
        rightMotor2.run(FORWARD);    // turn motor 4 backward
        leftMotor1.setSpeed(speed);      
        leftMotor2.setSpeed(speed);     
        
        break;
    case BUTTON_7:
        leftMotor1.run(FORWARD);      // turn motor 1 forward
        leftMotor2.run(FORWARD);      // turn motor 2 forward
        rightMotor1.run(BACKWARD);    // turn motor 3 backward
        rightMotor2.run(BACKWARD);    // turn motor 4 backward
        rightMotor1.setSpeed(speed);      
        rightMotor2.setSpeed(speed);   
        break;
    case BUTTON_8:
        leftMotor1.run(BACKWARD);     // turn it on going backward
        leftMotor2.run(BACKWARD);     // turn it on going backward
        rightMotor1.run(BACKWARD);    // turn it on going backward
        rightMotor2.run(BACKWARD);    // turn it on going backward
        leftMotor1.setSpeed(speed);
        leftMotor2.setSpeed(speed);
        rightMotor1.setSpeed(speed); 
        rightMotor2.setSpeed(speed);
        break;
    case BUTTON_9:
        leftMotor1.run(BACKWARD);      // turn motor 1 forward
        leftMotor2.run(BACKWARD);      // turn motor 2 forward
        rightMotor1.run(FORWARD);    // turn motor 3 backward
        rightMotor2.run(FORWARD);    // turn motor 4 backward
        leftMotor1.setSpeed(speed);      
        leftMotor2.setSpeed(speed);  
        break;
    case BUTTON_UP:
        speed = min(speed+10,MAX_SPEED);
        break;
    case BUTTON_DOWN:
        speed = max((speed - 10), 105);
        break;
    case BUTTON_OK:
        speed = default_speed;
        break;
    default:
      //stops
      break;

  }
}