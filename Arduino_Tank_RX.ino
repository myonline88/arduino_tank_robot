#include <Servo.h> 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "SoftwareSerial.h"
const int rxpin = 0;   // if you wish to use bluetooth reverse the normal 
const int txpin = 1;   // rx-tx for software serial to enable programming while bluetooth is connected

SoftwareSerial blue(rxpin, txpin); // new serial port on pins 2 and 3`
 
/*----- Declare all constant pin to be used ----*/
 
#define CE_PIN   9    //2.4GHz CE_pin of NRF24L01 module
#define CSN_PIN  10    //2.4GHz CSN_pin of NRF24L01 module

const uint64_t pipe = 0xE8E8F0F0E1LL; // This is the transmit pipe to communicate the two module
 
/*-----Object Declaration ----*/
RF24 radio(CE_PIN, CSN_PIN); // Activate  the Radio

String content = "",di="D";   // for bluetooth
int vStatus = 'g';    // for bluetooth 
 
 
int joystick[8];  // 8 element array holding the Joystick readings


Servo scanservo;     // servo for the robotic arm's elbow 
Servo baseservo;     // servo for the robotic arm's elbow 
Servo armservo;      // servo for the robot's arm  
Servo handservo;     // servo for the of robot's hand  
Servo gripperservo;  // servo for the of robot's gripper  
int currentpos,currentpos2,currentpos3;
int pos,pos2,pos3,pos4;    // variable to store the servo position 
int leftdist=0;
int rightdist=0;
int centerdist=0;

int servopower=12;
int rightview = 115;
int centerview = 102;
int leftview = 68;
int speedA = 255;
int speedB = 255;
int motorspeed;
int initialdelay = 1000;

boolean spotlighton = false;
int spotlightpin = 23;                              
unsigned long last = millis();    //remember when we last received an IRmessage
boolean leftd=false;
boolean centerd=false;
boolean rightd=false;
boolean elbowdwn=true;
boolean motorpoweron = false;

int outputValue = 0;        // value output to the PWM (analog out)

char contcommand;
int modecontrol;
int power;
int elbowpos = 0;
const int pingPin = A5;    // A5; // Trigger pin  
const int echoPin = A4;    // A4; // Echo pin  

int frontDistance = 0;
boolean autonomous =false;
int averageDistance = 0;        // stores the average value
int averageDistance2 = 0;       // stores the average value

const int motor1pin1  = 3;    //3       // left motor pin1  
const int EnableA     = 5;    //5       // left motor enable
const int motor1pin2  = 4;    //4       // left motor pin2

const int motor2pin1  = 7;    //7        // right motor pin1
const int EnableB     = 6;    //6        // right motor enable
const int motor2pin2  = 8;    //8        // right motor pin2

void setup() { 
 Serial.begin(9600); /* Opening the Serial Communication */
 delay(1000);
// blue.begin(9600);
// blue.setTimeout(10);
 Serial.println("Nrf24L01 Receiver Starting");
 
 radio.begin();
 radio.setPALevel(RF24_PA_MIN);
 radio.setAutoAck(true);
 radio.openReadingPipe(1,pipe);
 radio.startListening();
 radio.powerUp();


  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(EnableA, OUTPUT);
  pinMode(EnableB, OUTPUT);

  analogWrite(EnableA, speedA);
  analogWrite(EnableB, speedB);

  scanservo.attach(2);   // 2;  for the 9g servo holding the HC-SR04
  baseservo.attach(A0);   // A0; robotic arm's base 
  armservo.attach(A1);    // A1; robotic arm's elbow
  handservo.attach(A2);   // A2; robotic arm's hand
  gripperservo.attach(A3);// A3; robotic arm's gripper
  scanservo.write(90);
  
  int baseservopos=91;
  int armservopos=13;
  int handservopos=175;
  int gripperservopos=120;

  robotarmreset();
  power =0;
}

void loop()   /****** MAIN LOOP: RUNS CONSTANTLY ******/
{
  if (radio.available())
  {
    // Read the data payload until we've received everything
      // Fetch the data payload
       radio.read( joystick, sizeof(joystick) );
       if(joystick[6] == 1) { power=1;  
       robotarmreset();
       autobot(); 
       } else { 
       nrf24L01();
       }
   }  else  {    
      Serial.println("No radio available");      
      halt();
      autobot();   // Autonomous mode. HC-SR04 must be installed
  }

}//--(end main loop )---

void robotarmreset(){
  baseservo.write(91);
  armservo.write(13);
  handservo.write(175);
  gripperservo.write(120);
}

void nrf24L01() {
 
//      radio.powerUp();
      Serial.print("X =");
      Serial.print(joystick[1]);
      Serial.print("\tY =");      
      Serial.print(joystick[0]);

      // motors for moving Left or Right
      if(joystick[1] >120) { motorspeed=joystick[1]+75; turnright(); }
      if(joystick[1] <95) { motorspeed=288-joystick[1]; turnleft(); }
      if(joystick[1] >=95 && joystick[1] <=120) { halt(); motorspeed=0; }
      Serial.print("\tX-Speed: ");      
      Serial.print(motorspeed);

     // motors for moving Forward or Backward
      if(joystick[0] >130) { motorspeed=joystick[0]+75; forward(); }
      if(joystick[0] <100) { motorspeed=288-joystick[0]; backward(); }
      if(joystick[0] >=100 && joystick[0] <=130) { halt();  motorspeed=0;}
      Serial.print("\tY-Speed: ");      
      Serial.print(motorspeed);

      // Operates the robotic arm
      Serial.print("\t Base = ");      
      Serial.print(joystick[2]);
      Serial.print("\t Arm =");      
      Serial.print(joystick[3]);
      Serial.print("\t Hand =");      
      Serial.print(joystick[4]);
      Serial.print("\t Gripper =");      
      Serial.print(joystick[5]);

      Serial.print("\tMode =");      
      Serial.print(joystick[6]);
      Serial.print("\tSpeed =");      
      Serial.println(joystick[7]);
           
      baseservo.write(joystick[2]);
      armservo.write(joystick[3]);
      handservo.write(joystick[4]);
      gripperservo.write(joystick[5]);    
                
 }

// Get the Distance returned by the HC-SR04
int ping(){
  long duration, inches;
  //Send Pulse
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  //Read Echo
  pinMode(pingPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

 // convert the time into a distance
  inches = microsecondsToInches(duration);
 // Serial.print("Ping:  ");
 // Serial.println(inches);
  return round(inches);
}

long microsecondsToInches(long microseconds){
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds){
  return microseconds / 29 / 2;
} 

void forward() {
   analogWrite(EnableA, motorspeed);
   analogWrite(EnableB, motorspeed);
  
   digitalWrite(motor1pin1, LOW);
   digitalWrite(motor1pin2, HIGH);

   digitalWrite(motor2pin1, LOW);
   digitalWrite(motor2pin2, HIGH);
}

void backward() {
   analogWrite(EnableA, motorspeed);
   analogWrite(EnableB, motorspeed);

   digitalWrite(motor1pin1, HIGH);
   digitalWrite(motor1pin2, LOW);

   digitalWrite(motor2pin1, HIGH);
   digitalWrite(motor2pin2, LOW);  
}

void halt() {
   analogWrite(EnableA, motorspeed);
   analogWrite(EnableB, motorspeed);

   digitalWrite(motor1pin1, LOW);
   digitalWrite(motor1pin2, LOW);

   digitalWrite(motor2pin1, LOW);
   digitalWrite(motor2pin2, LOW);
}

void turnleft() {
   analogWrite(EnableA, motorspeed);
   analogWrite(EnableB, motorspeed);

   digitalWrite(motor1pin1, HIGH);
   digitalWrite(motor1pin2, LOW);

   digitalWrite(motor2pin1, LOW);
   digitalWrite(motor2pin2, HIGH);
}

void turnright() {
   analogWrite(EnableA, motorspeed);
   analogWrite(EnableB, motorspeed);
 
   digitalWrite(motor1pin1, LOW);
   digitalWrite(motor1pin2, HIGH);

   digitalWrite(motor2pin1, HIGH);
   digitalWrite(motor2pin2, LOW);
} 

void bluetooth() {
  if(blue.available()>0){ // 
   vStatus = blue.read();
   }

   if(vStatus=='a'){ // Forward
    Serial.println(vStatus);
    forward();
   }  

   if(vStatus=='d'){ // right
    Serial.println(vStatus);
    turnright();
   }

  if(vStatus=='c'){ // Stop
    Serial.println(vStatus);
    halt();
  }

  if(vStatus=='b'){ // left
    Serial.println(vStatus);
    turnleft();
  } 
 
  if(vStatus=='e'){ // Reverse
    Serial.println(vStatus);
    backward();
  }

  if (vStatus =='f'){ // Boton ON se mueve sensando distancia 
    power = 1;
    if(power == 1) { autobot(); }
  }

  if (vStatus=='g'){ // Boton OFF, detiene los motores no hace nada 
    power = 0;
   halt();
 }
} 


void autobot() {

  motorspeed = 225;   //change this according to the speed of your
                      //desired motors speed

  averageDistance = ping();
  Serial.print("Distance from object: ");
  Serial.println(averageDistance);

  if (averageDistance > 16) {
  // go forward
     forward();
     pos=centerview;
     scanservo.write(pos);
     } else {
     evaluatepath();
  //   camerascan();
  }

  // executes the pre-evaluated option
  if (averageDistance > 7 && averageDistance < 14) {
     speedA = speedA-(255/averageDistance);
     speedB = speedB-(255/averageDistance);  
   // backward();
   // delay(170);
    findroute();
   }

  if (averageDistance >=0 && averageDistance <=7) {   
     speedA = speedA-(255/averageDistance);
     speedB = speedB-(255/averageDistance);  
     backward();
     delay(300);
     halt();
     delay(300);
     findEscape();
  } 
  if (averageDistance <=5 && leftdist <=5 && rightdist<=5) {     
     halt();
     delay(300);
     findEscape();
  } 
}

void findEscape() {
  // if got trapped on a difficult obstacle on all sides,
  // it will repeat scanning around while turning right until it found a clear way

  while (centerdist < 12 && rightdist <5 && leftdist <5 ) {
   halt();
   delay(300);
   scanservo.write(leftview);
   delay(300);
   leftdist = ping();
 // Serial.println(leftdist);
   scanservo.write(rightview);
   delay(600);
   rightdist = ping();
 // Serial.println(rightdist);
   scanservo.write(centerview);
   delay(600);
   centerdist = ping();
 // Serial.println(centerdist); 
    turnright();
    delay(200);
    }
 // --- after scanning the terrain and if it found a clear way, it goes forward
   forward(); 
}

// ==== Performs evaluation from a farther distance to execute slight course changes==== 
void findbetterpath() {
  if(leftdist > 10 && leftdist < 14 && centerdist > 15) {
    turnright();
    delay(800);
    } else {
    forward();
  } 
  if(rightdist > 10 && rightdist < 14 && centerdist > 15) {
    turnleft();
    delay(800);
    } else {
    forward();
  }  
}
// ======= performs course direction based on the left or right distances 
// ======= detected by the Sonar which are evaluated depending
// ======= on which way is wider and clearer 

void findroute() {
   scanservo.write(leftview);
   delay(300);
   leftdist = ping();
 // Serial.println(leftdist);

   scanservo.write(rightview);
   delay(600);
   rightdist = ping();
 // Serial.println(rightdist);

  if(leftdist < rightdist) {
   scanservo.write(centerview);
   averageDistance = ping();
    while (averageDistance  < 18 ) {
    turnright();
    averageDistance = ping();
    }
    forward();
  } else {     //if(leftdist > rightdist) {
   scanservo.write(centerview);
   averageDistance = ping();
    while (averageDistance  < 18 ) {
    turnleft();
    averageDistance = ping();
    }
    forward();
  }  
 }

// === when the robot accidentally bumpt on obstacle
// === which the sonar failed to detect early

void evaluatepath() {
   lookleft();
   lookright();
   lookcenter();
}

// ======= Each look-function performs abrupt course direction if the robot 
// ======= happens to run at close proximity on either left
// ======= or right side. It will measure its distance from 
// ======= the sides at a clear distance of at least 12 inches

void lookleft() {
// look left ----
   scanservo.write(leftview);
   delay(250);
   leftdist = ping();

   if (leftdist < 18) {
    halt();
    delay(200);
    while (leftdist < 17 ) {
    scanservo.write(leftview);
     turnright();
    leftdist = ping();
   }
   scanservo.write(centerview);
  }
}

void lookright() {
// look right ----
   scanservo.write(rightview);
   delay(600);
   rightdist = ping();

 //  Serial.print("rightdist: ");
 //  Serial.println(rightdist);

   if (rightdist < 18) {
    halt();
    delay(200);
    while (rightdist < 17 ) {
    scanservo.write(rightview);
     turnleft();
     rightdist = ping();
    }
   scanservo.write(centerview);
  }
}

void lookcenter(){
// look center ----
   scanservo.write(centerview);
   delay(250);
   averageDistance=ping();
   centerdist=ping();

   if (centerdist < 15) {
   backward();
   delay(200);
   turnleft();
   delay(800);
   } else {
   scanservo.write(centerview);
   forward();
  }
}
           
