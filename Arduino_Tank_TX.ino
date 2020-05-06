/* Example: nRF24L01 
 
   1 - GND 
   2 - VCC 3.3V  
   3 - CE - Arduino pin 9
   4 - CSN - Arduino pin 10
   5 - SCK - Arduino pin 13
   6 - MOSI - Arduino pin 11
   7 - MISO - Arduino pin 12
   8 - UNUSED 
 
  You can used an Analog Joystick or 2 10K potentiometers 
 
   GND - Arduino GND 
   VCC - Arduino +5V
   X Pot - Arduino A0
   Y Pot - Arduino A1
 
/*----- Import all required Libraries -----*/
 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
 
/*----- Declare all constant pin to be used ----*/
 
#define CE_PIN   9  // Uno-Nano = 9     // Mega = 48
#define CSN_PIN  10  // Uno-Nano = 10    // Mega = 53

#define JOYSTICK_Y A0
#define JOYSTICK_X A1
#define JOYSTICK_SBASE A2
#define JOYSTICK_ARM A3
#define JOYSTICK_HAND A4
#define JOYSTICK_GRIPPER A5
#define JOYSTICK_SPEED A6

const uint64_t pipe = 0xE8E8F0F0E1LL; // This is the transmit pipe to communicate the two module
 
/*-----Object Declaration ----*/
RF24 radio(CE_PIN, CSN_PIN); // Activate  the Radio
 
/*-----Declaration of Variables -----*/ 
int joystick[8];  // Six element array holding the Joystick readings

boolean spotlighton = false;
int spotlightpin = 2;                              
unsigned long last = millis(); //remember when we last received an IRmessage
 
void setup()   
{
  Serial.begin(9600); /* Opening the Serial Communication */
  radio.begin();
 // radio.setPALevel(RF24_PA_MAX);
 // radio.setDataRate(RF24_1MBPS);      // set data rate through the air
 // radio.setRetries(1, 2);            // set the number and delay of retries
  radio.setAutoAck(true);
  radio.openWritingPipe(pipe);
  pinMode(spotlightpin , INPUT);

}//--(end setup )---
 
void loop()   /* Runs Continuously */
{
  joystick[0] = map(analogRead(JOYSTICK_Y), 0, 1023, 179, 0);     // Reading Analog Y
  joystick[1] = map(analogRead(JOYSTICK_X), 0, 1023, 179, 0);     // Reading Analog X
  joystick[2] = map(analogRead(JOYSTICK_SBASE), 0, 1023, 179, 0);   // Reading Analog 2
  joystick[3] = map(analogRead(JOYSTICK_ARM), 0, 1023, 179, 0);   // Reading Analog 3
  joystick[4] = map(analogRead(JOYSTICK_HAND), 0, 1023, 179, 0);  // Reading Analog 4
  joystick[5] = map(analogRead(JOYSTICK_GRIPPER), 0, 1023, 179, 0);   // Reading Analog 5
  joystick[6] = digitalRead(spotlightpin);        // Reading digital pin 2
  joystick[7] = map(analogRead(JOYSTICK_SPEED), 0, 1023, 0, 255);   // Reading Analog 6
 
 // Serial.println(joystick[0]);
 // Serial.println(joystick[1]);

 // if(digitalRead(up) == LOW) {
 //   joystick[2] = 130; // Reading Analog Z
 //   Serial.println("Up");
 // }  

  if(digitalRead(spotlightpin) == HIGH) {
       if (millis() - last > 250) {     //has it been 1/4 sec since last message
         spotlighton = !spotlighton;    //toggle the spotlight power supply
         joystick[6] = spotlighton;           // Reading pin 6
      //   Serial.println(joystick[6]);
         } else {
         joystick[6] = spotlighton;           // Reading pin 6
      //   Serial.println(joystick[6]);
        }
       last = millis();
     }
  
  radio.powerUp();
  delay(5);
  radio.write( joystick, sizeof(joystick) );

}//--(end main loop )---
