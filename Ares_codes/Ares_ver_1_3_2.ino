/*
	Name: Ares Mini Sumo Robot
	Copyright: ©UoP Robotics Team (https://github.com/UoP-Robotics)
	Authors: Katerina Tertimpa, Georgios Pegiazis (https://github.com/GeorgePeg)
	Date: 02/04/26 15:48
 	Version: 1.3.2
  	License: GNU General Public License v3.0, 29 June 2007
  	Description: Latest version of Ares for the robotic competition Robotic Arena with remote control using RC5 protocol with a LED turning
  	on & off for debugging.
  	Application for debugging: https://play.google.com/store/search?q=ble+controller+%E2%80%93+arduino+esp32&c=apps
*/
#define IR_USE_ESP32_RMT_TIMER true 
#include <Arduino.h>
#include <IRremote.hpp>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// DEFINE FOR DIRECTIONS 
#define LEFT 0
#define RIGHT 1
int searchDir = LEFT;
bool edgeActive = false ;
float k = 1.36; //-> Smoothing factor

// BUTTONS
#define BUTT_1 18
#define BUTT_2 36
int m ;
bool aosComplete = false;

// LINE SENSORS  (ANALOG INPUTS) 
#define LINE_L 17 //
#define LINE_R 12 //

// OPP SENSORS  (DIGITAL INPUTS )
#define PIN_OPP_L  16
#define PIN_OPP_FL 15
#define PIN_OPP_FC 7
#define PIN_OPP_FR 10
#define PIN_OPP_R 11

// LEFT MOTOR 
#define L_IN1  48  
#define L_IN2  45 
#define PWM_L  47

// RIGHT MOTOR 
#define R_IN1  3  
#define R_IN2  46 
#define PWM_R   9

// LEDS
#define LED_1  21
#define LED_2  23

// PID PARAMETERS   
float Kp = 1.75;     /*Numbers may have to change through testing*/
float Ki = 0.27;
float Kd = 0.01;

float pidError = 0;
float pidLastError = 0;
float pidIntegral = 0;

//---Bluetooth Connection Parameters---
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

//IR PIN
const int IR_PIN = 42; 
int code = 0;

/*State Machine using enumeration. Why?:
  ->To secure that every time we push a button remotely, the only thing that changes is the state we are.
  ->For readability issues.
  ->To secure that in every revision of the loop(), the robot will look ONLY at current state, and it will run the commands we want continiously.
  ->To provide a PERMANENT MEMORY to our robot.
*/
enum RobotState {
  STOP, ATTACK, SEARCH
}; 
//Initial State
RobotState currentState = STOP;

//--- Bluetooth Callback Class to Check Connectivity ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
        deviceConnected = true;
        pServer->updateConnParams(pServer->getConnId(), 0x06, 0x12, 0, 100);
    };
    void onDisconnect(BLEServer* pServer) { 
        deviceConnected = false;
        pServer->getAdvertising()->start(); 
    }
};
//---Function for debugging through BLE---
void debugMsg(String msg) {
  Serial.println(msg);
  if (deviceConnected) {
    pTxCharacteristic->setValue(msg.c_str());
    pTxCharacteristic->notify();
    delay(20);
  }
}
//---Function to handle the right motor---
void motorRight(int speed){
  if (speed >= 0 ){
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  } else {
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH); speed = -speed; 
  }
  analogWrite(PWM_R, constrain(speed, 0, 255));
}
//---Function to handle the left motor---
void motorLeft(int speed){
  if (speed >= 0 ){
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
  } else {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH); speed = -speed; 
  }
  analogWrite(PWM_L, constrain(speed, 0, 255));
}
//---Function to stop the robot when it needs---
void motorStop(){
  analogWrite(PWM_L, 0); analogWrite(PWM_R, 0);  
}
//---Start routine Function---
void startRoutine(){

   unsigned long edgeStart = 0;
   int edgeState = 0;

  while (edgeState < 3){

 switch(edgeState){
    //TURN 45 DEGREES
    case 0 :
    edgeStart = millis();
    motorRight(-60);
    motorLeft(0);
    edgeState = 1;  // MOVE TO NEXT CASE 
    break;

    case 1:    
    if (millis() - edgeStart >= 300){
      edgeStart = millis();
      // MOVE FORWARD
      motorRight(60);
      motorLeft(60);
      edgeState = 2 ;    // MOVE TO NEXT CASE
    }
    break;

    case 2:
     if ( millis() - edgeStart >= 500){
        motorStop();      
        edgeStart = millis();
        edgeState = 3;   // MOVE THE NEXT CASE 
     } break;

    // EXIT CASE
    default: 
      break; 
    }
    delay(10);
  }
}
//---Search Function---
void search(int line_L, int line_R, int dir){
  // MOVE IN A CIRCULAR MOTION  
  if (dir == LEFT){
   motorRight(40); 
   motorLeft(60);
  } else {
   motorRight(60); 
   motorLeft(40);
  }
  debugMsg("SEARCH DIR: " + String(dir));
}
//---Function that detects whether the robot is inside the ring area or at the edges---
void foundEdge(int dir){ 
  static unsigned long edgeStart = 0;
  static int edgeState = 0;
   int i = k*100;
 switch(edgeState){
    //REVERSE
    case 0 :
    edgeStart = millis();
    motorRight(-i);
    motorLeft(-100);
    edgeState = 1;  // MOVE TO NEXT CASE 
    break;

    case 1:    // ΑΛΛΑΓΕΣ ΓΙΑ ΑΝΑ ΔΩ ΑΜΑ ΟΝΤΩΣ ΕΙΝΑΙ 180
    if (millis() - edgeStart >= 300) {
      edgeStart = millis();
       //ROTATE                         
      if (dir == LEFT){
       // ROTATE LEFT BACKWARDS
       motorRight(100); 
       motorLeft(-100);  
      } else {
       // ROTATE RIGHT BACKWARDS 
       motorRight(-100); 
       motorLeft(100); 
      }
      edgeState = 2 ;    // MOVE TO NEXT CASE
    }
    break;

    case 2:
     if ( millis() - edgeStart >= 500){
      //edgeStart = millis();
      // STOP MOTORS
       motorStop();
       edgeState = 3;   // MOVE THE NEXT CASE 
     } break;
  
    case 3 : 
      edgeState = 0 ;  // RESET TO BE READY FOR THE NEXT EVENT 
      edgeActive = false ;
      break ;
  } 
}
void AOS(){
  static unsigned long aosStart = 0;
  static int aosStep = 0;

  if (aosComplete) return; // Αν τελείωσε, μην ξαναμπείς

  switch(aosStep) {
    case 0: // Ξεκίνα το χρόνο
      aosStart = millis();
      motorStop();
      aosStep = 1;
      debugMsg("AOS: Waiting...");
      break;

    case 1: // Περίμενε 1.5 δευτερόλεπτο
      if (millis() - aosStart >= 1500) {
        aosStart = millis();
        motorLeft(175);
        motorRight(175);
        aosStep = 2;
        debugMsg("AOS: Moving!");
      }
      break;

    case 2: // Κινήσου για 500ms και μετά σταμάτα
      if (millis() - aosStart >= 500) {
        motorStop();
        aosComplete = true; // ΤΕΛΟΣ ΡΟΥΤΙΝΑΣ
        currentState = SEARCH; // Γύρνα αυτόματα σε SEARCH mode
        debugMsg("AOS Done -> Searching");
      }
      break;
  }
}

//---PID Function---
int PID(int error) {
   pidError = (float)error;
   pidIntegral += pidError;
   float derivative = pidError - pidLastError;

   float output = Kp * pidError + Ki * pidIntegral + Kd * derivative;

   // SPEED LIMITATION FOR SMOTHER MOVEMENT
   if (output > 40) output = 40;       
   if (output < -40) output = -40;

   pidLastError = pidError;
   return (int)output;
}
//---Function that implements the Fuzzy Logic Algorithm using the correction from the PID Algorithm in order to attack. 
void fuzzyAttack(int line_L, int line_R) {
 //MAP OPPONENT SENSORS TO FUZZY DIRECTION
  int fuzzyL  =  (digitalRead(PIN_OPP_L)  == LOW) ;
  int fuzzyFL =  (digitalRead(PIN_OPP_FL)  == LOW) ;
  int fuzzyFR =  (digitalRead(PIN_OPP_FR)  == LOW) ;
  int fuzzyR  =  (digitalRead(PIN_OPP_R)  == LOW) ;
  int fuzzyFC = (digitalRead(PIN_OPP_FC) == LOW);

  // OPPONENT DIRECTION (-100 LEFT, +100 RIGHT)
  int opponentDir = (fuzzyFR *60 + fuzzyR*100) - (fuzzyFL*60 + fuzzyL*100);
  //Calling the PID function to get the correction
  int correction = PID(opponentDir);

  int baseSpeed = (digitalRead(PIN_OPP_FC) == LOW) ? 255 : 150; // BASE FORWARD SPEED 
  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = (baseSpeed + correction)*k; //-> Using the smoothing factor to "balance" the movement of each motor

  // CLAMP SPEEDS
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  motorLeft(leftSpeed);
  motorRight(rightSpeed);

  debugMsg("ERR: " + String(opponentDir) + " CORR: " + String(correction));
} 

//---Set-Up Function.Here the code runs only once---
void setup() {
  Serial.begin(115200);
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(PWM_L, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(PWM_R, OUTPUT);
  // SENSOR PINS
  pinMode(LINE_L, INPUT);
  pinMode(LINE_R, INPUT);
  pinMode(PIN_OPP_L,  INPUT_PULLUP);
  pinMode(PIN_OPP_FL, INPUT_PULLUP);
  pinMode(PIN_OPP_FC, INPUT_PULLUP); 
  pinMode(PIN_OPP_FR, INPUT_PULLUP);  
  pinMode(PIN_OPP_R,  INPUT_PULLUP); 
 /* // BUTTON PINS
  pinMode(BUTT_1, INPUT_PULLUP);
  pinMode(BUTT_2, INPUT_PULLUP); */
  // LED PINS
  pinMode(LED_1, OUTPUT); 
  pinMode(LED_2, OUTPUT); 
  // BLE SETUP
  BLEDevice::init("ARES_MINI_SUMO_ROBOT");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  // IR SETUP
  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
  Serial.println("System Ready.");

  /*while (m == 0){
   if (digitalRead(BUTT_1) == LOW ){ m = 1;currentState = ATTACK;} 
   else if (digitalRead(BUTT_2) == LOW) {m = 2;currentState = SEARCH;} 
   delay(10);   
  }*/

  /*    bool started = false; 
    while (!started){
    if(IrReceiver.decode()) {
    /*Warning: Our remote control is old and some times it sends more than a signal code for each button. 
    For instance, for the button "1", during some test we did, it gave us the following codes: 0x1041 & 0x841.
    Therefore, we use what we call "bitwise AND" with specific hexademical number to ignore the "toggle" bit.
    */
  /*code = IrReceiver.decodedIRData.decodedRawData; //& 0xF7FF;
    debugMsg("PROTOCOL: " + String(IrReceiver.getProtocolString()));
    debugMsg("COMMAND (HEX): 0x" + String(code, HEX)); 
      if (code == 0xfb040707) { // Αν πατηθεί το "1"
        //m = 1; 
        currentState = ATTACK;
        started = true;
        debugMsg("Starting: ATTACK MODE");
      } 
      else if (code == 0xfa050707) { // Αν πατηθεί το "2"
       // m = 2; 
        currentState = STOP;
        aosComplete = false; // Ενεργοποίηση AOS ρουτίνας
        started = true;
        debugMsg("Starting: STOP MODE");
      }
      IrReceiver.resume();
    }
    delay(10);
    }


  /*if (m == 1) startRoutine(); 
  else if (m == 2) aosComplete = false;*/
  digitalWrite(LED_1, HIGH); 
}


//---Loop Function.Here the code runs forever---
void loop() {  
  //Reading the color of the arena continiously 
  int line_L = analogRead(LINE_L);
  int line_R = analogRead(LINE_R);
  //Message construction for BLE
  String data = "L: " + String(line_L) + "| R: " + String(line_R);
  //Time to decode the signals we receive from the RC5 remote control
  if(IrReceiver.decode()) {
    /*Warning: Our remote control is old and some times it sends more than a signal code for each button. 
    For instance, for the button "1", during some test we did, it gave us the following codes: 0x1041 & 0x841.
    Therefore, we use what we call "bitwise AND" with specific hexademical number to ignore the "toggle" bit.
    */
    code = IrReceiver.decodedIRData.decodedRawData; //& 0xF7FF;
    debugMsg("PROTOCOL: " + String(IrReceiver.getProtocolString()));
    debugMsg("COMMAND (HEX): 0x" + String(code, HEX)); 
      if (code == 0xfb040707) { // Αν πατηθεί το "1"
        //m = 1; 
        digitalWrite(LED_1, LOW); 
        digitalWrite(LED_2, HIGH); 
        currentState = ATTACK;
        //started = true;
        debugMsg("Starting: ATTACK MODE");
      } 
      else if (code == 0xfa050707) { // Αν πατηθεί το "2"
       // m = 2; 
        currentState = STOP;
        digitalWrite(LED_1, HIGH); 
        digitalWrite(LED_2, LOW); 
        aosComplete = false; // Ενεργοποίηση AOS ρουτίνας
       // started = true;
        debugMsg("Starting: STOP MODE");
      }
    //We change the state of the robot based on the code we get from the remote control.
    if(code == 0xfb040707) currentState = ATTACK;
    //if(code == 0x1042 ) currentState = SEARCH;
    if(code == 0xa050707) currentState = STOP;
    IrReceiver.resume();
  }
  //Now, based on the state, the robot behaves how we want to behave.
  switch(currentState){
    case ATTACK:

      if(edgeActive == true) {
        foundEdge(searchDir);
        //motorStop();
        debugMsg("Edge Detected!");
        debugMsg(data);
        return;
      }
      if(line_L < 3100){
        edgeActive=true;
        debugMsg("Left Edge detected!");
        searchDir = LEFT; //Toggle direction
      }
      else if (line_R < 3700) {
        edgeActive = true;
        debugMsg("Right Edge detected!");
        searchDir = RIGHT; //Toggle direction
      }
     else if (digitalRead(PIN_OPP_FL) == LOW || digitalRead(PIN_OPP_FR) == LOW || digitalRead(PIN_OPP_FC) == LOW) {
             fuzzyAttack(line_L, line_R); 
              debugMsg("ATTACK");

     }else if(digitalRead(PIN_OPP_L) == LOW ) {
      motorRight(200);
      motorLeft(-200);
      debugMsg("LEFT DETECTED");
      delay(175);
    } else if(digitalRead(PIN_OPP_R) == LOW) {
      motorRight(-200);  
      motorLeft(200);
      debugMsg("RIGHT DETECTED"); 
      delay(175);   
    } else {
      search(line_L, line_R,searchDir);
      debugMsg("SEARCH 2.0");
 
    }
      break;
    /*case SEARCH:
      if(edgeActive == true) {
        foundEdge(searchDir);
        //motorStop();
        debugMsg("Edge Detected!");
        debugMsg(data);
        return;
      }
      if(line_L < 3900){
        edgeActive = true;
        debugMsg("Left Edge detected!");
        searchDir = LEFT; //Toggle direction
      }
      else if (line_R < 3900) {
        edgeActive = true;
        debugMsg("Right Edge detected!");
        searchDir = RIGHT; //Toggle direction
      }

      if (!aosComplete && m == 2){
        AOS();
      }
      else {
        search(line_L, line_R, searchDir);
        debugMsg("ARES is searching!");
      }
      break;*/
    case STOP:

      motorStop();
      break;

  }
  delay(25); //We may have to completely remove it and use the millis() function
}
