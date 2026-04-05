/*
	Name: Ares Mini Sumo Robot
	Copyright: ©UoP Robotics Team (https://github.com/UoP-Robotics)
	Author: Katerina Tertimpa (https://github.com/katerina-kt), Georgios Pegiazis (https://github.com/GeorgePeg)
	Date: 18/03/26 16:11
  Version: 1.1.0
  License: GNU General Public License v3.0, 29 June 2007
  Description: Latest version of Ares for the robotic competition Robotic Arena.
  Application for debugging: https://play.google.com/store/search?q=ble+controller+%E2%80%93+arduino+esp32&c=apps
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- BLE CONFIG ---
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// LEFT MOTOR 
#define L_IN1 45  // MOVING FORWARD
#define L_IN2  48 // MOVING BACKWARD
#define PWM_L  47// MOTOR SPEED

// RIGHT MOTOR 
#define R_IN1 46  // MOVING BACKWARD
#define R_IN2  3 // MOVING FORWARD
#define PWM_R   9// MOTOR SPEED 

// STANDBY
const int  STBY = 12;

// EDGE SENSOR THRESHOLD 
//#define EDGE_L_THRESHOLD  526 /*Number*/
//#define EDGE_R_THRESHOLD  256 /*Number*/

// DEFINE FOR DIRECTIONS 
#define LEFT 0
#define RIGHT 1
int searchDir = LEFT;
bool edgeActive = false ;

// LINE SENSORS  (ANALOG INPUTS) 
#define LINE_L 17 //
#define LINE_R 12 //

// OPP SENSORS  (DIGITAL INPUTS )
#define PIN_OPP_L  16
#define PIN_OPP_FL 15
#define PIN_OPP_FC 7
#define PIN_OPP_FR 10
#define PIN_OPP_R 11

// START BUTTON
//#define START 

// PID PARAMETERS   
float Kp = 1.5;     /* The numbers may need change  */
float Ki = 0.0;
float Kd = 0.03;

float pidError = 0;
float pidLastError = 0;
float pidIntegral = 0;

// --- BLE CALLBACKS ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { 
      deviceConnected = false;
      pServer->getAdvertising()->start(); 
    }
};

// Function that sends a message to both the Bluetooth and Serial
void debugMsg(String msg) {
  Serial.println(msg);
  if (deviceConnected) {
    pTxCharacteristic->setValue(msg.c_str());
    pTxCharacteristic->notify();
  }
}


void startRoutine(){

   unsigned long edgeStart = 0;
   int edgeState = 0;

  while (edgeState < 3){

 switch(edgeState){
    //TURN 45 DEGREES
    case 0 :
    edgeStart = millis();
    motorRight(90);
    motorLeft(0);
    edgeState = 1;  // MOVE TO NEXT CASE 
    break;

    case 1:    /* ΑΛΛΑΓΕΣ ΓΙΑ ΑΝΑ ΔΩ ΑΜΑ ΟΝΤΩΣ ΕΙΝΑΙ 180 */
    if (millis() - edgeStart >= 600){
      edgeStart = millis();
      // MOVE FORWARD
      motorRight(-90);
      motorLeft(-90);
      edgeState = 2 ;    // MOVE TO NEXT CASE
    }
    break;

    case 2:
     if ( millis() - edgeStart >= 1000){
        motorStop();      
        edgeStart = millis();
        edgeState = 3;   // MOVE THE NEXT CASE 
     } break;

    // EXIT CASE
    default: 
      break; 
    }
    delay(100);
  }
 }

// LEFT MOTOR ENGAGEMENT ACCORDING TO SPEED 
void motorRight(int speed){
  if (speed >= 0 ){
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);  
    speed = -speed; 
  }
    analogWrite(PWM_L, constrain(speed, 0, 255));
}

// RIGHT MOTOR ENGAGEMENT ACCORDING TO SPEED 
void motorLeft(int speed){
  if (speed >= 0 ){
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);  
    speed = -speed; 
  }
analogWrite(PWM_R, constrain(speed, 0, 255));
}

void motorStop(){
  // STOP ALL CONNECTION WITH THE PCB
  analogWrite(PWM_L, 0); 
  analogWrite(PWM_R, 0);  
}


void foundEdge(int dir){ 
  static unsigned long edgeStart = 0;
  static int edgeState = 0;

 switch(edgeState){
    //REVERSE
    case 0 :
    edgeStart = millis();
    motorRight(60);
    motorLeft(60);
    edgeState = 1;  // MOVE TO NEXT CASE 
    break;

    case 1:    // ΑΛΛΑΓΕΣ ΓΙΑ ΑΝΑ ΔΩ ΑΜΑ ΟΝΤΩΣ ΕΙΝΑΙ 180
    if (millis() - edgeStart >= 600) {
      edgeStart = millis();
       //ROTATE                         
      if (dir == LEFT){
       // ROTATE LEFT BACKWARDS
       motorRight(-60); 
       motorLeft(60);  
      } else {
       // ROTATE RIGHT BACKWARDS 
       motorRight(60); 
       motorLeft(-60); 
      }
      edgeState = 2 ;    // MOVE TO NEXT CASE
    }
    break;

    case 2:
     if ( millis() - edgeStart >= 1000){
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


int PID(int error) {
   pidError = (float)error;
   pidIntegral += pidError;
   float derivative = pidError - pidLastError;

   float output = Kp * pidError + Ki * pidIntegral + Kd * derivative;

   // SPEED LIMITATION FOR SMOTHER MOVEMENT
   if (output > 100) output = 100;       /*Possible change*/
   if (output < -100) output = -100;

   pidLastError = pidError;
   return (int)output;
}

void search(int line_L, int line_R, int dir){
  // MOVE IN A CIRCULAR MOTION  
  if (dir == LEFT){
   motorRight(-60); 
   motorLeft(-100);
  } else {
   motorRight(-60); 
   motorLeft(-100);
  }
  debugMsg("SEARCH DIR: " + String(dir));
}

void fuzzyAttack(int line_L, int line_R) {
  // MAP OPPONENT SENSORS TO FUZZY DIRECTION
  int fuzzyL  =  (digitalRead(PIN_OPP_L)  == LOW) ? 150 : 0;
  int fuzzyFL =  (digitalRead(PIN_OPP_FL)  == LOW) ? 60: 0;
  int fuzzyFR =  (digitalRead(PIN_OPP_FR)  == LOW) ? 60 : 0;
  int fuzzyR  =  (digitalRead(PIN_OPP_R)  == LOW) ? 150 : 0;

  // OPPONENT DIRECTION (-100 LEFT, +100 RIGHT)
  int opponentDir = (fuzzyFR + fuzzyR) - (fuzzyFL + fuzzyL);

  int correction = PID(opponentDir);
debugMsg("CORRECTION: " + String(correction));

  int baseSpeed = (digitalRead(PIN_OPP_FC) == LOW) ? -255 : -180;// BASE FORWARD SPEED (Possible change)
  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // CLAMP SPEEDS
  leftSpeed  = constrain(leftSpeed, -200, 255);
  rightSpeed = constrain(rightSpeed, -200, 255);

  motorLeft(leftSpeed);
  motorRight(rightSpeed);
} 

void setup() {
  Serial.begin(115200);
  // MOTOR PINS 
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  pinMode(STBY,  OUTPUT);  
  digitalWrite(STBY, HIGH);
 

  // SENSOR PINS
  pinMode(LINE_L, INPUT_PULLUP);
  pinMode(LINE_R, INPUT_PULLUP);
  pinMode(PIN_OPP_L,  INPUT_PULLUP);
  pinMode(PIN_OPP_FL, INPUT_PULLUP);
  pinMode(PIN_OPP_FC, INPUT_PULLUP); 
  pinMode(PIN_OPP_FR, INPUT_PULLUP);  
  pinMode(PIN_OPP_R,  INPUT_PULLUP); 
//  pinMode(START,  INPUT_PULLUP); /*KEEPS THE SIGNAL HIGH UNLESS PULLED LOW */

  // BLE SETUP
  BLEDevice::init("S3_SUMO_DEBUG");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();

  Serial.println("System Ready. Connect via Bluetooth...");

 // START BUTTON PRESSED AND RELEASED
 // while (digitalRead(START));
  //while (!digitalRead(START));
   startRoutine();
  
}

void loop() {


  
   if (edgeActive == true){
    foundEdge(searchDir);
    return;
   }

  // CONTINUOUSLY READ FOR THE LINE SENSORS
  int line_L = analogRead(LINE_L);
  int line_R = analogRead(LINE_R); 

  if(line_L < 3500){
    edgeActive = true;
    debugMsg("LINE LEFT DETECTED");
    // TOGGLE THE DIRECTION
    searchDir = LEFT; 
    return;

  } else if (line_R < 3500){
    edgeActive = true;
    debugMsg("LINE RIGHT DETECTED");
    // TOGGLE THE DIRECTION
    searchDir = RIGHT; 
    return;

  } 
  if (digitalRead(PIN_OPP_L) == LOW || digitalRead(PIN_OPP_R) == LOW || 
             digitalRead(PIN_OPP_FL) == LOW || digitalRead(PIN_OPP_FR) == LOW || digitalRead(PIN_OPP_FC) == LOW) {
             fuzzyAttack(line_L, line_R); 

  } else {
    search(line_L, line_R,searchDir); 
  }

}
