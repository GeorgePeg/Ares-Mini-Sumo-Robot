/*
  Name: Ares Mini Sumo Robot
  Copyright: ©UoP Robotics Team (https://github.com/esdaLabWro)
  Authors: Katerina Tertimpa (https://github.com/katerina-kt), Georgios Pegiazis (https://github.com/GeorgePeg), Ornela-Maria Qoshi (https://github.com/ornela-maria)
  Date: 30/03/26 19:20
  Version: 1.3
  License: GNU General Public License v3.0, 29 June 2007
  Description: Latest version of Ares for the robotic competition Robotic Arena. The robot waits to receive the codes from the referees' remote control through RC5 protocol. 
  Application for debugging: https://play.google.com/store/search?q=ble+controller+%E2%80%93+arduino+esp32&c=apps
*/

// DEFINE FOR DIRECTIONS 
#define LEFT 0
#define RIGHT 1
int searchDir = LEFT;
bool edgeActive = false ;
float k = 1.36; //-> Smoothing factor

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

// BUTTONS
#define BUTT_1 18
#define BUTT_2 36
int m ;
bool aosComplete = false;

// PID PARAMETERS   
float Kp = 1.75;     /*Numbers may have to change through testing*/
float Ki = 0.27;
float Kd = 0.01;

float pidError = 0;
float pidLastError = 0;
float pidIntegral = 0;

//IR PIN
const int start_pin = 42; 
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

//---Function to handle the left motor---
void motorLeft(int speed){
  if (speed >= 0 ){
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  } else {
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH); speed = -speed; 
  }
  analogWrite(PWM_L, constrain(speed, 0, 255));
}
//---Function to handle the right motor---
void motorRight(int speed){
  if (speed >= 0 ){
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
  } else {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH); speed = -speed; 
  }
  analogWrite(PWM_R, constrain(speed, 0, 255));
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

  if (aosComplete) return; // If it ends, don't return back

  switch(aosStep) {
    case 0: // Start the time
      aosStart = millis();
      motorStop();
      aosStep = 1;
      break;

    case 1: // Wait for 1.5 seconds
      if (millis() - aosStart >= 1500) {
        aosStart = millis();
        motorLeft(175);
        motorRight(175);
        aosStep = 2;
      }
      break;

    case 2:// Move for 500ms and then stop
      if (millis() - aosStart >= 500) {
        motorStop();
        aosComplete = true; // End of routine
        currentState = SEARCH;// Return to SEARCH mode
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
  //int fuzzyFC = (digitalRead(PIN_OPP_FC) == LOW);

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

} 

void setup() {
  Serial.begin(115200);
  //MOTOR PINS
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
  // BUTTON PINS
  pinMode(BUTT_1, INPUT_PULLUP);
  pinMode(BUTT_2, INPUT_PULLUP); 
  // IR SETUP
  Serial.println("System Ready.");

  while (m == 0){
   if (digitalRead(BUTT_1) == LOW ){ m = 1;currentState = ATTACK;} 
   else if (digitalRead(BUTT_2) == LOW) {m = 2;currentState = SEARCH;} 
   delay(10);   
  }
  
 // Waiting for the referee signal
  while (digitalRead(start_pin) == LOW) {
    motorStop(); //Wait until the signal is HIGH
    delay(5);
  }   

  // Wait for 5 seconds based on the regulations
  unsigned long startWait = millis();
  while(millis() - startWait < 5000) {
    motorStop();
    if (digitalRead(start_pin) == LOW) return; // If the referee cancels it
  }  
  if (m == 1) startRoutine(); 
  else if (m == 2) aosComplete = false;
}

void loop() {
    // Secure Referee: If the signal stops, the robot "dies" imediately
  if (digitalRead(start_pin) == LOW) {
    motorStop();
    return; // Return back to loop and do nothing
  }
  //Reading the color of the arena continiously 
  int line_L = analogRead(LINE_L);
  int line_R = analogRead(LINE_R);

      //Now, based on the state, the robot behaves how we want to behave.
      switch(currentState){
        case ATTACK:
        if(edgeActive == true) {
          foundEdge(searchDir);
          return;
        }
        if(line_L < 3900){
          edgeActive=true;
          searchDir = LEFT; //Toggle direction
          return;
        }
        else if (line_R < 3900) {
          edgeActive = true;
          searchDir = RIGHT; //Toggle direction
          return;
        }
        else if (digitalRead(PIN_OPP_FL) == LOW || digitalRead(PIN_OPP_FR) == LOW || digitalRead(PIN_OPP_FC) == LOW) {
               fuzzyAttack(line_L, line_R); 

        }else if(digitalRead(PIN_OPP_L) == LOW ) {
          motorRight(200);
          motorLeft(-200);
          delay(175);
        } else if(digitalRead(PIN_OPP_R) == LOW) {
          motorRight(-200);  
          motorLeft(200);
          delay(175);   
        } else {
          search(line_L, line_R,searchDir);
        }
        break;

        case SEARCH:
        if(edgeActive == true) {
          foundEdge(searchDir);
          return;
        }
        if(line_L < 3900){
          edgeActive = true;
          searchDir = LEFT; //Toggle direction
          return;
        } 
        else if (line_R < 3900) {
          edgeActive = true;
          searchDir = RIGHT; //Toggle direction
          return;
        }

        if (!aosComplete && m == 2){
          AOS();
        }
        else if (digitalRead(PIN_OPP_FL) == LOW || digitalRead(PIN_OPP_FR) == LOW || digitalRead(PIN_OPP_FC) == LOW) {
               fuzzyAttack(line_L, line_R); 

        }else if(digitalRead(PIN_OPP_L) == LOW ) {
          motorRight(200);
          motorLeft(-200);
          delay(175);
        } else if(digitalRead(PIN_OPP_R) == LOW) {
          motorRight(-200);  
          motorLeft(200);
          delay(175);   
        } else {
          search(line_L, line_R,searchDir);
        }
        break;

        case STOP:
        motorStop();
        break;
      }
}