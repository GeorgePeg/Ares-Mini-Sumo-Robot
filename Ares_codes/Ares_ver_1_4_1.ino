/*
	Name: Ares Mini Sumo Robot
	Copyright: ©UoP Robotics Team (https://github.com/UoP-Robotics)
	Authors: Katerina Tertimpa, Georgios Pegiazis (https://github.com/GeorgePeg)
	Date: 08/05/26 
 	Version: 1.4.1
  	License: GNU General Public License v3.0, 29 June 2007
  	Description: Latest version of Ares for the robotic competition Robotic Arena with remote control using RC5 protocol with a LED turning
  	on & off for debugging.
  	Application for debugging: https://play.google.com/store/search?q=ble+controller+%E2%80%93+arduino+esp32&c=apps
*/
// DEFINE FOR DIRECTIONS 
#define LEFT 0
#define RIGHT 1
int searchDir = LEFT;
bool edgeActive = false ;
float k = 1.36; //-> Smoothing factor

//bool started = false; 

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
#define RED_LED  21
#define YELLOW_LED  14
#define GREEN_LED  13

// PID PARAMETERS   
float Kp = 1.75;     /*Numbers may have to change through testing*/
float Ki = 0.27;
float Kd = 0.01;

float pidError = 0;
float pidLastError = 0;
float pidIntegral = 0;

//IR PIN
const int start_pin = 42; 
//int code = 0;

enum RobotState {
  STOP, ATTACK, START
}; 
//Initial State
RobotState currentState = STOP;

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
  //debugMsg("SEARCH DIR: " + String(dir));
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
/*
void AOS(){
  static unsigned long aosStart = 0;
  static int aosStep = 0;
  bool aosFinished = false ;

  while (!aosFinished ){
  switch(aosStep) {
    case 0: // Ξεκίνα το χρόνο
      aosStart = millis();
      motorStop();
      aosStep = 1;
      //debugMsg
      Serial.println("AOS: Waiting...");
      break;

    case 1: // Περίμενε 1.5 δευτερόλεπτο
      if (millis() - aosStart >= 1500) {
        aosStart = millis();
        motorLeft(175);
        motorRight(175);
        aosStep = 2;
        //debugMsg
        Serial.println("AOS: Moving!");
      }
      break;

    case 2: // Κινήσου για 500ms και μετά σταμάτα
      if (millis() - aosStart >= 500) {
        motorStop();
        aosFinished = true; // ΑΥΤΟ ΕΙΝΑΙ ΤΟ ΚΛΕΙΔΙ για να βγει από το while
       // currentState = SEARCH; // Γύρνα αυτόματα σε SEARCH mode
        //debugMsg
        Serial.println("AOS Done -> Searching");
      }
      break;
    } 
  }
}
*/
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
  //int fuzzyL  =  (digitalRead(PIN_OPP_L)  == LOW) ;
  int fuzzyFL =  (digitalRead(PIN_OPP_FL)  == LOW) ;
  int fuzzyFR =  (digitalRead(PIN_OPP_FR)  == LOW) ;
  //int fuzzyR  =  (digitalRead(PIN_OPP_R)  == LOW) ;
  int fuzzyFC = (digitalRead(PIN_OPP_FC) == LOW);

  // OPPONENT DIRECTION (-100 LEFT, +100 RIGHT)
  int opponentDir = (fuzzyFR *60 /*+ fuzzyR*100*/) - (fuzzyFL*60 /*+ fuzzyL*100*/);
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
  // MOTOR PINS 
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);


  //pinMode(STBY,  OUTPUT);
  //pinMode(IR_PIN, INPUT);  
  //digitalWrite(STBY, HIGH);

   // LED PINS
  pinMode(RED_LED, OUTPUT); 
  pinMode(YELLOW_LED, OUTPUT); 
  pinMode(GREEN_LED, OUTPUT);  

  // SENSOR PINS
  pinMode(LINE_L, INPUT_PULLUP);
  pinMode(LINE_R, INPUT_PULLUP);
  pinMode(PIN_OPP_L,  INPUT_PULLUP);
  pinMode(PIN_OPP_FL, INPUT_PULLUP);
  pinMode(PIN_OPP_FC, INPUT_PULLUP); 
  pinMode(PIN_OPP_FR, INPUT_PULLUP);  
  pinMode(PIN_OPP_R,  INPUT_PULLUP); 

  pinMode(start_pin, INPUT);
    
  //!started
  motorStop(); 
  digitalWrite(RED_LED, HIGH );

  while (digitalRead(start_pin) == LOW ){
    delay(5);
    }
  digitalWrite(RED_LED, LOW); 
  digitalWrite(YELLOW_LED, HIGH); 

    
  /* 
  unsigned long startWait = millis();
  while(millis() - startWait < 5000) {
    digitalWrite(YELLOW_LED, HIGH);
    currentState = STOP;
    if (digitalRead(start_pin) == LOW){
      digitalWrite(YELLOW_LED, HIGH);
      while(digitalRead(start_pin) == LOW); // wait for the start to be pressed
      startWait = millis(); 
    } 
  }
  digitalWrite(YELLOW_LED, LOW); 

  AOS();*/ 
  currentState = START;
}

void loop() {
  //Reading the color of the arena continiously 
  int line_L = analogRead(LINE_L);
  int line_R = analogRead(LINE_R);

  if (digitalRead(start_pin == LOW)){
    currentState = STOP;
  }else if ( currentState == STOP && digitalRead(start_pin) == HIGH ){
    currentState = START;
  }

  switch(currentState){
    case START:
    digitalWrite(RED_LED, HIGH); 
    digitalWrite(YELLOW_LED, HIGH); 
    digitalWrite(GREEN_LED, HIGH); 
    if (line_L < 3800 || line_R < 3800 || digitalRead(PIN_OPP_FL) == LOW ||
    digitalRead(PIN_OPP_FR) == LOW || digitalRead(PIN_OPP_FC) == LOW ){
      currentState = ATTACK;
    }

    static unsigned long aosTimer = millis();
   // case 1: waiting ( 1.5 s)
      if (millis() - aosTimer < 1500) {
        motorStop();
      } 
      // case 2: movement  (500ms)
      else if (millis() - aosTimer < 2000) {
        motorLeft(175);
        motorRight(175);
      } 
      // case 3: finish AOS
      else {
        currentState = ATTACK;
      }
      break; 

    case ATTACK:
    digitalWrite(RED_LED, LOW); 
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, HIGH); 

    if(edgeActive == true) {
      foundEdge(searchDir);
      Serial.println("Edge Detected! 2.0");
      return;
    }
    if(line_L < 3800){
      edgeActive=true;
      Serial.println("Left Edge detected! 2.0");
      searchDir = RIGHT; //Toggle direction
    }
    else if (line_R < 3800) {
      edgeActive = true;
      Serial.println("Right Edge detected! 2.0");
      searchDir = LEFT; //Toggle direction
    }
    else if (digitalRead(PIN_OPP_FL) == LOW || digitalRead(PIN_OPP_FR) == LOW || digitalRead(PIN_OPP_FC) == LOW) {
      fuzzyAttack(line_L, line_R); 
      Serial.println("ATTACK 2.0");
    }
    else if(digitalRead(PIN_OPP_L) == LOW ) {
      motorRight(200);
      motorLeft(-200);
      Serial.println("LEFT SIDE DETECTED 2.0");
      delay(175);
    } 
    else if(digitalRead(PIN_OPP_R) == LOW) {
      motorRight(-200);  
      motorLeft(200);
      Serial.println("RIGHT SIDE DETECTED 2.0"); 
      delay(175);   
    } 
    else {
      search(line_L, line_R,searchDir);
      Serial.println("SEARCH 2.0");
    }
      /*motorLeft(250);
      motorRight(250);*/
      break;

      case STOP:
      digitalWrite(RED_LED, HIGH); 
      digitalWrite(GREEN_LED, LOW); 

      motorStop();
      break;
  }
  delay(25); //We may have to completely remove it and use the millis() function
}
