/*----------------------------------------VARIABLES----------------------------------------*/
//Global Constants
const int AlanConstant = 1000;                                                  //Used for scaling all the normalized fractions to a standardized range in honor of Alan
const int SwiniScale = 255;                                                     //Used for defining a scale for motors speeds and for ease of PWM in honor of Swini
const int KarthiksPathLength = 256;                                             //Capacity of the path array
const int ManyasMilliseconds=50;                                                //Delay for confirming the junction
const int debounceDelay = 200;                                                  //Standard duration for debounce
const int normalizedThreshold = AlanConstant/2;                                 //The threshold value for the normalized range which determines the difference between the existence of a line and the contrary.
const int weights[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};      //Pre-assigned weights for calculating the weighted mean
const int calibTimeLimit = 3000;                                                //Maximum time it should calibrate for in ms
const int blindTurnTime = 150;                                                  //Time for which the bot blindly turns to get off the line
const int uTurnBlindTime = 300;                                                 //Time for which the bot blindry turns at dead ends

//Pin definitions
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};                     //Pre-defines the pins from which the arduino reads/the ones connected to IR sensor
const int endLED = 12;                                                          //Pre-defines the pin for the LED that is on at the end
const int calibLED = 11;                                                        //Pre-defines the pin for the LED that is on at the time of calibration
const int idleLED = LED_BUILTIN;                                                //Pre-defines the pin for the LED on during idle mode
const int calibrationButtonPin = 2;                                             //Pre-defines the pin connected to the calibration button, Note that D2 is a digital pin with interrupt capability
const int dryRunButtonPin = 3;                                                  //Pre-defines the pin connected to the dry run button, Note that D3 is a digital pin with interrupt capability
const int actualRunButtonPin = 4;                                               //Pre-defines the pin connected to the actual run button
const int PWMA = 10;                                                            //Pre-defines the pin for PWM of Motor A (The left motor)
const int AIN2 = 9;                                                             //Pre-defines the pin for AIN2 of the left motor
const int AIN1 = 8;                                                             //Pre-defines the pin for AIN1 of the left motor
const int BIN1 = 7;                                                             //Pre-defines the pin for BIN1 of the right motor
const int BIN2 = 6;                                                             //Pre-defines the pin for BIN2 of the right motor
const int PWMB = 5;                                                             //Pre-defines the pin for PWM of Motor B (The right motor)

//Sensor and calibration variables
bool polarity;                                                                  //0 means black tape on white bg(darker tape), 1 means white tape on black bg(lighter tape), actual colors could be anything
int sensorRaw[8] = {0, 0, 0, 0, 0, 0, 0, 0};                                    //Direct readings, Analog
int sensorNormalized[8] = {0, 0, 0, 0, 0, 0, 0, 0};                             //Normalized form of direct readings
int sensorMax[8] = {0, 0, 0, 0, 0, 0, 0, 0};                                    //Max value each sensor reads (defines limits)
int sensorMin[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};            //Min value each sensor reads (defines limits)
bool sensorBool[8] = {false, false, false, false, false, false, false, false};  //It is 0 or 1 based on whether the bot detects a line or not
int activeSensorCount = 0;                                                      //How many sensors detect a line
bool lineDetected = false;                                                      //true if atleast one sensor sees the line
bool isCalibrated = false;                                                      //Whether the auto-calibration is done or not

//PID Variables
float errorP;                                                                   //current error
float errorI;                                                                   //accumulated error
float errorD;                                                                   //derivative term for error
float prevError;                                                                //error in the last loop
float Kp;                                                                       //Proportional Gain
float Ki;                                                                       //Integral Gain
float Kd;                                                                       //Derivative Gain
float correction;                                                               //PID output of correction to be made in motor speed
int weightedLinePosition = 0;                                                   //Gives the weighted number which determines where the line is
int lastPosition;                                                               //Incase the bot gets lost

//Motor Control Variables
const int maxSpeed = SwiniScale;                                                //Upper speed limit, full power to motor
const int minSpeed = 0;                                                         //Lower speed limit, no power to motor
int baseSpeed;                                                                  //This will be set as dryBaseSpeed or actualBaseSpeed but used directly at operation time
int dryBaseSpeed;                                                               //Base speed for dry run (slower and more stable)
int actualBaseSpeed;                                                            //Base speed for actual run (can be faster)
int leftSpeed;                                                                  //speed of left motor during adjustment
int rightSpeed;                                                                 //speed of right motor during adjustment, difference in speed causes rotation
int turnDuration;                                                               //how long to turn for
char currentTurn = '\0';                                                        //The turn being executed right now

//Path Logic Variables
char path[256];                                                                 //Stores the decisions made by the robot at every junction (FLRB)
int pathIndex = 0;                                                                  //pseudo pointer to track index
int pathExecIndex = 0;                                                              //pointer in actual run mode
bool junctions[3] = {false, false, false};                                      //junctions[0] is left, junctions[1] is forward, junctions[2] is right

//Mode and State variables
//NOTE : In calibration mode, the general state of the robot is irrelevant
enum Mode{idleMode, calibMode, dryRunMode, actualRunMode};                      //Definition of modes
Mode mode = idleMode;                                                           //Variable to store the mode
enum State{idleState, followingLine, atJunction, turning, atEnd};               //Definition of general states
State state = idleState;                                                        //Variable to store the state
enum CalibState{calibIdle, calibStart, calibSampling, calibEnd};                //Definition of calibration states
CalibState calibState = calibIdle;                                              //Variable to store the calibration state
bool turningComplete;                                                           //tells if the turn is completed
bool junctionDetected = false;                                                  //Raw detection
bool junctionConfirmed = false;                                                 //Confirmed detection
bool allWhite;                                                                  //Sensors see all white
bool endConfirmed = false;                                                      //Confirmed end detection
bool endLEDState = false;                                                       //Whether the end LED is on or not
bool calibLEDState = false;                                                     //Whether the calibration LED is on or not
bool lineLost = false;                                                          //Whether the line was lost (does not become true on reaching end)

//Time Variables, unit : ms
unsigned long currentTime;                                                      //stores current time, used as reference, called by millis()
unsigned long lastTime;                                                         //reusable timestamp for the last event that occured
unsigned long lastCalibTime;                                                    //stores the last time calibration started
unsigned long whiteStartTime;                                                   //starts the timer when sensors go all white to detect end
unsigned long turnStartTime;                                                    //stores the timestamp of when it starts turning
unsigned long junctionStartTime;                                                //stores the timestamp of when the junction was seen
unsigned long dryRunButtonDebounce;                                             //Acts as a debounce for the dry run button
unsigned long actualRunButtonDebounce;                                          //Acts as a debounce for the actual run button
unsigned long calibrationButtonDebounce;                                        //Acts as a debounce for the calibration button

/*----------------------------------------FUNCTIONS----------------------------------------*/
//Function Declarations--------------------------------------------------------------------//
void checkButtons();                                                            //Checks and modifies modes based on button interactions. Calib Button acts as a calib button if it is in idle mode or it acts as a kill/halt switch if it is in other modes
void readSensors();                                                             //Reads input from the sensors
void updateMinMax();                                                            //Updates raw min and max values from the sensor
void normalizeReadings();                                                       //Normalizes the readings to the Alan scale and also creates a standardized value of higher for line and lower for background based on polarity detected
void updateSensorBool();                                                        //Updates the sensorBool array values
void startCalibration();                                                        //Starts calibration process
void runCalibration();                                                          //Actual calibration process
void endCalibration();                                                          //Ends the calibration process
void calculateLinePosition();                                                   //Calculates the position of the line using weights
void calculatePID();                                                            //Calculates all the errors and the correction in PID
void setMotorSpeeds(int leftSpeed, int rightSpeed);                             //Standard function for setting the motor speeds
void moveForward(int motorSpeed);                                               //Sets motor speeds to move forward
void moveBackward(int motorSpeed);                                              //Sets motor speeds to move backward
void pivotLeft(int motorSpeed);                                                 //Pivots the bot left, Ross' favorite function designed for sharp turns
void pivotRight(int motorSpeed);                                                //Pivots the bot right, Ross' other favorite function designed for sharp turns
void turnLeft(int motorSpeed);                                                  //Turns the bot to the left, designed for smooth turns
void turnRight(int motorSpeed);                                                 //Turns the bot to the right, designed for smooth turns
void stopMotors();                                                              //Stops the movement of motors  
void applyPIDCorrection(int motorSpeed, float correction)                       //Applies the correction from PID function to the motor speed
void checkAvailablePaths();                                                     //Checks which paths are available here
void detectJunction();                                                          //Detects if there is a junction
void confirmJunction();                                                         //Confirms if there is a junction
void makeTurnDecision();                                                        //Decides what the current turn should be
void executeTurn();                                                             //Executes the turn decision
void optimizePath();                                                            //One of the most important functions we have made to reduce the path based on recorded moves


//Button Functions-------------------------------------------------------------------------//

void checkButtons(){
  if(digitalRead(calibrationButtonPin)==LOW){
    if(currentTime-calibrationButtonDebounce>debounceDelay){
      calibrationButtonDebounce = currentTime;
      if(mode==idleMode) startCalibration();
      else{
        mode = idleMode;
        state = idleState;
        stopMotors();
        digitalWrite(endLED, LOW);
        endLEDState = false;
        calibLEDState = false;
        junctionDetected = false;
        junctionConfirmed = false;
        endConfirmed = false;
        turningComplete = false;
        currentTurn = '\0';
      }
    }
  }else if(digitalRead(dryRunButtonPin)==LOW){
    if(currentTime-dryRunButtonDebounce>debounceDelay){
      dryRunButtonDebounce = currentTime;
      if(mode==idleMode && isCalibrated){
        mode = dryRunMode;
        state = followingLine;
        pathIndex = 0;
        baseSpeed = dryBaseSpeed;
        lineLost = false;

        digitalWrite(idleLED, LOW);
        digitalWrite(endLED, LOW);
        endLEDState = false;
      }
    }
  }else if(digitalRead(actualRunButtonPin)==LOW){
    if(currentTime-actualRunButtonDebounce>debounceDelay){
      actualRunButtonDebounce = currentTime;
      if(mode==idleMode && isCalibrated && pathIndex>0){
        mode = actualRunMode;
        state = followingLine;
        pathExecIndex = 0;
        baseSpeed = actualBaseSpeed;
        lineLost = false;
        digitalWrite(idleLED, LOW);
        digitalWrite(endLED, LOW);
        endLEDState = false;
      }
    }
  }
}

//Sensor and Calibration Functions---------------------------------------------------------//
void startCalibration(){
  mode = calibMode;
  calibState = calibStart;
  calibLEDState = true;
  digitalWrite(calibLED, HIGH);
  digitalWrite(idleLED, LOW);
  currentTime = millis();
  lastCalibTime = currentTime;
  isCalibrated = false;
  for(int i = 0; i<8; i++){
    sensorMax[i] = 0;
    sensorMin[i] = 1023;
    sensorRaw[i] = 0;
    sensorNormalized[i] = 0;
    sensorBool[i] = false;
  }
  weightedLinePosition = 0;
  activeSensorCount = 0;
  lineDetected = false;
  endLEDState = false;
  polarity = false;
  stopMotors();
  calibState = calibSampling;
}

//This function will be called repeatedly by loop()
void runCalibration(){
    if(calibState==calibSampling){
      readSensors();
      updateMinMax();
      if(millis()-lastCalibTime>=calibTimeLimit){
        long int centreSum = 0, edgeSum = 0;
        for(int i = 2; i<6; i++) centreSum += sensorMin[i]+(sensorMax[i]-sensorMin[i])/2;
        for(int i = 0; i<2; i++) edgeSum += sensorMin[i] + (sensorMax[i]-sensorMin[i])/2;
        for(int i = 6; i<8; i++) edgeSum += sensorMin[i] + (sensorMax[i]-sensorMin[i])/2;
        polarity = centreSum>edgeSum?1:0;
        calibState = calibEnd;
      }
    }
    if(calibState==calibEnd) endCalibration();
}

void endCalibration(){
  isCalibrated = true;
  calibState = calibIdle;
  mode = idleMode;
  digitalWrite(calibrationLED, LOW);
  digitalWrite(idleLED, HIGH);
}

void readSensors(){
  for(int i = 0; i<8; i++) sensorRaw[i] = analogRead(sensorPins[i]);
}

void updateMinMax(){
  for(int i = 0; i<8; i++){
    if(sensorRaw[i]<sensorMin[i]) sensorMin[i] = sensorRaw[i];
    if(sensorRaw[i]>sensorMax[i]) sensorMax[i] = sensorRaw[i];
  }
}

void normalizeReadings(){
  for(int i = 0; i<8; i++){
    if(sensorMax[i]-sensorMin[i]==0) sensorNormalized[i] = 0;
    else{
      sensorNormalized[i] = ((long)(sensorRaw[i]-sensorMin[i])*AlanConstant)/(sensorMax[i]-sensorMin[i]); 
      sensorNormalized[i] = constrain(sensorNormalized[i], 0, AlanConstant);
      sensorNormalized[i] = polarity==0?AlanConstant-sensorNormalized[i]:sensorNormalized[i];
    }
  }
}

void updateSensorBool(){
  activeSensorCount = 0;
  lineDetected = false;
  for(int i = 0; i<8; i++){
    sensorBool[i] = sensorNormalized[i]>normalizedThreshold;
    if(sensorBool[i]){
      activeSensorCount++;
      lineDetected = true;
    }
    
  }
}

//PID Functions----------------------------------------------------------------------------//
void calculateLinePosition(){
  long int numeratorSum = 0;
  long int denominatorSum = 0;
  for(int i = 0; i<8; i++){
    numeratorSum += (long)sensorNormalized[i]*weights[i];
    denominatorSum += sensorNormalized[i];
  }
  if(lineDetected){
    weightedLinePosition = numeratorSum/denominatorSum;
    lastPosition = weightedLinePosition;
  }else{
    if(lastPosition>0){
      weightedLinePosition = 3500;
    }else{
      weightedLinePosition = -3500;
    }
  }
}

void calculatePID(){
  errorP = weightedLinePosition;
  errorI += errorP;                
  errorD = errorP - prevError;
  errorI = constrain(errorI, -10000, 10000);
  correction = (Kp * errorP) + (Ki * errorI) + (Kd * errorD);
  prevError = errorP;
}

//Path Functions-------------------------------------------------------------------------//
void checkAvailablePaths(){
  junctions[0] = (sensorBool[0]&&sensorBool[1]);
  junctions[1] = (sensorBool[3] || sensorBool[4]);
  junctions[2] = (sensorBool[6]&&sensorBool[7]);//Leaving 2 and 5 as they mostly pick up randomly since they are near the edge
}

void detectJunction(){
  if(junctionDetected || state==atJunction || state == turning || state == atEnd) return;
  junctionDetected = ((sensorBool[0]&&sensorBool[1]) || (sensorBool[6]&&sensorBool[7]) || activeSensorCount>=7);
  if(junctionDetected) junctionStartTime = currentTime;
}

void verifyJunction(){
  if(junctionDetected && !junctionConfirmed && (currentTime - junctionStartTime > ManyasMilliseconds)){
    checkAvailablePaths();
    if(activeSensorCount>=7){
      endConfirmed = junctionConfirmed = true;
      state = atEnd;
    }else if(junctions[0] || junctions[2]){
      junctionConfirmed = true;
      state = atJunction;  
    }else{
      junctionDetected = false;  
    }
  }
}

void makeTurnDecision(){
  if(mode==dryRunMode){
    if(junctions[0]){
     currentTurn = 'L';
    }else if(junctions[1]){
      currentTurn = 'F';
    }else if(junctions[2]){
      currentTurn = 'R';
    }else{
      currentTurn = 'B';
    }
    if(pathIndex<KarthiksPathLength) path[pathIndex++] = currentTurn;
  }else if(mode==actualRunMode){
    if(pathExecIndex<pathIndex){
      currentTurn = path[pathExecIndex++];
    }else{
      currentTurn = 'S';
    }
  }
  
  state = turning;
  turnStartTime = currentTime;
  turningComplete = false;  
}

void executeTurn(){
  if(turningComplete){
    stopMotors();
    state = followingLine;
    turningComplete = false;
    junctionDetected = false;
    junctionConfirmed = false;
    return;
  }

  switch(currentTurn){
    case 'L':
      pivotLeft(baseSpeed);
      if(currentTime-turnStartTime>blindTurnTime && (sensorBool[3] || sensorBool[4])) turningComplete = true;
      break;
    case 'R':
      pivotRight(baseSpeed);
      if(currentTime-turnStartTime>blindTurnTime && (sensorBool[3] || sensorBool[4])) turningComplete = false;
      break;
    case 'B':
      pivotRight(baseSpeed);
      if(currentTime-turnStartTime>uTurnBlindTime && (sensorBool[3] || sensorBool[4])) turningComplete = true;
      break;
    case 'F':
      moveForward(baseSpeed);
      if(currentTime-turnStartTime>blindTurnTime) turningComplete = true;
      break;
    default:
      stopMotors();
      break;
  }
}



//Motor Functions------------------------------------------------------------------------//
void setMotorSpeeds(int leftSpeed, int rightSpeed){
  leftSpeed = constrain(leftSpeed, -SwiniScale, SwiniScale);
  rightSpeed = constrain(rightSpeed, -SwiniScale, SwiniScale);

  if(!leftSpeed){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }else if(leftSpeed>0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }else{
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }

  if(!rightSpeed){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }else if(rightSpeed>0){
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }else{
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }

  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
}

//NOTE : In the below functions motorSpeed has to strictly be the magnitude (only positive values)
void moveForward(int motorSpeed){setMotorSpeeds(motorSpeed, motorSpeed);}
void moveBackward(int motorSpeed){setMotorSpeeds(-motorSpeed, -motorSpeed);}
void pivotLeft(int motorSpeed){setMotorSpeeds(-motorSpeed, motorSpeed);}
void pivotRight(int motorSpeed){setMotorSpeeds(motorSpeed, -motorSpeed);}
void turnLeft(int motorSpeed){setMotorSpeeds(0, motorSpeed);}
void turnRight(int motorSpeed){setMotorSpeeds(motorSpeed, 0);}
void stopMotors(){setMotorSpeeds(0, 0);}

void applyPIDCorrection(int baseSpeed, float correction){
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;
  setMotorSpeeds(leftSpeed, rightSpeed);
}



//STANDARD FUNCTIONS---------------------------------------------------------------------//
void setup() {
  // put your setup code here, to run once:
  pinMode(endLED, OUTPUT);
  pinMode(calibLED, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(actualRunButtonPin, INPUT_PULLUP);
  pinMode(dryRunButtonPin, INPUT_PULLUP);
  pinMode(calibrationButtonPin, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();
}
