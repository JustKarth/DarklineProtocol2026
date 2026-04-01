/*----------------------------------------VARIABLES----------------------------------------*/
//Global Constants
const int AlanConstant = 1000;                                                  //Used for scaling all the normalized fractions to a standardized range in honor of Alan
const int normalizedThreshold = AlanConstant/2;                                 //The threshold value for the normalized range which determines the difference between the existence of a line and the contrary.
const int weights[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};      //Pre-assigned weights for calculating the weighted mean

//Pin definitions
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};                     //Pre-defines the pins from which the arduino reads/the ones connected to IR sensor
const int externalLED = 13;                                                     //Pre-defines the pin for the LED that is on at the end
const int calibrationLED = 12;                                                  //Pre-defines the pin for the LED that is on at the time of calibration

//Sensor and calibration variables
bool polarity;                                                                  //0 means black tape on white bg(darker tape), 1 means white tape on black bg(lighter tape), actual colors could be anything
int sensorRaw[8] = {0, 0, 0, 0, 0, 0, 0, 0};                                    //Direct readings, Analog
int sensorNormalized[8] = {0, 0, 0, 0, 0, 0, 0, 0};                             //Normalized form of direct readings
int sensorMax[8] = {0, 0, 0, 0, 0, 0, 0, 0};                                    //Max value each sensor reads (defines limits)
int sensorMin[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};            //Min value each sensor reads (defines limits)
bool sensorBool[8] = {false, false, false, false, false, false, false, false};  /It's 0 or 1 based on whether the bot detects a line or not
int weightedLinePosition = 0;                                                   //Gives the weighted number which determines where the line is
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
int lastPosition;                                                               //Incase the bot gets lost

//Motor Control Variables
const int maxSpeed = 255;                                                       //Upper speed limit, full power to motor
const int minSpeed = 0;                                                         //Lower speed limit, no power to motor
int baseSpeed;                                                                  //This will be set as dryBaseSpeed or actualBaseSpeed but used directly at operation time
int dryBaseSpeed;                                                               //Base speed for dry run (slower and more stable)
int actualBaseSpeed;                                                            //Base speed for actual run (can be faster)
int leftSpeed;                                                                  //speed of left motor during adjustment
int rightSpeed;                                                                 //speed of right motor during adjustment, difference in speed causes rotation
int turnDuration;                                                               //how long to turn for
char currentTurn;//The turn being executed right now

//Path Logic Variables
char path[256];                                                                 //Stores the decisions made by the robot at every junction (FLRB)
int pathIndex;                                                                  //pseudo pointer to track index
int pathExecIndex;                                                              //pointer in actual run mode

//Mode and State variables
//NOTE : In calibration mode, the general state of the robot is irrelevant
enum Mode{idleMode, calibrationMode, dryRunMode, actualRunMode};                //Definition of modes
Mode mode;                                                                      //Variable to store the mode
enum State{idleState, followingLine, atJunction, turning, atEnd};               //Definition of general states
State state;                                                                    //Variable to store the state
enum CalibState{calibIdle, calibStart, calibSampling, calibEnd};                //Definition of calibration states
CalibState calibState;                                                          //Variable to store the calibration state
bool turningComplete;                                                           //tells if the turn is completed
bool junctionDetected;                                                          //Raw detection
bool junctionConfirmed;                                                         //Confirmed detection
bool allWhite;                                                                  //Sensors see all white
bool endConfirmed;                                                              //Confirmed end detection
bool endLEDState;                                                               //Whether the end LED is on or not
bool calibLEDState;                                                             //Whether the calibration LED is on or not
bool lineLost;                                                                  //Whether the line was lost (does not become true on reaching end)

//Time Variables, unit : ms
unsigned long currentTime;                                                      //stores current time, used as reference, called by millis()
unsigned long lastTime;                                                         //reusable timestamp for the last event that occured
unsigned long lastCalibTime;                                                    //stores the last time calibration started
unsigned long whiteStartTime;                                                   //starts the timer when sensors go all white to detect end
unsigned long turnStartTime;                                                    //stores the timestamp of when it starts turning
unsigned long junctionStartTime;                                                //stores the timestamp of when the junction was seen
unsigned long dryRunButtonDebounce;                                             //Acts as a debounce for the dry run button
unsigned long actualRunButtonDebounce;                                          //Acts as a debounce for the actual run button
unsigned long calibrationButtonDebouce;                                         //Acts as a debounce for the calibration button

/*----------------------------------------FUNCTIONS----------------------------------------*/
//Function Declarations--------------------------------------------------------------------//
void stopMotors();
void readSensors();
void updateMinMax();
void normalizeReadings();
void updateSensorBool();
void startCalibration();
void runCalibration();

//Sensor and Calibration Functions---------------------------------------------------------//
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
    }
  }
}

void updateSensorBool(){
  activeSensorCount = 0;
  lineDetected = false;
  for(int i = 0; i<8; i++){
    if(polarity==0) sensorBool[i] = sensorNormalized[i]<normalizedThreshold;
    else sensorBool[i] = sensorNormalized[i]>normalizedThreshold;
    if(sensorBool[i]){
      activeSensorCount++;
      lineDetected = true;
    }
  }
}

void startCalibration(){
  calibState = calibStart;
  calibLEDState = true;
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
  isCalibrated = false;
  endLEDState = false;
  calibState = calibSampling;
  stopMotors();
}

void runCalibration(){
    
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
