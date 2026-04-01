//Sensor and calibration variables
const int AlanConstant = 1000;//Used for scaling all the normalized fractions to a standardized range in honor of Alan
const int normalizedThreshold = AlanConstant/2;
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};//Pre-defines the pins from which the arduino reads/the ones connected to IR sensor
const int weights[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};//Pre-assigned weights for calculating the weighted mean
bool polarity;//0 means black tape on white bg(darker tape), 1 means white tape on black bg(lighter tape), actual colors could be anything
int sensorRaw[8] = {0, 0, 0, 0, 0, 0, 0, 0};//Direct readings, Analog
int sensorNormalized[8] = {0, 0, 0, 0, 0, 0, 0, 0};//Normalized form of direct readings
int sensorMin[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};//Min value each sensor reads (defines limits)
int sensorMax[8] = {0, 0, 0, 0, 0, 0, 0, 0};//Max value each sensor reads (defines limits)
bool sensorBool[8];//It's 0 or 1 based on whether the bot detects a line or not
int weightedLinePosition = 0;//Gives the weighted number which determines where the line is
bool lineDetected = false;//true if atleast one sensor sees the line
int activeSensorCount = 0;//How many sensors detect a line
bool isCalibrated = false;

//PID Variables
float errorP;//current error
float errorI;//accumulated error
float errorD;//derivative term for error
float prevError;//error in the last loop
float Kp;//Proportional Gain
float Ki;//Integral Gain
float Kd;//Derivative Gain
float correction;//PID output of correction to be made in motor speed
int lastPosition;//Incase the bot gets lost

//Motor Control Variables
const int maxSpeed = 255;//Upper speed limit, full power to motor
const int minSpeed = 0;//Lower speed limit, no power to motor
int baseSpeed;//This will be set as dryBaseSpeed or actualBaseSpeed but used directly at operation time
int dryBaseSpeed;//Base speed for dry run (slower and more stable)
int actualBaseSpeed;//Base speed for actual run (can be faster)
int leftSpeed;//speed of left motor during adjustment
int rightSpeed;//speed of right motor during adjustment, difference in speed causes rotation
int turnDuration;//how long to turn for
char currentTurn;//The turn being executed right now

//Path Logic Variables
char path[256];//Stores the decisions made by the robot at every junction (FLRB)
int pathIndex;//pseudo pointer to track index
int pathExecIndex;//pointer in actual run mode

//Run Mode Variables, defines the mode in which the robot is operating
enum Mode{idleMode, dryRun, actualRun};
Mode mode;

//State variables
enum State{idleState, followingLine, atJunction, turning, atEnd};
State state;
bool turningComplete;//tells if the turn is completed
bool junctionDetected;//Raw detection
bool junctionConfirmed;//Confirmed detection
bool allWhite;//Sensors see all white
bool endConfirmed;//Confirmed end detection
bool LEDState;//Whether the LED is on or not
bool lineLost;//Whether the line was lost (does not become true on reaching end)

//Time Variables, unit : ms
unsigned long currentTime;//stores current time, used as reference
unsigned long lastTime;//reusable timestamp for the last event that occured
unsigned long whiteStartTime;//starts the timer when sensors go all white to detect end
unsigned long turnStartTime;//stores the timestamp of when it starts turning
unsigned long junctionStartTime;//stores the timestamp of when the junction was seen
unsigned long buttonDebounce;//Acts as a debounce for the physical buttons

void updateMinMax(){
  for(int i = 0; i<8; i++){
    if(sensorRaw[i]<sensorMin[i]) sensorMin[i] = sensorRaw[i];
    if(sensorRaw[i]>sensorMax[i]) sensorMax[i] = sensorRaw[i];
  }
}

void readSensors(){
  for(int i = 0; i<8; i++) sensorRaw[i] = analogRead(sensorPins[i]);
}

void normalizeReadings(){
  for(int i = 0; i<8; i++){
    if(sensorMax[i]-sensorMin[i]==0){
      sensorNormalized[i] = 0;
      continue;
    }
    sensorNormalized[i] = ((long)(sensorRaw[i]-sensorMin[i])*AlanConstant)/(sensorMax[i]-sensorMin[i]);
    sensorNormalized[i] = constrain(sensorNormalized[i], 0, AlanConstant);
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

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
