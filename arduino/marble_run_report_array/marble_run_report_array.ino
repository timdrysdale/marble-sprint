/*

Marble Run 
Tim Drysdale 20 August 2020

LICENSE: AGPLv3
COPYRIGHT: (c) Timothy D Drysdale

Await "go" button to be pushed
Open gates for marbles, controlled by servos sharing same PWM signal
Record time that each timing gate detects marble passing
Report times for each lane
Put gates up ready for next race

State machine boilerplate from:
https://www.edn.com/electronics-blogs/embedded-basics/4406821/Function-pointers---Part-3--State-machines



*/

#include <Servo.h>

int timeMs = 0; //for the messages
bool debug = false;
bool encoderPlain = false;
int sameNeeded = 8;

unsigned long interval = 100; //set by interval command
bool timeIsUp = false;
bool goButtonState = LOW;
int lastGoButtonState = LOW;   // the previous reading from the input pin

int gateUpAngle = 60;
int gateDownAngle = 120;
int position = 0;
volatile long startTime = 0;
Servo gateServo;  // create a servo object


enum PinAssignments {
gateServoPin = 16,
lane1DetectPin = 3,
lane2DetectPin = 9,
lane3DetectPin = 10,
lane4DetectPin = 11,
goButtonPin = 2,
lane1LEDPin = 5,
lane2LEDPin = 6,
lane3LEDPin = 7,
lane4LEDPin = 8,
};


volatile long lane1Millis = 0;
volatile long lane2Millis = 0;
volatile long lane3Millis = 0;
volatile long lane4Millis = 0;

bool lane1Finished = false;
bool lane2Finished = false;
bool lane3Finished = false;
bool lane4Finished = false;



/**
 * Defines the valid states for the state machine
 * 
 */
typedef enum
{
  STATE_PREPARE,
  STATE_START,
  STATE_REPORT,
} StateType;

//state Machine function prototypes
void Sm_State_Prepare(void);
void Sm_State_Start(void);
void Sm_State_Report(void);


/**
 * Type definition used to define the state
 */
 
typedef struct
{
  StateType State; /**< Defines the command */
  void (*func)(void); /**< Defines the function to run */
} StateMachineType;
 
/**
 * A table that defines the valid states of the state machine and
 * the function that should be executed for each state
 */
StateMachineType StateMachine[] =
{
  {STATE_PREPARE, Sm_State_Prepare},
  {STATE_START, Sm_State_Start},
  {STATE_REPORT, Sm_State_Report},
};
 
int NUM_STATES = 3;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_PREPARE;

void Sm_State_Prepare(void) {
  
  digitalWrite(lane1LEDPin, HIGH);
  digitalWrite(lane2LEDPin, HIGH);
  digitalWrite(lane3LEDPin, HIGH);
  digitalWrite(lane4LEDPin, HIGH);
  
  lane1Finished = false;
  lane2Finished = false;
  lane3Finished = false;
  lane4Finished = false;
  position = 0;
  
  gateServo.write(gateUpAngle);
  
  delay(500);
  
  digitalWrite(lane1LEDPin, LOW);
  digitalWrite(lane2LEDPin, LOW);
  digitalWrite(lane3LEDPin, LOW);
  digitalWrite(lane4LEDPin, LOW);
   
  // wait for the start button
  Serial.println("{\"msg\":\"status\",\"status\":\"prepared\"}"); 
  goButtonState = digitalRead(goButtonPin);
  
  while(goButtonState == false) {
    goButtonState = digitalRead(goButtonPin);
  } 
  startTime = millis();
  lane1Millis = -1;
  lane2Millis = -1;
  lane3Millis = -1;
  lane4Millis = -1;
  
  SmState = STATE_START;
}


void Sm_State_Start(void) {
  
  // put down the gates and blink the lane lights
  
  gateServo.write(gateDownAngle);
  Serial.println("{\"msg\":\"status\",\"status\":\"started\"}"); 
  digitalWrite(lane1LEDPin, HIGH);
  digitalWrite(lane2LEDPin, HIGH);
  digitalWrite(lane3LEDPin, HIGH);
  digitalWrite(lane4LEDPin, HIGH);

  
  delay(250);
  
  digitalWrite(lane1LEDPin, LOW);
  digitalWrite(lane2LEDPin, LOW);
  digitalWrite(lane3LEDPin, LOW);
  digitalWrite(lane4LEDPin, LOW); 

  delay(250);
  
  
  SmState = STATE_REPORT;
}

void Sm_State_Report(void) {
  
  SmState = STATE_REPORT; //stay in this state unless we race finishes or button pressed
  
  goButtonState = digitalRead(goButtonPin);
  
  bool finishedState = lane1Finished && lane2Finished && lane3Finished && lane4Finished;
 
  if (goButtonState == HIGH || finishedState == HIGH) {
    if (finishedState == HIGH) {
       Serial.println("{\"msg\":\"status\",\"status\":\"finished\"}"); 
    } else {
      Serial.println("{\"msg\":\"status\",\"status\":\"incomplete\"}"); 
    }
    delay(200); // wait for interrupts to finish registering lane times
    report_times();
    SmState = STATE_PREPARE;
  }

}


void report_times(void)
{
  Serial.print("{\"msg\":\"report\",\"raw_times\":["); 
  Serial.print(startTime);
  Serial.print(",");
  Serial.print(lane1Millis);
  Serial.print(",");
  Serial.print(lane2Millis);
  Serial.print(",");
  Serial.print(lane3Millis);
  Serial.print(",");
  Serial.print(lane4Millis);
  Serial.print("],\"finished\":[");
  Serial.print(lane1Finished);
  Serial.print(",");
  Serial.print(lane2Finished);
  Serial.print(",");
  Serial.print(lane3Finished);
  Serial.print(",");
  Serial.print(lane4Finished);
  Serial.println("]}");
}


void setup() {

  
  gateServo.attach(gateServoPin); // attaches the servo on pin 16 to the servo object
  Serial.setTimeout(50);
  Serial.begin(115200); // open a serial connection to your computer

  pinMode(lane1DetectPin, INPUT);
  pinMode(lane2DetectPin, INPUT);
  pinMode(lane3DetectPin, INPUT);
  pinMode(lane4DetectPin, INPUT);
  
  pinMode(goButtonPin, INPUT);     
  
  pinMode(lane1LEDPin, OUTPUT);
  pinMode(lane2LEDPin, OUTPUT);
  pinMode(lane3LEDPin, OUTPUT);
  pinMode(lane4LEDPin, OUTPUT);
  
  digitalWrite(lane1LEDPin, HIGH);
  digitalWrite(lane2LEDPin, HIGH);
  digitalWrite(lane3LEDPin, HIGH);
  digitalWrite(lane4LEDPin, HIGH);

  attachInterrupt(digitalPinToInterrupt(lane1DetectPin), detectLane1, RISING);
  attachInterrupt(digitalPinToInterrupt(lane2DetectPin), detectLane2, RISING);
  attachInterrupt(digitalPinToInterrupt(lane3DetectPin), detectLane3, RISING);
  attachInterrupt(digitalPinToInterrupt(lane4DetectPin), detectLane4, RISING);
  
}


void loop(){
  Sm_Run();
} 



void Sm_Run(void)
{
  if (SmState < NUM_STATES)
  {
    if (debug){
      Serial.print("{\"State\":");
      Serial.print(SmState);
      Serial.println("}");
    }

    (*StateMachine[SmState].func)();
    
  }
  else{
    Serial.println("Exception in State Machine");
  }
  
}


void detectLane1(){
  if (!lane1Finished) {
  lane1Finished = true;
  digitalWrite(lane1LEDPin, HIGH);
  lane1Millis = millis();
  }
}

void detectLane2(){
  if (!lane2Finished) {
  lane2Finished = true;
  digitalWrite(lane2LEDPin, HIGH);
  lane2Millis = millis();
  }
}

void detectLane3(){
  if (!lane3Finished) {
  lane3Finished = true;
  digitalWrite(lane3LEDPin, HIGH);
  lane3Millis = millis();
  }
}

void detectLane4(){
  if (!lane4Finished) {
  lane4Finished = true;
  digitalWrite(lane4LEDPin, HIGH);
  lane4Millis = millis();
  }
}


