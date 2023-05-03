// new pretensiong algorithm
#include "HX711-multi.h"
#include <TimerOne.h>
#include "InputDebounce.h"
#include <FrequencyTimer2.h>

#include "version_tracker.h"

/**
       Cmd value                           Mask value
            [3]<---------\                     [0x8]
             |           |                       |
             |           |                       |
      [1]----+----[2]X  [30]            [0x2]----+----[0x4]X
       ^     |     ^     |                       |
       |     |     |     |                       |
       |    [0]<-------- /                     [0x1]
       |     Y     |                             Y
       |           |
       \---[12]----/

*/

#include<ADS1115_WE.h>
#include<Wire.h>
#define I2C_ADDRESS 0x48

//ADS1115_WE adc(I2C_ADDRESS);
// ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
boolean haveADC = false;

int   REPEATED_SAMPLES = 2;   // number of samples for averaging while pretensioning
#define BUTTON_DEBOUNCE_DELAY   20   // [ms]

/* Pins to the load cell amp */
#define CLK 2      // clock pin to the load cell amp
#define DOUT1 3    // data pin to the first lca
#define DOUT2 4    // data pin to the second lca
#define DOUT3 5    // data pin to the third lca
#define DOUT4 6    // data pin to the third lca

/* */
#define ENDSTOP_0 7
#define ENDSTOP_1 8
#define ENDSTOP_2 9
#define ENDSTOP_3 10
#define MOTOR_DIRECTION 12

#define MOVE_STEP     digitalWrite(12, HIGH)
#define MOVE_BACK     digitalWrite(12, LOW)
#define MOVE_COND(x)  digitalWrite(12, x)

/* Motor index using mask value. */
#define cMOTOR_ZERO      0x01
#define cMOTOR_ONE       0x02
#define cMOTOR_TWO       0x04
#define cMOTOR_THREE     0x08

/* select 1=16 or 0=256 microstepping and the output pin that controls it (11) */
#define STEP_SIZE       1
#define MICROSTEPPING   11

/* Direction used for home, step function. */
#define TOWARDS_THE_CENTER   0
#define AWAY_FROM_CENTER     1

#define BOOT_MESSAGE "MIT_ML_SCALE V0.8"

#define HOME_SPEED 250

#define TARE_TIMEOUT_SECONDS 4
#define CHANNEL_COUNT 4

#define cSTPMM    (long)(640 / 2)

/* Increment value applied inside pre-Tensioning function */
#if false
#define cPREX2_INC    (0.1)
#define cPREY2_INC    (0.1)
#else
#define cPREX2_INC    (0.05)
#define cPREY2_INC    (0.05)
#endif

byte DOUTS[CHANNEL_COUNT] = {DOUT1, DOUT2, DOUT3, DOUT4};
long int results[CHANNEL_COUNT];

HX711MULTI scales(CHANNEL_COUNT, DOUTS, CLK);

/* Entry point function for motors home execution. */
void home_Movement( int motor, uint8_t dir);
void home_MultiMotorXY( uint8_t dir);
void home_MultiMotorX( uint8_t dir);
void home_MultiMotorY( uint8_t dir);
void home_Motor( byte mask, uint8_t dir);

/* Entry point function for step motor execution. */
void step_Movement( byte m, uint8_t dir);
void step_MotorX( void);
void step_MotorY( void);
void step_Motor( byte m);
void step_MotorXY(void);

/* Entry point function for pre-tensioning execution. */
void motor_PreTensioning( byte m);
void motor_PreTensioningX(void);
void motor_PreTensioningY(void);
void motor_PreTensioningXY(void);

/* */
void farFromEndStop_Move( byte m, uint8_t dir);
void farFromEndStop( byte m, uint8_t dir);
void farFromEndStopXY( uint8_t dir);
void farFromEndStopX( uint8_t dir);
void farFromEndStopY( uint8_t dir);

/* Data container for stats about pre tensioning. */
typedef struct
{
  uint16_t singleStepTowards;
  uint16_t incPreTens;
  uint16_t singleStepAway;
} PreTensionMoveStat_TypeDef;

typedef struct
{
  uint8_t Dir;
  uint8_t Mask;
  uint8_t Cmd;
  uint8_t EndPointMask;
  long    StepCount;
  long    Target;
  long    MaxTravel;
  long    MaxForce;
  long    preTensTime;
  float   Threshold;
  boolean Moving;
  boolean Ended;
  boolean Display;
  const long stpmm = cSTPMM;
} BiAxialRunTimeParam_TypeDef;

/* */
uint8_t actualDir = TOWARDS_THE_CENTER;
uint8_t actualMask = 0;
uint8_t actualCmd = 0;
/* pulse output */
//byte motor = 9;
int mn = 0;
byte mask = 0;                  // which bits of PORTC will be pulsed to create steps <- CHANGE
byte EndPointMask = 0;
long step_count = 0;
long timer = 500;
int ledState = LOW;
boolean moving = false;
long target = 0;
boolean th = false;
boolean manual_motion = false;
volatile boolean ended = false;

float threshold = 0;           // force threshold to be keeping
long zero_time;
long zero_factor;
boolean display = false;
//const long stpmm = 640 / 2;   // steps to move 1 mm, with two motors now it is halved!!!!
const long stpmm = cSTPMM;      // steps to move 1 mm, with two motors now it is halved!!!!
long max_travel = 200, max_force = 2500;
long preTime = 0;
float preX = 0, preY = 0;
float preX2, preY2;

int axis;

int pretension_idle = 4;      // number of pretension idle cycles before pretensioning ended

#define M0 A0
#define M1 A1
#define M2 A2
#define M3 A3

volatile boolean EndStop0_State = false;    // true == pressed
volatile boolean EndStop1_State = false;
volatile boolean EndStop2_State = false;
volatile boolean EndStop3_State = false;

static InputDebounce inDebounce_EndStop0;
static InputDebounce inDebounce_EndStop1;
static InputDebounce inDebounce_EndStop2;
static InputDebounce inDebounce_EndStop3;
unsigned long now;

/* */
uint16_t homingSpeed = HOME_SPEED;

/* Set to 1 to enable debug msg for debounce. */
#define DEBUG_DEBOUNCE  0

void EndStop0_releasedCallback(uint8_t pinIn)
{
#if (DEBUG_DEBOUNCE==1)
  Serial.print("RELEASED (pin: ");
  Serial.print(pinIn);
  Serial.println(")");
#endif
  EndStop0_State = false;
}

void EndStop1_releasedCallback(uint8_t pinIn)
{
#if (DEBUG_DEBOUNCE==1)
  Serial.print("RELEASED (pin: ");
  Serial.print(pinIn);
  Serial.println(")");
#endif
  EndStop1_State = false;
}

void EndStop2_releasedCallback(uint8_t pinIn)
{
#if (DEBUG_DEBOUNCE==1)
  Serial.print("RELEASED (pin: ");
  Serial.print(pinIn);
  Serial.println(")");
#endif
  EndStop2_State = false;
}

void EndStop3_releasedCallback(uint8_t pinIn)
{
#if (DEBUG_DEBOUNCE==1)
  Serial.print("RELEASED (pin: ");
  Serial.print(pinIn);
  Serial.println(")");
#endif
  EndStop3_State = false;
}

void EndStop0_pressedCallback(uint8_t pinIn)
{
#if (DEBUG_DEBOUNCE==1)
  Serial.print("PRESSED (pin: ");
  Serial.print(pinIn);
  Serial.println(")");
#endif
  EndStop0_State = true;
}

void EndStop1_pressedCallback(uint8_t pinIn)
{
#if (DEBUG_DEBOUNCE==1)
  Serial.print("PRESSED (pin: ");
  Serial.print(pinIn);
  Serial.println(")");
#endif
  EndStop1_State = true;
}

void EndStop2_pressedCallback(uint8_t pinIn)
{
#if (DEBUG_DEBOUNCE==1)
  Serial.print("PRESSED (pin: ");
  Serial.print(pinIn);
  Serial.println(")");
#endif
  EndStop2_State = true;
}

void EndStop3_pressedCallback(uint8_t pinIn)
{
#if (DEBUG_DEBOUNCE==1)
  Serial.print("PRESSED (pin: ");
  Serial.print(pinIn);
  Serial.println(")");
#endif
  EndStop3_State = true;
}

/**
*/
void tare()
{
  bool tareSuccessful = false;

  unsigned long tareStartTime = millis();

  while (!tareSuccessful && millis() < (tareStartTime + TARE_TIMEOUT_SECONDS * 1000))
  {
    tareSuccessful = scales.tare(20, 10000); //reject 'tare' if still ringing
  }
}

float scale_factor[CHANNEL_COUNT] = { 8947.398 /*10678.1*/, 10678.1 / (9.81 / 8.21) , 10678.1  / (9.81 / 8.15), 10678.1  / (9.81 / 8.18)} ;
//105.849 * 9.81 * 7.03 / (9.81/14.35); // approx Netwtons

float force[CHANNEL_COUNT]; // last values read
float MonoAxialMeanForceX, MonoAxialMeanForceY;
/**
*/
void printRawData()
{
  scales.read(results);

  for (int i = 0; i < scales.get_count(); ++i)
  {
    force[i] = -(results[i] / scale_factor[i]);
    Serial.print( force[i], 3); // three decimals for weight measures!!
    Serial.print( /* (i != scales.get_count() - 1) ? */ "\t" /* : "\n"*/);
  }
  float voltage = 0;

  // if (haveADC) for(int i=0;i<10;i++) voltage +=  adc.getResult_V();
  //Serial.print(voltage/10,5);
  Serial.println(voltage * (75 / 10) / 5.0, 3);
  MonoAxialMeanForceY = (force[0] + force[3]) / 2;
  MonoAxialMeanForceX = (force[1] + force[2]) / 2;
}

/**
*/
void getRawData()
{
  if (preTime == 0) {
    scales.read(results);

    for (int i = 0; i < scales.get_count(); ++i)
    {
      force[i] = -(results[i] / scale_factor[i]) ;
    }
    MonoAxialMeanForceY = (force[0] + force[3]) / 2;
    MonoAxialMeanForceX = (force[1] + force[2]) / 2;
  }
  else {
    force[0] = force[1] = force[2] = force[3] = 0;
    for (int times = 0; times < REPEATED_SAMPLES; times++) { // we want to do the average of many samples
      scales.read(results);
      for (int i = 0; i < scales.get_count(); ++i)
      {
        force[i] += -(results[i] / scale_factor[i]) ;
      }
    }
    for (int i = 0; i < scales.get_count(); ++i) force[i] = force[i] / REPEATED_SAMPLES;
    MonoAxialMeanForceY = (force[0] + force[3]) / 2;
    MonoAxialMeanForceX = (force[1] + force[2]) / 2;
  }
}

/**
  @brief -1=all, 0..3 individual,12=X,30=Y,255=all disabled  CHANGE, maybe a mask to specify which to move for each step
*/
void select_motor(int i)
{
  switch (i)
  {
    case -1:
      mask = 0x0f;
      break;
    case  0:
      mask = 1;
      break;
    case  1:
      mask = 2;
      break;
    case  2:
      mask = 4;
      break;
    case  3:
      mask = 8;
      break;
    case 12:
      mask = 0x06;
      break;              // 0110
    case 30:
      mask = 0x09;
      break;              // 1001
    case 255:
      mask = 0;           //none moves
      break;
    default:
      break;
  }
  // if (i==255) digitalWrite(motor,1); // disable motor drivers
  // else digitalWrite(motor,0); // enable motor drivers
}

/*
   @brief EndStopX_State == 0 -> the motor is not at the end

*/
uint8_t checkEndStop( byte m)
{
  uint8_t ret = 0;

  switch (m)
  {
    case 0x0f:
      ret = (EndStop3_State << 3) | (EndStop2_State << 2) | (EndStop1_State << 1) | (EndStop0_State << 0);
      break;
    case 0x01:
      ret = EndStop0_State;
      break;
    case 0x02:
      ret = (EndStop1_State << 1);
      break;
    case 0x04:
      ret = (EndStop2_State << 2);
      break;
    case 0x08:
      ret = (EndStop3_State << 3);
      break;
    case 0x06:
      ret = (EndStop2_State << 2) | (EndStop1_State << 1);
      break;
    case 0x09:
      ret = (EndStop0_State << 0) | (EndStop3_State << 3);
      break;
  }
  return ret;
}

/**
    @brief Called every 1ms, update the inDebounce state and update the EndPoint mask var with
           the values of end point state.

*/
void EndStopChkISR()
{
  now = millis();

  inDebounce_EndStop0.process(now);
  inDebounce_EndStop1.process(now);
  inDebounce_EndStop2.process(now);
  inDebounce_EndStop3.process(now);
  // EndPointMas is "bit like" the var "mask".
  EndPointMask = (EndStop3_State << 3) | (EndStop2_State << 2) | (EndStop1_State << 1) | (EndStop0_State << 0);
}

/**
*/



void setup()
{
  Wire.begin();
  Serial.begin(115200);
  delay(50);
  /*
    // check if ADS1115 is present
    if(!adc.init()){
    Serial.println("ADS1115 not connected!");
    }
    else {
    haveADC=true;
    adc.setVoltageRange_mV(ADS1115_RANGE_6144);
    adc.setCompareChannels(ADS1115_COMP_0_GND);
    adc.setMeasureMode(ADS1115_CONTINUOUS);
    adc.setCompareChannels(ADS1115_COMP_0_GND);
    }
  */
  // Direction of motor
  pinMode(MOTOR_DIRECTION, OUTPUT);

  // Configure Timer1 to run every 1.116 microseconds
  Timer1.initialize(186 * 3 * 2); // 1116
  Timer1.attachInterrupt(StepAndMovingChkISR);

  /* Timer2 is used to verify debounce. */
  FrequencyTimer2::setPeriod(1000);
  FrequencyTimer2::setOnOverflow(EndStopChkISR);

  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(MICROSTEPPING, OUTPUT);
  digitalWrite(MICROSTEPPING, LOW); // fast, 16 microsteps

  inDebounce_EndStop0.registerCallbacks( EndStop0_pressedCallback, EndStop0_releasedCallback, NULL, NULL);
  inDebounce_EndStop1.registerCallbacks( EndStop1_pressedCallback, EndStop1_releasedCallback, NULL, NULL);
  inDebounce_EndStop2.registerCallbacks( EndStop2_pressedCallback, EndStop2_releasedCallback, NULL, NULL);
  inDebounce_EndStop3.registerCallbacks( EndStop3_pressedCallback, EndStop3_releasedCallback, NULL, NULL);

  inDebounce_EndStop0.setup( ENDSTOP_0, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES, 0, InputDebounce::ST_NORMALLY_CLOSED);
  inDebounce_EndStop1.setup( ENDSTOP_1, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES, 0, InputDebounce::ST_NORMALLY_CLOSED);
  inDebounce_EndStop2.setup( ENDSTOP_2, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES, 0, InputDebounce::ST_NORMALLY_CLOSED);
  inDebounce_EndStop3.setup( ENDSTOP_3, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES, 0, InputDebounce::ST_NORMALLY_CLOSED);

  select_motor(-1);

  help();
  tare();
}

/**
   @brief Check target step count, and manage "moving" flag. Here the "target" variable is checked against
          "step_count" limit. If reached set to FALSE "moving" and set TRUE "ended".
          Olso this function update the value of PORTC writing the mask value.
*/
void StepAndMovingChkISR(void)
{

  if (moving == false && manual_motion == false)
  {
    return;
  }

  if (EndStop0_State || EndStop1_State || EndStop2_State || EndStop3_State)
  {
    // Stop the running command.
    moving = false;
    ended = true;     // Stop the current command
    return; // no motor moves!!
  }

  if (ledState == LOW)
  {
    ledState = HIGH;
    if (step_count < target)
    {
      step_count++;
    } else {
      if (step_count > target)
      {
        step_count--;
      }
    }
    //blinkCount = blinkCount + 1;  // increase when LED turns on
  } else {
    ledState = LOW;
  }
  //digitalWrite(motor, ledState); // --- CHANGE to move the desired axis
  PORTC = mask * ledState;
  if (target == step_count && moving)
  {
    moving = false;
    ended = true;
  }
}


void process_line()
{
  int motor;
  char cmd;
  int uu;

  cmd = Serial.read();

  if (cmd > 'Z')
  {
    cmd -= 32;
  }

  switch (cmd)
  {
    case 'A':               // A123 Set homing speed
      homingSpeed = Serial.parseInt();
      Serial.println("ok");
      break;
    case 'B':               // Print out firmware version
      Serial.print(FIRMWARE_VER);
      Serial.print(" ");
      Serial.println("ok");
      break;
    case 'R':               // R resets the load cell (current force=0 Newtons
      moving = false;
      threshold = 0;
      step_count = 0;
      Serial.println("ok");
      break;
    case 'T':               // T for sensors tare
      tare();
      Serial.println("ok");
      break;
    // The command 'F' is not used
    case 'F':               // F123.34 sets the pulling force to 123.34 Newtons. It will move to keep that force
      threshold = Serial.parseFloat();
      MOVE_STEP;  //digitalWrite(12, HIGH);
      Serial.println("ok");
      target = step_count + 200 * stpmm;
      th = true;
      moving = true;      // Enable motor inside "blinkLED"
      display = true;
      break;
    case '?':               // ? prints out current number of steps and the current force on the load cell
      printPos();
      break;
    case 'X':               // X123 moves 123 steps (it can be negative too)
      target = Serial.parseInt() * stpmm;
      moving = true;      // Enable motor inside "blinkLED"
      display = true;
      threshold = 0;
      MOVE_COND(target > step_count); //digitalWrite(12, target > step_count);
      actualDir = (target > step_count);  // save the motion direction
      actualMask = mask;  // save the mask for this activity.
      actualCmd = 'X';
      Serial.println(target);
      zero_time = millis();
      break;
    case 'H':               // H for help menu
      help();
      break;
    case 'S':               // S stop motion
      moving = false;
      threshold = 0;
      th = false;
      display = false;
      preTime = 0;
      Serial.println("ok");
      break;
    case 'D':               // D toggle display
      display = !display;
      break;
    // The command 'P' is not used.
    case 'P':               // P peel test
      moving = true;      // Enable motor inside "blinkLED"
      display = true;
      MOVE_STEP;  //digitalWrite(12, HIGH);
      target = step_count + max_travel * stpmm;
      break;
    case 'V':               // V123 change speed to 123 mm/min
      timer = Serial.parseInt();
      Timer1.setPeriod(timer);
      Serial.println("ok");
      break;
    case 'M':               // M123 selects the motors to move, -1=all, 0..3:invidual, 12,30, 255=none
      mn = Serial.parseInt();
      select_motor(mn);
      Serial.println("ok");
      break;
    case 'L':               // L123 sets the max elongation value to 123 mm for any operation
      max_travel = Serial.parseInt();
      Serial.println("ok");
      break;
    case 'C':               // C123 sets the maximum force value to 123 Newtons
      max_force = Serial.parseInt();
      Serial.println("ok");
      break;
    case 'K':               // K123 sets the pre-tensioning force X-axis value to 123 Newtons
      preX = Serial.parseFloat();
      Serial.println("ok");
      break;
    case 'J':               // J123 sets the pre-tensioning force Y-axis value to 123 Newtons
      preY = Serial.parseFloat();
      Serial.println("ok");
      break;
    case 'W':               // W123 pre-tensions the active axes to the set values for 123 seconds
      preTime = Serial.parseInt() * 1000 + millis();
      pretension_idle = 4;
      Serial.println("ok");
      // Update the force[x] values
      getRawData();
      // set preX and preY value
      preX2 = cPREX2_INC; //0.1;
      preY2 = cPREY2_INC; //0.1;
      // The Motors movements is controlled inside "loop" function until preTime is different from 0
      break; // preTime is time to stop in ms
    case 'E':               // endstops query
      Serial.print(EndStop0_State);
      Serial.print(" ");
      Serial.print(EndStop1_State);
      Serial.print(" ");
      Serial.print(EndStop2_State);
      Serial.print(" ");
      Serial.println(EndStop3_State);
      Serial.println("ok");
      break;
    case 'O':                 // home axis -- go to center!!!
      motor = Serial.parseInt();
      home_Movement(motor, TOWARDS_THE_CENTER);
      Serial.println("ok");
      break;
    case 'I':                 // home out axis -- away from center!!!
      motor = Serial.parseInt();
      home_Movement(motor, AWAY_FROM_CENTER);
      Serial.println("ok");
      break;
    case 'U':                 // microstepping level
      uu = Serial.parseInt();
      Serial.print("ok U"); Serial.println(uu);
      if (uu == 1) digitalWrite(MICROSTEPPING, HIGH);
      else digitalWrite(MICROSTEPPING, LOW);
      break;
    case 'Z':                 // oversampling level
      REPEATED_SAMPLES = Serial.parseInt();
      Serial.println("ok");
      break;
    case '@':                 // @1.1 2.2 3.3 4.4 changes the multipliers of all load cells to these new values
      if (Serial.available() > 2)
      {
        scale_factor[0] = Serial.parseFloat();
        scale_factor[1] = Serial.parseFloat();
        scale_factor[2] = Serial.parseFloat();
        scale_factor[3] = Serial.parseFloat();
      }

      for (int i = 0; i < CHANNEL_COUNT; i++)
      {
        Serial.println(scale_factor[i]);
      }
      Serial.println("ok");
      break;
    default:
      break;
  }
  while (Serial.read() != 10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

/**
*/
void help()
{
  Serial.print(F("\nIaccarino's biaxial tester ver"));
  Serial.println(FIRMWARE_VER);
  Serial.println(F("by misan"));
  Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
  Serial.println(F("H for help menu"));
  Serial.println(F("T for sensors tare"));
  Serial.println(F("S stop motion"));
  Serial.println(F("D toggle display"));
  Serial.println(F("P peel test"));
  Serial.println(F("E endstops query"));
  Serial.println(F("O home axis -- go to center"));
  Serial.println(F("I home out axis -- away from center"));
  Serial.println(F("V123 change speed to 123 mm/min"));
  Serial.println(F("R resets the load cell (current force=0 Newtons"));
  Serial.println(F("F123.34 sets the pulling force to 123.34 Newtons. It will move to keep that force"));
  Serial.println(F("X123 moves 123 steps (it can be negative too)"));
  Serial.println(F("L123 sets the max elongation value to 123 mm for any operation"));
  Serial.println(F("C123 sets the maximum force value to 123 Newtons"));
  Serial.println(F("K123 sets the pre-tensioning force X-axis value to 123 Newtons"));
  Serial.println(F("J123 sets the pre-tensioning force Y-axis value to 123 Newtons"));
  Serial.println(F("W123 pre-tensions the active axes to the set values for 123 seconds"));
  Serial.println(F("M123 selects the motors to move, -1=all, 0..3:invidual, 12,30, 255=none"));
  Serial.println(F("@1.1 2.2 3.3 4.4 changes the multipliers of all load cells to these new values"));
  Serial.println(F("A123 Set motor home speed"));
  Serial.println(F("B print out firmware version"));
  Serial.println(F("U1 sets the microstep level to 16 microsteps (256 otherwise)"));
  Serial.println(F("Z123 selects the number os samples averaged for a single load cell measurement"));
  Serial.println(F("? prints out current number of steps and the current force on the load cell"));
  Serial.println("");
}

void printPos()
{
  Serial.print(millis() - zero_time);
  Serial.print(" ");
  Serial.print(1.0 * step_count / stpmm, 4 );
  Serial.print(" ");
  printRawData();
  Serial.flush();
}

/**
       Cmd value                           Mask value
            [3]<---------\                     [0x8]
             |           |                       |
             |           |                       |
      [1]----+----[2]X  [30]            [0x2]----+----[0x4]X
       ^     |     ^     |                       |
       |     |     |     |                       |
       |    [0]<-------- /                     [0x1]
       |     Y     |                             Y
       |           |
       \---[12]----/

*/

void loop()
{
  now = millis();

  if (Serial.available())
  {
    process_line();
  }

  //force = bascula.get_units() / 3 * 9.81;
  if (display)
  {
    printPos();
  } else {
    getRawData();     // read cells even if we do not print the values
  }

  /* changed by Miguel to ABS so it works for Flexion test too */
  if (abs(MonoAxialMeanForceX) > max_force || abs(MonoAxialMeanForceY) > max_force)
  {
    moving = false; // security feature
    ended = true;
    Serial.println("MonoAxialMeanForce reached");
  }

  if (th)
  {
    if (threshold > 0 && (MonoAxialMeanForceX < threshold && MonoAxialMeanForceY < threshold ))
    {
      moving = true;  // it will do one new step on the motor if force is lower than 10 N
    } else {
      moving = false;
    }
  } else {
    if (preTime > 0)
    { // pre-tensioning is called for
      Serial.print("mask "); Serial.println(mask);
      motor_PreTensioning(mask);
    }
  }

  if (ended)
  {
    Serial.println("ok loop");  // stop displaying data if it was, print the missing ok at the end of motion
    ended = false;
    display = false;
    /* Check the moving flag */
    if (moving)
    {
      /* Just to make sure the flag is set to false when the ended flag was set to true. */
      moving = false;
      Serial.println("moving flag anomaly");
    }
    /* Manage speciale action for 'X' command used to start Break Test, Peeling Test and Step movement. */
    if ( actualCmd == 'X')
    {
      Serial.println("Cmd X complete");
      /* Set commando to zero because here we manage the end of the command execution. */
      actualCmd = 0;
      /* Test if there is an endpoint pressed*/
      if ( EndPointMask)
      {
        Serial.println("EndPoint Set");
        /* Start the function that move away the motor. */
        // farFromEndStop_Move(actualMask, actualDir); with this commented home is not catastrophic
      }
      /* Command X is used for the Break Test, that set a new value to max_force. */
      /* At this point max_force is set again to the maximum absolute value. */
      max_force = 2500;
    }
  }
}

/**
*/
void testCells()
{
  printRawData();
  delay(100);
}

/**
   @brief This funcion execute the "home" command from the PC. It can run single motor, all motor or motor on single axie.
          The function starts putting the selected motor at the end point; if more than a motor is selected, all the
          motor are put to the end point. Then the function starts to repositioning the motor far from the end point; if more
          than one motor is selected, for all other motors the process is completed.
   @param
*/
void home_Movement( int motor, uint8_t dir)
{

  switch (motor)
  {
    case -1:
      mask = 0x0f;
      home_MultiMotorXY( dir);
      break;
    case  0:
      mask = cMOTOR_ZERO; // 0x01;
      home_Motor( mask, dir);
      break;
    case  1:
      mask = cMOTOR_ONE;  // 0x02;
      home_Motor( mask, dir);
      break;
    case  2:
      mask = cMOTOR_TWO;  // 0x04;
      home_Motor( mask, dir);
      break;
    case  3:
      mask = cMOTOR_THREE; // 0x08;
      home_Motor( mask, dir);
      break;
    case 12:
      mask = 0x06;
      home_MultiMotorX( dir);
      break;              // b0110
    case 30:
      mask = 0x09;
      home_MultiMotorY( dir);
      break;              // b1001
    case 255:
      mask = 0;           //none moves
      break;
    default:
      mask = 0;
      break;
  }
}

/**
   @brief Move all motors axes. The function starts putting the selected motor at the end point;
          if more than a motor is selected, all the motor are put to the end point. Then the function
          starts to repositioning the motor far from the end point; if more
          than one motor is selected, for all other motors the process is completed.

   @param dir 0 = towards the center, 1 = away from the center
*/
void home_MultiMotorXY( uint8_t dir)
{
  boolean run = true;

  MOVE_COND(dir);
  while ( run)
  {
    run = false;
    // Y axie
    if ( EndStop0_State == 0)
    {
      PORTC = cMOTOR_ZERO;
      delayMicroseconds(20);
      PORTC = 0;
      delayMicroseconds(homingSpeed);
      run = true;
    }

    if ( EndStop3_State == 0)
    {
      PORTC = cMOTOR_THREE;
      delayMicroseconds(20);
      PORTC = 0;
      delayMicroseconds(homingSpeed);
      run = true;
    }
    // X axie
    if ( EndStop1_State == 0)
    {
      PORTC = cMOTOR_ONE;
      delayMicroseconds(20);
      PORTC = 0;
      delayMicroseconds(homingSpeed);
      run = true;
    }

    if ( EndStop2_State == 0)
    {
      PORTC = cMOTOR_TWO;
      delayMicroseconds(20);
      PORTC = 0;
      delayMicroseconds(homingSpeed);
      run = true;
    }
  }

  delay(10);

  MOVE_COND(!dir);
  run = true;
  while ( run)
    //for(int vv=0; vv<400; vv++) // 40  steps instead of 1
  {
    run = false;
    // Y axis
    if ( EndStop0_State > 0)
    {
      PORTC = cMOTOR_ZERO;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }

    if ( EndStop3_State > 0)
    {
      PORTC = cMOTOR_THREE;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }
    // X axis
    if ( EndStop1_State > 0)
    {
      PORTC = cMOTOR_ONE;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }

    if ( EndStop2_State > 0)
    {
      PORTC = cMOTOR_TWO;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }
  }
}

/**
   @brief Move X axis to home.

   @param dir 0 = towards the center, 1 = away from the center
*/
void home_MultiMotorX( uint8_t dir)
{
  boolean run = true;

  MOVE_COND(dir);
  while ( run)
  {
    run = false;

    if ( EndStop1_State == 0)
    {
      PORTC = cMOTOR_ONE;
      delayMicroseconds(20);
      PORTC = 0;
      delayMicroseconds(homingSpeed);
      run = true;
    }

    if ( EndStop2_State == 0)
    {
      PORTC = cMOTOR_TWO;
      delayMicroseconds(20);
      PORTC = 0;
      delayMicroseconds(homingSpeed);
      run = true;
    }
  }

  delay(10);

  MOVE_COND(!dir);
  run = true;
  while ( run)
  {
    run = false;
    if ( EndStop1_State > 0)
    {
      PORTC = cMOTOR_ONE;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }

    if ( EndStop2_State > 0)
    {
      PORTC = cMOTOR_TWO;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }
  }
}


/**
   @brief Move Y axis to home

   @param dir 0 = towards the center, 1 = away from the center
*/
void home_MultiMotorY( uint8_t dir)
{
  boolean run = true;

  MOVE_COND(dir);
  while ( run)
  {
    run = false;

    if ( EndStop0_State == 0)
    {
      PORTC = cMOTOR_ZERO;
      delayMicroseconds(20);
      PORTC = 0;
      delayMicroseconds(homingSpeed);
      run = true;
    }

    if ( EndStop3_State == 0)
    {
      PORTC = cMOTOR_THREE;
      delayMicroseconds(20);
      PORTC = 0;
      delayMicroseconds(homingSpeed);
      run = true;
    }
  }

  delay(10);

  MOVE_COND(!dir);
  run = true;
  while ( run)
  {
    run = false;
    if ( EndStop0_State > 0)
    {
      PORTC = cMOTOR_ZERO;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }

    if ( EndStop3_State > 0)
    {
      PORTC = cMOTOR_THREE;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }
  }
}

/**
  @brief Move *single* motor to home position

  @param[in] mask single motor activatin: 0x1, 0x2, 0x4, 0x8, 0x6, 0x9, 0xF
  @param[in] dir 0 = towards the center, 1 = away from the center
*/
void home_Motor( byte mask, uint8_t dir)
{
  boolean run;

  MOVE_COND(dir);
  run = true;
  while ( run)
  {
    run = false;
    if ( (EndPointMask & mask) == 0)
    {
      PORTC = mask;
      delayMicroseconds(20);
      PORTC = 0;
      delayMicroseconds(homingSpeed);
      run = true;
    }
  }

  delay(10);

  MOVE_COND(!dir);
  run = true;
  while ( run)
  {
    run = false;
    if ( (EndPointMask & mask) > 0)
    {
      PORTC = mask;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }
  }
}

/**
  @brief Entry point of the PreTensioning function. The dedicated motors pre-tensioning function is called
         from here.
  @param[in] m   motor mask as "mask value"
*/
void motor_PreTensioning( byte m)
{
  switch (m)
  {
    case 0x0f:
      motor_PreTensioningXY();
      break;
    case 0x06:
      motor_PreTensioningX();
      break;
    case 0x09:
      motor_PreTensioningY();
      break;
    default:
      break;
  }
}

/**
  @brief Execute the pre-tensioning for X motors axis. The code for the pretensioning is execute inside
         this function, to speedup the procedure. The function return only when preTime is zero that means the
         at the end of the pre tensioning.
*/
void motor_PreTensioningX(void)
{
  int k;
  /* Just collect some stats about pre tensioning. */
  PreTensionMoveStat_TypeDef preTens;

  preTens.singleStepTowards = 0;
  preTens.incPreTens = 0;
  preTens.singleStepAway = 0;

  while ( preTime > 0)
  {
    k = 2;
    if (Serial.available())
    {
      process_line();
    }

    printPos();
    /* */
    if (MonoAxialMeanForceX > max_force )
    {
      moving = false; // security feature
      Serial.print("ok Pre-tensX ended max_force ");
      Serial.println(MonoAxialMeanForceX);
    }

    // X axis
    if (force[1] < preX2)
    {
      step_Movement(cMOTOR_ONE, AWAY_FROM_CENTER);
      k--;
      preTens.singleStepAway++;
    }

    if (force[2] < preX2)
    {
      step_Movement(cMOTOR_TWO, AWAY_FROM_CENTER);
      k--;
      preTens.singleStepAway++;
    }

    //
    if (k == 2)
    { // if none moves then it is time to increase the limit
      if (preX2 < preX)
      {
        preX2 += cPREX2_INC;
        preTens.incPreTens++;
      }
    }
    // The MonoAxialMeanForceX var is calculated inside the printPos function
    if ((force[1] / MonoAxialMeanForceX) > 1.1)
    {
      step_Movement(cMOTOR_ONE, TOWARDS_THE_CENTER);
      preTens.singleStepTowards++;
    }
    if ((force[2] / MonoAxialMeanForceX) > 1.1)
    {
      step_Movement(cMOTOR_TWO, TOWARDS_THE_CENTER);
      preTens.singleStepTowards++;
    }

    if (millis() >= preTime )
    {
      preTime = 0;  // end of pre-tensioning
      select_motor(mn); // block motors that were selected initially
      display = false;
      //Serial.println("ok Pre-tensioning ended");
      Serial.print("ok Pre-tensioning ended ");
      Serial.print(preTens.singleStepAway); Serial.print(" ");
      Serial.print(preTens.incPreTens); Serial.print(" ");
      Serial.print(preTens.singleStepTowards); Serial.print(" ");
      Serial.println(MonoAxialMeanForceX);
      MOVE_STEP;  //digitalWrite(12, HIGH); // in order to tension canvas
      break;
    }
  }
}

/**
  @brief Execute the pre-tensioning for Y motors axis. The code for the pretensioning is execute inside
         this function, to speedup the procedure. The function return only when preTime is zero that means the
         at the end of the pre tensioning.
*/
void motor_PreTensioningY(void)
{
  int k;
  /* Just collect some stats about pre tensioning. */
  PreTensionMoveStat_TypeDef preTens;

  preTens.singleStepTowards = 0;
  preTens.incPreTens = 0;
  preTens.singleStepAway = 0;

  while ( preTime > 0)
  {
    k = 2;
    if (Serial.available())
    {
      process_line();
    }

    printPos();
    /* */
    if ( MonoAxialMeanForceY > max_force)
    {
      moving = false; // security feature
      Serial.print("ok Pre-tensY ended max_force ");
      Serial.println(MonoAxialMeanForceY);
    }

    // Y axis
    if (force[0] < preY2)
    {
      step_Movement(cMOTOR_ZERO, AWAY_FROM_CENTER);
      k--;
      preTens.singleStepAway++;
    }

    if (force[3] < preY2)
    {
      step_Movement(cMOTOR_THREE, AWAY_FROM_CENTER);
      k--;
      preTens.singleStepAway++;
    }

    //
    if (k == 2)
    { // if none moves then it is time to increase the limit
      if (preY2 < preY)
      {
        preY2 += cPREY2_INC;
        preTens.incPreTens++;
      }
    }
    // The MonoAxialMeanForceY var is calculated inside the printPos function
    if ((force[0] / MonoAxialMeanForceY) > 1.1)
    {
      step_Movement(cMOTOR_ZERO, TOWARDS_THE_CENTER);
      preTens.singleStepTowards++;
    }

    if ((force[3] / MonoAxialMeanForceY) > 1.1)
    {
      step_Movement(cMOTOR_THREE, TOWARDS_THE_CENTER);
      preTens.singleStepTowards++;
    }

    if (millis() >= preTime )
    {
      preTime = 0;  // end of pre-tensioning
      select_motor(mn); // block motors that were selected initially
      display = false;
      //Serial.println("ok Pre-tensioning ended");
      Serial.print("ok Pre-tensioning ended ");
      Serial.print(preTens.singleStepAway); Serial.print(" ");
      Serial.print(preTens.incPreTens); Serial.print(" ");
      Serial.print(preTens.singleStepTowards); Serial.print(" ");
      Serial.println(MonoAxialMeanForceY);
      MOVE_STEP;  //digitalWrite(12, HIGH); // in order to tension canvas
      break;
    }
  }
}

/**
  @brief Execute the pre-tensioning for X and Y motors axes
*/
void motor_PreTensioningXY(void)
{
  int k = 4;


  float average = (force[0] + force[1] + force[2] + force[3]) / 4;

  // Y axis
  if (force[0] < preY)
  {
    step_Movement(cMOTOR_ZERO, AWAY_FROM_CENTER);
    k--;
  }

  if (force[3] < preY)
  {
    step_Movement(cMOTOR_THREE, AWAY_FROM_CENTER);
    k--;
  }

  // X axis
  if (force[1] < preX)
  {
    step_Movement(cMOTOR_ONE, AWAY_FROM_CENTER);
    k--;
  }

  if (force[2] < preX)
  {
    step_Movement(cMOTOR_TWO, AWAY_FROM_CENTER);
    k--;
  }

  //
  //  if(k==4)
  //  { // if none moves then it is time to increase the limit
  //    if (preX2<preX)
  //    {
  //      preX2 += cPREX2_INC;
  //    }
  //    if (preY2<preY)
  //    {
  //      preY2 += cPREY2_INC;
  //    }
  //  }



  for (int i = 0; i < 4; i++)
  {
    if ((force[i] / average) > 1.1)
    {
      byte msk = (1 << i);
      step_Movement(msk, TOWARDS_THE_CENTER);
      k--;
    }
  }

  if (k == 4) { // no motion happened during this cycle == idle
    pretension_idle--;
  }
  else pretension_idle = 4; // reset if anything moved

  if (millis() >= preTime || pretension_idle <= 0 )
  {
    preTime = 0;  // end of pre-tensioning
    select_motor(mn); // block motors that were selected initially
    display = false;
    Serial.println("ok Pre-tensioning ended");
    MOVE_STEP;  //digitalWrite(12, HIGH); // in order to tension canvas
  }
}

/**
  @brief Entry point of step motor execution. This function set the direction and
         select the right function to finalize the step movement on the motor.
         Single step motor function check for the end point is reached.
  @param[in] m   motor/motors to move as for mask value
  @param[in] dir direction, can assume values:
                TOWARDS_THE_CENTER   0
                AWAY_FROM_CENTER     1
*/
void step_Movement( byte m, uint8_t dir)
{
  byte tmp = mask;

  MOVE_COND(dir);
  mask = m;

  switch (m)
  {
    case 0x0f:
      step_MotorXY();
      break;
    case cMOTOR_ZERO:     // 0x01
    case cMOTOR_ONE:      // 0x02:
    case cMOTOR_TWO:      // 0x04:
    case cMOTOR_THREE:    // 0x08:
      step_Motor( m);
      break;
    case 0x06:
      step_MotorX();
      break;
    case 0x09:
      step_MotorY();
      break;
    default:
      break;
  }
  mask = tmp;
  if (preTime > 0 && dir != TOWARDS_THE_CENTER) { // I need to read rawData again
    getRawData();
  }
}

/**
  @brief Execute the X axis motor step movement. The direction must be impose from the caller.
*/
void step_MotorX( void)
{
  // X axie
  if ( EndStop1_State == 0)
  {
    mask = cMOTOR_ONE;
    PORTC = mask;
    delay(1);
    PORTC = 0;
  }

  if ( EndStop2_State == 0)
  {
    mask = cMOTOR_TWO;
    PORTC = mask;
    delay(1);
    PORTC = 0;
  }
}

/**
  @brief Execute the Y axis motor step movement. The direction must be impose from the caller.
*/
void step_MotorY( void)
{
  // Y axie
  if ( EndStop0_State == 0)
  {
    mask = cMOTOR_ZERO;
    PORTC = mask;
    delay(1);
    PORTC = 0;
  }

  if ( EndStop3_State == 0)
  {
    mask = cMOTOR_THREE;
    PORTC = mask;
    delay(1);
    PORTC = 0;
  }
}

/**
  @brief Execute single motor step movement. The direction must be impose from the caller.
  @param[in]  m  single motor mask as for mask value.
*/
void step_Motor( byte m)
{
  if ( (EndPointMask & m) == 0)
  {
    PORTC = m;
    delay(1);
    PORTC = 0;
  }
}

/**
   @brief Move all motors axes of one step, checking if the end point reached.
          The direction must be impose from the caller.

*/
void step_MotorXY(void)
{
  // Y axie
  if ( EndStop0_State == 0)
  {
    mask = cMOTOR_ZERO;
    PORTC = mask;
    delay(1);
    PORTC = 0;
  }

  if ( EndStop3_State == 0)
  {
    mask = cMOTOR_THREE;
    PORTC = mask;
    delay(1);
    PORTC = 0;
  }
  // X axie
  if ( EndStop1_State == 0)
  {
    mask = cMOTOR_ONE;
    PORTC = mask;
    delay(1);
    PORTC = 0;
  }

  if ( EndStop2_State == 0)
  {
    mask = cMOTOR_TWO;
    PORTC = mask;
    delay(1);
    PORTC = 0;
  }
}

/**
   @brief This funcion move the motor far from the endpoint
   @param m mask
   @param direction.
*/
void farFromEndStop_Move( byte m, uint8_t dir)
{
  switch (m)
  {
    case 0x0F:
      farFromEndStopXY( dir);
      break;
    case cMOTOR_ZERO:     // 0x01
    case cMOTOR_ONE:    // 0x02;
    case cMOTOR_TWO:    // 0x04;
    case cMOTOR_THREE:  // 0x08; ln
      Serial.print("farFromEndStop ");
      Serial.print(m); Serial.print(" ");
      Serial.print(dir);
      farFromEndStop( m, dir);
      break;
    case 0x06:
      farFromEndStopX( dir);
      break;              // b0110
    case 0x09:
      farFromEndStopY( dir);
      break;              // b1001
    default:
      break;
  }
}

/**
   @brief

   @param dir 0 = towards the center, 1 = away from the center
*/
void farFromEndStopXY( uint8_t dir)
{
  boolean run = true;

  if (dir == 0)
    MOVE_COND(1);
  else
    MOVE_COND(0);

  //MOVE_COND(!dir);

  run = true;
  while ( run)
  {
    run = false;
    // Y axis
    if ( EndStop0_State > 0)
    {
      PORTC = cMOTOR_ZERO;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }

    if ( EndStop3_State > 0)
    {
      PORTC = cMOTOR_THREE;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }
    // X axis
    if ( EndStop1_State > 0)
    {
      PORTC = cMOTOR_ONE;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }

    if ( EndStop2_State > 0)
    {
      PORTC = cMOTOR_TWO;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }
  }
}

/**
   @brief Move X axis to home.

   @param dir 0 = towards the center, 1 = away from the center
*/
void farFromEndStopX( uint8_t dir)
{
  boolean run = true;

  if (dir == 0)
    MOVE_COND(1);
  else
    MOVE_COND(0);

  //MOVE_COND(!dir);

  run = true;
  while ( run)
  {
    run = false;
    if ( EndStop1_State > 0)
    {
      PORTC = cMOTOR_ONE;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }

    if ( EndStop2_State > 0)
    {
      PORTC = cMOTOR_TWO;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }
  }
}


/**
   @brief Move Y axis to home

   @param dir 0 = towards the center, 1 = away from the center
*/
void farFromEndStopY( uint8_t dir)
{
  boolean run = true;

  if (dir == 0)
    MOVE_COND(1);
  else
    MOVE_COND(0);

  //MOVE_COND(!dir);

  run = true;
  while ( run)
  {
    run = false;
    if ( EndStop0_State > 0)
    {
      PORTC = cMOTOR_ZERO;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }

    if ( EndStop3_State > 0)
    {
      PORTC = cMOTOR_THREE;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }
  }
}

/**
  @brief Move *single* motor to home position

  @param[in] m single motor activatin: 0x1, 0x2, 0x4, 0x8, 0x6, 0x9, 0xF
  @param[in] dir 0 = towards the center, 1 = away from the center
*/
void farFromEndStop( byte m, uint8_t dir)
{
  boolean run;

  if (dir == 0)
    MOVE_COND(1);
  else
    MOVE_COND(0);

  //MOVE_COND(!dir);
  run = true;
  while ( run)
  {
    run = false;
    if ( (EndPointMask & m) > 0)
    {
      PORTC = m;
      delayMicroseconds(20);
      PORTC = 0;
      delay(1);
      run = true;
    }
  }
}
