/*
$Id: PItemp.h 45 2012-03-03 18:37:30Z aaron $
$URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/PItemp/PItemp.h $

Structures and definitions used in tempPID.ino

It appears that Arduino makes prototypes for all functions
at TOP of file, BEFORE structure definitions.
Hence, we can't just define any structs used as function parameters
in the .ino file.  It needs to be in a separate .h, #include'd early so that
Arduino has a chance of defining these before they are used.
*/

#define PIN_PWM_FAN_SPEED     3
#define PIN_PWM_TEMP_DISPLAY  6
#define PIN_LED_INCREASE      7
#define PIN_LED_DECREASE      4
#define PIN_LED_HEARTBEAT    13

#define PIN_SENSE_TEMP 0
#define PIN_SENSE_CMD  1

/* poly from Temperature (F) to ln(R) */
#define TPOLY_F_A (3.0533e-5)   /* These are obtained from calibration */
#define TPOLY_F_B (-0.030109)   /* of thermistor probe */
#define TPOLY_F_C 7.584
#define RT0 19500//19500 /*22000*/  /* resistor on thermistor divider */

#define R_COUNTS(x) (RT0*(1023/(x)-1))  /* Resistance at given sensor counts */

/* Calibration to cooking thermometer directly from ATD output
   C to counts */
#define CPOLY0  32.1489
#define CPOLY1   1.05929815
#define CPOLY2   0.049783756
#define CPOLY3  (-0.000088614784)

typedef struct MotorInfo_s {
  int minSpeed;    // motor stops spinning at this level, switch to wide-scale PWM [0..255]
  int startSpeed;  // min cmd needed to start motor from stop
  int startTime;   // milliseconds needed to start motor
} MotorInfo;

typedef struct PID_param_s {
  float gain;  /// gain for error term
  float Ti;    /// Integration time, close to time constant (+delay?)
  float u;     /// current control input, try to make temp this value
  float Int;   /// last recorded integrator level
} PID_param;

// Persistent state settings
typedef struct PID_State_s {
  MotorInfo Motor;
  PID_param  PID;

  // LPF for sensor readings. 1=no filtering
  //                 close to 0=much filtering
  float LPF_Alpha;
  float LogPeriod;  // minutes
  bool LoopClosed;
  bool AnalogCommand;  // temp set point from pot, not serial command
} PID_State;

#define INRANGE(i,lo,hi) (((i)>=(lo))&&((i)<=(hi)))
#define ABS(x)  (((x)<0)?-(x):x)

//------------------- Free Functions
int clamp8(const float x);
float absLimit(float val, float thresh);
float clipLimit(float val, float lo, float hi);
void loadState(PID_State *s);
void updateStoredState(PID_State *CurrentState);
bool validState(PID_State *s);
float getElapsedMinutes();  // in minutes
void setHeatingSlopeLED(float slope);
void setFanSpeed(int cmd);
void setState(const char *key, const char *valStr, float &val);
void setState(const char *key, const char *valStr,   int &val);
//void updateFlashingLEDs();

float tempF_counts(float counts);  /* temp(F) from counts */
float counts_tempF(float tempF);   /* counts from tempF */
float counts_tempC(float tempC);
float tempC_counts(float counts);
