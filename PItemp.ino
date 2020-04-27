/*
$Id: PItemp.ino 113 2012-09-09 22:49:16Z aaron $
$URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/PItemp/PItemp.ino $

PI controller for temperature.

Assumes that the output control variable is positive correlated with plant
temperature.

In this case, the controlled input is a fan speed.
The problem with fans is that they have a minimum speed at which they
will operate.  If the commanded speed is below this level, it will
revert to a bang-bang mode.

The PI controller is computed on a continuous time basis,
and is robust to changing update times, which will happen
when we enter the bang-bang regime.

Type "show;" to the serial port to display commands,
and state variables that can be set.
"+;" or "-;" to increase/decrease temperature setting, one count per symbol

Set "MotorSpeed" to  open loop and manually command fan speed.
Set "TempCounts" to set desired temperature, and let computer set fan speed.
*/

#include "PItemp.h"  // struct defns

const char *PItemp_ID = "$Rev: 113 $";

//----------------------------------------------------------------------
PID_State State;  // persistent on EEPROM
float prevSet;    // LPF temp set pot
float prevTemp;   // LPF temperature sensor
const int SerialRate = 9600;
int PrevFanSpeed = 0;  // assume we started with fan off
int OpenLoopSpeed = 0;
int DisplayTempC = 0;  // default F
//----------------------------------------------------------------------

#include <ioUtil.h>
SerialLineBuffer SerialCmd;

#include <EEPROM.h>  // we seem to need this to get EEPROM library included in project

unsigned long count=0; // count of total updates processed

//====================================== DEFAULT INITIAL SETTINGS BELOW!

void reset()  // restore state to default values
{
  State.Motor.minSpeed   = 30;
  State.Motor.startSpeed = 200;
  State.Motor.startTime  = 2000;

  State.PID.gain = 1;
  State.PID.Ti   = 10;
  State.PID.u    = 0;
  State.PID.Int  = 0;

  // LPF for sensor readings. close to 0 for more smoothing, 1=no filtering
  State.LPF_Alpha = 0.02;  
  State.LogPeriod = 0.2;  // minutes
  State.LoopClosed = false;
  State.AnalogCommand = false;  // temp set point from pot, not serial command
  init("RESET");
  updateStoredState(&State);
}

//------------------------------------------- Initialize hardware states

void setup()
{
  // analog input pins do not need setup
  pinMode(PIN_PWM_TEMP_DISPLAY,OUTPUT);
  pinMode(PIN_PWM_FAN_SPEED,   OUTPUT);
  pinMode(PIN_LED_INCREASE, OUTPUT);
  pinMode(PIN_LED_DECREASE, OUTPUT);
  pinMode(PIN_LED_HEARTBEAT,OUTPUT);  // diagnostic output light pin
  Serial.begin(SerialRate);
  restore();
  //State.LoopClosed=false;  // open command loop on boot while debugging
  State.PID.Int = 0;  // always start with 0 integrator.  not saving it as part of "state" anymore
}

void init(const char *comment)
{
  OpenLoopSpeed = 0;
  //PID_Int = initIntegrator(&(State.PID));
  prevSet  = analogRead(PIN_SENSE_CMD );
  prevTemp = analogRead(PIN_SENSE_TEMP);
  PrevFanSpeed = 0;  // assume we start with fan off

  analogWrite(PIN_PWM_FAN_SPEED,200);
  digitalWrite(PIN_LED_INCREASE,HIGH);
  digitalWrite(PIN_LED_DECREASE,HIGH);

  Serial.print(F("### "));
  Serial.println(comment);
  printState(&State);
}

void restore()  // restore state to stored value, if possible
{
  loadState(&State);
  if (!validState(&State))
    {
      reset();
      return;
    }
  init("RESTORE");
}

float getTempReading()
{
  int val = analogRead(PIN_SENSE_TEMP);
  float smoothVal = State.LPF_Alpha * val + prevTemp * (1.0-State.LPF_Alpha);
  prevTemp = smoothVal;

  // update temperature display output
  analogWrite(PIN_PWM_TEMP_DISPLAY,clamp8(0.25*smoothVal));
  return(smoothVal);
}

float prevSetPrint = -999;
float getSetPoint()  // from pot
{
  int val = analogRead(PIN_SENSE_CMD);
  float smoothVal = State.LPF_Alpha * val + prevSet * (1.0-State.LPF_Alpha);

  float e = prevSetPrint - smoothVal;
  if (ABS(e) > 10)
    {
      prevSetPrint = smoothVal;
      Serial.print("# Desired temperature ");Serial.print(smoothVal,1);Serial.println(" counts");
    }

  prevSet = smoothVal;
  return(smoothVal);
}

void setSpeed(int speed)
{
  int lo = State.Motor.minSpeed * .7; // allow some hysterysis
  if (lo < 3) lo = 3;
  if (speed <= lo)
    { // too slow to try.  just turn off.
      setFanSpeed(0);
      PrevFanSpeed = 0;
      //setHeatingSlopeLED(-1);
      return;
    }

  if (PrevFanSpeed > lo)
    { // fan is blowing, just change speed.
      setFanSpeed(speed);
      //setHeatingSlopeLED((PrevFanSpeed<=speed)?-1:1);
      PrevFanSpeed = speed;
      return;
    }

  // fan was off

  if (speed > State.Motor.minSpeed)
    { // kick fan on again
      setFanSpeed(State.Motor.startSpeed);
      Serial.println(F("Kickstarting Fan..."));
      delay(State.Motor.startTime);
    }
  else speed=0;  // force fan off,... shouldn't be necessary, but make sure
  setFanSpeed(speed);
  //setHeatingSlopeLED((PrevFanSpeed<=speed)?1:-1);

  PrevFanSpeed = speed;
}

int tweakSetPoint(char *key)
{
  if (!((*key=='-') || (*key=='+'))) return(0);  // no change
  int changeCount = 0;
  while (*key=='+') { key++; changeCount++; }
  while (*key=='-') { key++; changeCount--; }
  Serial.print(F("#SetPoint increase "));
  Serial.print(changeCount);
  Serial.println(F(" counts"));
  State.PID.u += changeCount;
  if (State.PID.u <    0) State.PID.u = 0;
  if (State.PID.u > 1024) State.PID.u = 1023.01;
  Serial.print(F("TempCounts "));
  Serial.println(State.PID.u,2);
  updateStoredState(&State);  // If PID.u changed, make sure it was stored
  return(changeCount);
}

enum CMD_CODE { ID_UNDEF, ID_TempCounts, ID_TempC, ID_TempF, 
   ID_Ti, ID_gain, ID_kick, ID_show, ID_save, ID_reset,
   ID_IntegratorValue, ID_MotorMin, ID_MotorStartSpeed,
   ID_MotorStartTime, ID_LogPeriod, ID_count, ID_LPF, ID_about, ID_help,
   ID_MotorSpeed
};
const char *STR[] = { "Undefined",
  "TempCounts", "TempC", "TempF", "Ti", "gain", "kick", "show",
  "save", "reset", "IntegratorValue", "MotorMin", "MotorStartSpeed",
  "MotorStartTime", "LogPeriod", "count", "LPF", "about", "help",
  "MotorSpeed"
};
void printTab() { Serial.print('\t'); }
void printSTR(const int id) { Serial.print(STR[id]); }
bool keyMatch(const char *key, const int idSTR) { return(keyMatch(key,STR[idSTR])); }


void processCommand()
{
  char *key,*val,*cmd;
  //static SerialLineBuffer cmd;  // doesn't work like this on Arduino gcc-avr
  if (!SerialCmd.isComplete()) return;
  cmd = SerialCmd.get();
  key = extractKey(cmd,&val);
  if ((key==NULL) || (key[0]==0)) return;  // blank line
  Serial.print(F("#CMD> "));Serial.print(key);
  if (val && *val) {Serial.print("\t");Serial.println(val);}
  else Serial.println("");

  if(tweakSetPoint(key)) {}
  else if (keyMatch(key,ID_TempCounts))
    { // setting for desired control point (counts)
      float cmd = State.PID.u;
      setState(key,val,cmd);
      if (cmd < 0)
          State.AnalogCommand = true;  // analog command. leave PID.u alone
      else
        {
          State.PID.u = cmd;
          State.AnalogCommand = false;
        }
      State.LoopClosed = true;
    }
  else if (keyMatch(key,ID_TempC) || keyMatch(key,ID_TempF))
    {
      float cmd, counts;
      setState(key,val,cmd);
      DisplayTempC = keyMatch(key,ID_TempC);
      counts = DisplayTempC ? counts_tempC(cmd) : counts_tempF(cmd);
      if ((counts < 2) || (counts > 1020))
        {
          Serial.print(F("#Ignoring Temperature counts setting of "));
          Serial.print(counts);
          Serial.println(F(" out of range."));
          return;
        }
      State.AnalogCommand = false;
      State.LoopClosed = true;
      State.PID.u = counts;
    }
  else if (keyMatch(key,ID_Ti) || keyMatch(key,ID_gain))
    { // try to maintain integrator portion of command under Ti/gain change
      float intCmd = State.PID.Int * State.PID.gain / State.PID.Ti;
      if  (keyMatch(key,ID_Ti)) setState(key,val,State.PID.Ti);
      else                      setState(key,val,State.PID.gain);
      State.PID.Int = intCmd * State.PID.Ti / State.PID.gain;
    }
  else if (keyMatch(key,ID_kick))
    { // kick cmd level by given amount, by changing integrator level
      float cmd = 0;
      setState(key,val,cmd);
      cmd += State.PID.Int * State.PID.gain / State.PID.Ti;
      State.PID.Int = cmd * State.PID.Ti / State.PID.gain;
    }
  else if (keyMatch(key,ID_show )) { printState(&State); return; }
  else if (keyMatch(key,ID_save )) {} // just trigger save State
  else if (keyMatch(key,ID_reset)) reset();
  else if (keyMatch(key,ID_IntegratorValue)) setState(key,val,State.PID.Int);
  else if (keyMatch(key,ID_MotorMin       )) setState(key,val,State.Motor.minSpeed);
  else if (keyMatch(key,ID_MotorStartSpeed)) setState(key,val,State.Motor.startSpeed);
  else if (keyMatch(key,ID_MotorStartTime )) setState(key,val,State.Motor.startTime);
  else if (keyMatch(key,ID_LogPeriod))       setState(key,val,State.LogPeriod);
  else if (keyMatch(key,ID_count)) Serial.println(count);
  else if (keyMatch(key,ID_LPF))
    {
      float alpha=0;
      setState(key,val,alpha);
      State.LPF_Alpha = clipLimit(alpha,0.000001,1);
      if (alpha > 1) alpha = 1;
      if (alpha != State.LPF_Alpha)
        {
          Serial.print(F("# LPF "));
          Serial.print(State.LPF_Alpha);
          Serial.print(F(" out of (0,1] limit.  setting LPF to "));
          Serial.println(alpha,6);
          State.LPF_Alpha = alpha;
        }
    }
  else if (keyMatch(key,ID_MotorSpeed))
    {
      setState(key,val,OpenLoopSpeed);
      OpenLoopSpeed = clamp8(OpenLoopSpeed);
      State.LoopClosed = false;
    }
  else if (keyMatch(key,ID_about)) Serial.println(PItemp_ID);
  else if (keyMatch(key,ID_help) || (*key=='?'))
    {
      Serial.println(F("show : will show state, and control variable settings."));
      Serial.print(F("Commands NOT shown by \"show\" :"));
      Serial.println(F("+/-\t|turn temp up/down"));
      Serial.println(F("TempC\t|Set closed loop temperature, deg C"));
      Serial.println(F("TempF\t|Set closed loop temperature, deg F"));
      Serial.println(F("kick X\t|Kick motor speed X counts, by spoofing integrator"));
      Serial.println(F("save\t|Save current state"));
      Serial.println(F("reset\t|Reset state to defaults"));
      Serial.print(F("about\t|print version : "));
      Serial.println(PItemp_ID);
      Serial.println(F("? or help : print this message"));
    }
  else
    {
      Serial.print("#Unrecognized command \"");
      Serial.print(key);
      Serial.println("\"");
      return;
    }
  updateStoredState(&State);  // if a persistant state param changed, store it
}

// print log update
void printLogLine(const float t, const float u, const float y, const float c, const float Integrator)
{
  float tCmd, tSens;
  if (DisplayTempC)
    {
      tCmd = tempC_counts(u);
      tSens= tempC_counts(y);
    }
  else
    {
      tCmd = tempF_counts(u);
      tSens= tempF_counts(y);
    }
  Serial.print(t,2);
  printTab(); Serial.print(tCmd,0);
  printTab(); Serial.print(tSens,2);
  printTab(); Serial.print(c,0); // motor speed command?
  printTab(); Serial.println(Integrator,2);
}

// just turn over/under lights both on for open loop now.
void updateOpenLoopDisplay()
{
  digitalWrite(PIN_LED_INCREASE,HIGH);
  digitalWrite(PIN_LED_DECREASE,HIGH);
}

float prevOpenLogT = -99;
void updateOpenLoopLog(const float t,const float temp)
{
  //// user beware.  speed command may not work... if motor stalls
  //setFanSpeed(OpenLoopSpeed);
  setSpeed(OpenLoopSpeed);
  
  //static float prevT = -99;
  float dt = t - prevOpenLogT;
  if (dt > State.LogPeriod)
    {
      printLogLine(t,State.PID.u,temp,OpenLoopSpeed,State.PID.Int);

/* old print, not same cols as closed loop
      Serial.print(count);Serial.print("\t");
      Serial.print(t,2);
      Serial.print("\t");
      Serial.print(temp,2);
      Serial.print("\t");
      Serial.println(OpenLoopSpeed);
*/
      prevOpenLogT = t;
      updateOpenLoopDisplay();//updateFlashingLEDs();
    }
}

//================================ This is where the control law happens
float prevUpdateT = -99;
float prevLogT = -99;
int updateCommand(float t, float y)
{
  float err = State.PID.u - y;
  float dt = (prevUpdateT>0) ? t - prevUpdateT : 0;
  prevUpdateT = t;
  State.PID.Int += err * dt;

  // Limit integrator s.t. the I term can saturate the command itself, but no more
  State.PID.Int = clipLimit(State.PID.Int,0,256.*State.PID.Ti/State.PID.gain);

  float cmd = State.PID.gain * (err + State.PID.Int/State.PID.Ti);

  int c = clamp8(cmd);
  if (t - prevLogT > State.LogPeriod)
    {
      printLogLine(t,State.PID.u,y,c,State.PID.Int);
      prevLogT = t;
    }
  setSpeed(c);
  setHeatingSlopeLED(err*0.1);
  return c;
}

//======================================================================
#define HEARTBEAT_PERIOD 20
void loop()
{
  count++;
  if (count%HEARTBEAT_PERIOD==0)digitalWrite(PIN_LED_HEARTBEAT,((count/HEARTBEAT_PERIOD)&1)?HIGH:LOW);

  processCommand();  // check for updates to parameters from serial port

  float temp = getTempReading();
  float t = getElapsedMinutes();
  if (State.LoopClosed)
    {
      if (State.AnalogCommand) State.PID.u = getSetPoint();
      updateCommand(t,temp);
    }
  else
    updateOpenLoopLog(t,temp);

  delay(15);
}

//======================================================================

void printState(PID_State *s)
{
  // print temp, command, integrator, dy/dt
  Serial.print(F("\n#State at "));
  Serial.print(getElapsedMinutes(),2);
  Serial.print(F("\tTemp="));
  Serial.print(getTempReading(),1);
  Serial.print(F("(counts)  Speed="));
  Serial.println(PrevFanSpeed);

  printSTR(ID_TempCounts); printTab();
  Serial.print(s->PID.u);
  Serial.print(F("\t# Desired temperature (counts)"));
  Serial.println(s->AnalogCommand ? 
                 F(" From pot.  Set this to disable pot") :
                 F(" Set to -1 ==> switch to pot"));

  Serial.print(F("IntegratorValue\t"));
  Serial.println(s->PID.Int);

  printSTR(ID_gain); printTab();
  Serial.print(s->PID.gain);
  Serial.println(F("\t# Proportional (err) gain"));

  printSTR(ID_Ti); printTab();
  Serial.print(s->PID.Ti,1);
  Serial.println(F("\t# Integration time (min)"));

  printSTR(ID_MotorMin); printTab();
  Serial.print(s->Motor.minSpeed);
  Serial.println(F("\t# minimum sustainable fan speed command"));

  printSTR(ID_MotorStartSpeed); printTab();
  Serial.print(s->Motor.startSpeed);
  Serial.println(F("\t# fan speed command to start fan"));

  printSTR(ID_MotorStartTime); printTab();
  Serial.print(s->Motor.startTime);
  Serial.println(F("\t# time (ms) to run fan at StartSpeed to get it running"));Serial.flush();

  printSTR(ID_LPF); printTab();
  Serial.print(s->LPF_Alpha);
  Serial.println(F("\t# Sensor LPF setting (0,1], 1=no filter, small==much filtering"));

  printSTR(ID_LogPeriod); printTab();
  Serial.print(s->LogPeriod);
  Serial.println(F("\t# time between log outputs (minutes)"));

  printSTR(ID_MotorSpeed); printTab();
  Serial.print(OpenLoopSpeed);
  Serial.println(F("\t# default motor speed, in open-loop mode"));
  
  if (State.LoopClosed)
  {
    Serial.print(F("# Loop closed.   Temperature setting from "));
    Serial.println(State.AnalogCommand?F("analog pot (TempCounts set <0)"):F("TempCounts command\n"));
  }
  else Serial.println(F("# Loop is open, Motor set to above speed.\n"));
  Serial.println(F("#minutes tCmd\ttemp\tspeed\tintegrator"));
  Serial.flush();  // wait for print to complete, Arduino>=1.0, purge print else
}

