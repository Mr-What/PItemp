/*
$Id: util.cpp 113 2012-09-09 22:49:16Z aaron $
$URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/PItemp/util.cpp $

Misc utilities which can be removed from tempPID, for ease of maintenance.
*/

#include "PItemp.h"  // struct defns

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <EEPROM.h>

//======================================================================
// clamp to 8-bit value
int clamp8(const float x)
{
  int i = (int)(x+0.5);
  if (i <  0 ) i=0;
  if (i > 255) i=255;
  return(i);
}

// limit absolute value to given thresh
float absLimit(float val, float thresh)
{
  if (val < -thresh) return(-thresh);
  if (val >  thresh) return( thresh);
  return val;
}

float clipLimit(float val, float lo, float hi)
{
  if (val < lo) return(lo);
  if (val > hi) return(hi);
  return(val);
}

void loadState(PID_State *s)
{
  int n = sizeof(*s);
  byte *b;
  b = (byte *)s;
  for (int i=0; i < n; i++, b++)
    *b = EEPROM.read(i);

  // decided to stop saving integrator.  Just start at 0?
  s->PID.Int=0;
}

// update any bytes of stored state that do not match current state
void updateStoredState(PID_State *myState)
{
  PID_State s;
  loadState(&s);

  // fool system into NOT saving PID.u setting when in analog command mode
  if (myState->AnalogCommand) s.PID.u = myState->PID.u;

  // fool system into NOT saving integrator
  s.PID.Int=myState->PID.Int;

  byte *stored, *current;
  int n = sizeof(s);
  stored  = (byte *)(&s);
  current = (byte *)(myState);
  for (int i=0; i < n; i++)
    {
      if (*stored != *current)
        {
          EEPROM.write(i,*current);
          Serial.print("#State byte ");
          Serial.print(i);
          Serial.println(" updated.");
        }
      current++;
      stored++;
    }
}

bool validState(PID_State *s)
{
  if (s->LPF_Alpha <= 0) return false;  
  if (s->LPF_Alpha > 1) return false;  
  if (!INRANGE(s->LogPeriod,    .001,10 )) return false;
  if (!INRANGE(s->Motor.minSpeed  ,0,255)) return false;
  if (!INRANGE(s->Motor.startSpeed,0,225)) return false;
  if (!INRANGE(s->Motor.startTime,1,50000))return false;
  if (!INRANGE(s->PID.gain,   .00001,100)) return false;
  if (!INRANGE(s->PID.Ti,        .01,500)) return false;
  if (!INRANGE(s->PID.u,0,1023)) return false;
  if (!INRANGE(s->PID.Int,0,99999)) return false;
  return true;
}

float getElapsedMinutes()  // in minutes
{
  unsigned long t_ms = millis();
  float t = t_ms * 0.0000166666666667;  // time in minutes
  return(t);
}

// set LED(s) indicating heating or cooling mode
//    If we need more PWM pins, switch these to digital pins,
//    and on/off displays.  Not using proportional display on these now.
void setHeatingSlopeLED(float slope)
{
  int val = 255*slope;
  val = clamp8(ABS(val));
  int pinActive;
  if (slope > 0)
    {
      digitalWrite(PIN_LED_INCREASE,LOW);
      pinActive =  PIN_LED_DECREASE;
    }
  else
    {
      digitalWrite(PIN_LED_DECREASE,LOW);
      pinActive =  PIN_LED_INCREASE;
    }
#ifdef PWM_HEATING_LEDs  // Defint this is heating LED outputs are PWM
  if (val < 250) analogWrite(pinActive,val);
  else
#endif
    digitalWrite(pinActive,HIGH);
}

// set fan speed, and update any status displays accordingly
void setFanSpeed(int cmd)
{
  analogWrite(PIN_PWM_FAN_SPEED, cmd);
}

void setState(const char *key, const char *valStr, float &val)
{
  if ( (valStr==NULL) || (valStr[0]==0) )
    {
      Serial.print("#Ignoring empty value for ");
      Serial.print(key);
      Serial.print(" command");
      return;
    }
  val = atof(valStr);
  Serial.print("\n");
  Serial.print(key);
  Serial.print("\t");
  Serial.println(val);
}

void setState(const char *key, const char *valStr, int &val)
{
  if ( (valStr==NULL) || (valStr[0]==0) )
    {
      Serial.print("#Ignoring empty value for ");
      Serial.print(key);
      Serial.print(" command");
      return;
    }
  val = atoi(valStr);
  Serial.print("\n");
  Serial.print(key);
  Serial.print("\t");
  Serial.println(val);
}

// flashing LED display while in open-loop mode
int FlashingLEDstate = 0;
int FlashingLEDcode[] = {0,1,3,2};  // grey code flasher
void updateFlashingLEDs()
{
  digitalWrite(PIN_LED_INCREASE,(FlashingLEDcode[FlashingLEDstate] & 1)?HIGH:LOW);
  digitalWrite(PIN_LED_DECREASE,(FlashingLEDcode[FlashingLEDstate] & 2)?HIGH:LOW);
  FlashingLEDstate++;
  if (FlashingLEDstate >= (sizeof(FlashingLEDcode)/sizeof(int))) FlashingLEDstate=0;
}

float tempF_counts(float c)  /* temp(F) from counts */
{
  float r = RT0*(1023./c-1);  /* Resistance of thermistor (ohms) from counts */
  float t = TPOLY_F_B*TPOLY_F_B - 4 * TPOLY_F_A * (TPOLY_F_C - log(r/1000)); /* disc */
  t = -(TPOLY_F_B + sqrt(t))/(TPOLY_F_A*2);
  return(t);
}
float counts_tempF(float tempF)   /* counts from tempF */
{
  //Serial.print("# temp ");Serial.print(tempF,1);Serial.println("F");
  float lnR = TPOLY_F_A;
  lnR *= tempF*tempF;
  //Serial.print("t2 part ");Serial.println(lnR,2);
  float x = tempF * TPOLY_F_B;
  //Serial.print("t part ");Serial.println(x,2);
  lnR += x + TPOLY_F_C;
  //Serial.println(TPOLY_F_C,3);
  ////float lnR = TPOLY_F_C + tempF*TPOLY_F_B + tempF*tempF*TPOLY_F_A;
  //Serial.println(lnR,4);
  x = exp(lnR)*1000;
  Serial.print("#desired thermistor resistance ");Serial.print(x,0);Serial.print(" Ohms\t");
  float c = 1023. * (RT0/(RT0+exp(lnR)*1000));
  Serial.print(c,1);Serial.println(" counts");
  return(c);
}
float counts_tempC(float C)
{
  float f = C*9/5.+32;
  return(counts_tempF(f));
}
float tempC_counts(float c)
{
  float f = tempF_counts(c);
  return((f-32)*5/9);
}
