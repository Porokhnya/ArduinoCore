//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "CorePinStateAction.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CorePinStateAction
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CorePinStateAction::CorePinStateAction()
{
  flags.isHoldOnTimer = true;
  flags.isActive = true;
  tTimer = 0;
  timerPast = 0;
  flags.lastPinState = 3;
  memset(&Settings,0,sizeof(Settings));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void CorePinStateAction::on()
{
  if(!Settings.Pin)
    return;

  if(flags.lastPinState != flags.stateOn)
  {
    flags.lastPinState = flags.stateOn;
    digitalWrite(Settings.Pin,flags.stateOn);
  }
     
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void CorePinStateAction::off()
{
  if(!Settings.Pin)
    return;

  if(flags.lastPinState !=  flags.stateOff)
  {
    flags.lastPinState =  flags.stateOff;
    digitalWrite(Settings.Pin, flags.stateOff);
  }
     
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void CorePinStateAction::init(byte pin, uint16_t HoldOnTime, uint16_t HoldOffTime, byte OnLevel, byte OffLevel)
{

 flags.stateOn = OnLevel;
 flags.stateOff = OffLevel;
 Settings.Pin = pin;
 Settings.HoldOnTime = HoldOnTime;
 Settings.HoldOffTime = HoldOffTime;
  
  if(Settings.Pin)
  {
    pinMode(Settings.Pin, OUTPUT);
    
    if(isActive())
      on();
    else
      off();
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool CorePinStateAction::isActive()
{
 return flags.isActive;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void CorePinStateAction::update()
{
  if(!isActive()) // таймер неактивен, выключаем пин и выходим
  {
    off();
    return;
  }

  unsigned long timerNow = millis();
  uint16_t dt = timerNow - timerPast;
  timerPast = timerNow;

  // прибавляем дельту
  tTimer += dt;
  unsigned long tCompare = 0;

  // теперь смотрим, какой интервал мы обрабатываем
  if(flags.isHoldOnTimer)
  {
    // ждём истекания интервала включения
    tCompare = Settings.HoldOnTime;
    
    if(tTimer > tCompare)
    {
      off();
      tTimer = 0;
      flags.isHoldOnTimer = false;
    }
  }
  else
  {
    tCompare = Settings.HoldOffTime;
    
      if(tTimer > tCompare)
      {
        on();
        tTimer = 0;
        flags.isHoldOnTimer = true;
      }
  } // else
  
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

