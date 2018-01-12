#ifndef _CORE_PERIODIC_PIN_ACTION_H
#define _CORE_PERIODIC_PIN_ACTION_H
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "CoreConfig.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// класс таймера
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// структура таймера
typedef struct
{
  byte Pin; // номер пина, на который таймер будет выдавать сигнал
  uint16_t HoldOnTime; // сколько времени держать сигнал "включено", в миллисекундах
  uint16_t HoldOffTime; // сколько времени держать сигнал "выключено", в миллисекундах
    
} CorePinStateSettings;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  bool isHoldOnTimer : 1; // если true - то ждём истечения периода включения, иначе - истечение периода выключения
  byte lastPinState : 2;
  byte isActive : 1;
  byte stateOn : 2; 
  byte stateOff : 2; 
  
} CorePinStateFlags;
//--------------------------------------------------------------------------------------------------------------------------------
class CorePinStateAction
{
 public:
  CorePinStateAction();
  
  CorePinStateSettings Settings; // настройки таймера
  
  void update(); // обновляет таймер

  bool isActive(); // возвращает true, если таймер активен
  void init(byte pin, uint16_t HoldOnTime, uint16_t HoldOffTime, byte OnLevel=HIGH, byte OffLevel=LOW); // инициализирует таймер

  void on();
  void off();

  void enable(){ flags.isActive = true; }
  void disable(){ flags.isActive = false; }

private:

  CorePinStateFlags flags;
  unsigned long timerPast, tTimer; // таймеры
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
