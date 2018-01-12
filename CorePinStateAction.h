#ifndef _CORE_PIN_STATE_ACTION_H
#define _CORE_PIN_STATE_ACTION_H
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "CoreConfig.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// класс таймера
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// структура таймера
typedef struct
{
  uint8_t Pin; // номер пина, на который таймер будет выдавать сигнал
  uint16_t HoldOnTime; // сколько времени держать сигнал "включено", в миллисекундах
  uint16_t HoldOffTime; // сколько времени держать сигнал "выключено", в миллисекундах
    
} CorePinStateSettings;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  bool isHoldOnTimer : 1; // если true - то ждём истечения периода включения, иначе - истечение периода выключения
  uint8_t lastPinState : 2;
  uint8_t isActive : 1;
  uint8_t stateOn : 2; 
  uint8_t stateOff : 2; 
  
} CorePinStateFlags;
//--------------------------------------------------------------------------------------------------------------------------------
class CorePinStateAction
{
 public:
  CorePinStateAction();
  
  CorePinStateSettings Settings; // настройки таймера
  
  void update(); // обновляет таймер

  bool isActive(); // возвращает true, если таймер активен
  void init(uint8_t pin, uint16_t HoldOnTime, uint16_t HoldOffTime, uint8_t OnLevel=HIGH, uint8_t OffLevel=LOW); // инициализирует таймер

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
