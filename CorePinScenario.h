#pragma once
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "CoreConfig.h"
#include "CoreArray.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// класс для выполнения периодической смены состояний на цифровых пинах по сценариям
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint8_t pin;          // номер пина, на котором устанавливать нужный уровень
  uint8_t level;        // уровень, который устанавливать на пине
  uint16_t duration;    // длительность удержания уровня, миллисекунд
  
} CorePinAction;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CorePinAction> CorePinActionsList;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class CorePinScenario
{
    public:

      CorePinScenario();

      bool enabled();
      void enable();
      void disable();
      void clear();
      void update();
      void add(CorePinAction action);

    private:

          bool isEnabled;
          CorePinActionsList* actions;
          size_t currentActionIndex;
          unsigned long timer;
  
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

