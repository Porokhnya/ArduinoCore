#pragma once
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "Core.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SIGNALS_ENABLED
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define CORE_SIGNAL_BYTES 10 // кол-во байт под сигналы
extern uint8_t SIGNALS[CORE_SIGNAL_BYTES];
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  sopNone, 
  sopCompareNoData,   // сравнить с показаниями "нет данных"
  sopInterval,        // A between B and C
  sopEqual,           // ==
  sopLess,            // <
  sopLessOrEqual,     // <=
  sopGreater,         // >
  sopGreaterOrEqual,  // >=
  sopNotEqual,        // != 
  
} SignalOperands; // операнды для работы с правилом
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  saRaiseSignal, // взвести сигнал
  saResetSignal // сбросить сигнал
    
} SignalActions; // возможные действия в записи сигнала в конфиге
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
struct SignalOneAction
{
  SignalOneAction();
  ~SignalOneAction();
  
  uint8_t action;
  uint8_t actionDataLength;
  uint8_t* actionData;
  
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<SignalOneAction*> SignalActionsList;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
 Подумаем над типами записей в конфиге, управляющих сигналами:

  1. Слежение за показаниями в хранилище: активен (маска дней недели, диапазон времени), имя датчика (если не указано - ни за чем не следим), параметры сравнения (попадание в диапазон, совпадение, меньше, больше), 
     действия, например: установить сигналы (номера сигналов), сбросить сигналы (номера сигналов);

     структура хранения в конфиге выглядит так:

     SignalRecord,
     1 байт - длина данных всей записи (для быстрой итерации)
     1 байт - признак - работать ли по времени, если выставлен - то следом идут 5 байт уставок времени, иначе - сразу оператор сравнения
     1 байт - маска дней недели, когда сигнал активен
     1 байт - час начала активности
     1 байт - минута начала активности
     1 байт - час окончания активности
     1 байт - минута окончания активности
     1 байт - оператор сравнения (никакого, попадание в диапазон, совпадение, меньше, больше, меньше либо равно, больше либо равно)
     1 байт - длина данных, с которыми сравниваем
     N байт - данные для сравнения, тип хранимых данных зависит от типа показаний датчика (температура, влажность и т.п.). 
              Для составных датчиков (например, влажности, где есть и температура, и влажность - указываем здесь ещё признак - какое значение с датчика нужно сравнивать - температура или влажность)
     1 байт - кол-во действий
     далее - N записей по действиям:
      1 байт - тип действия (например, установить сигнал)
      1 байт - длина данных действия
      N байт - данные действия

    в конце - имя датчика, оканчивающееся '\0' - если не указано - работает без привязки в датчику, просто по времени

    Итого получаем, на примере одного действия и максимально длинного имени датчика в 9 символов - размер структуры без заголовка (SignalRecord)
    будет минимум 23 байта. Очевидно, что лучше не вычитывать напрямую, а хранить смещения адресов в EEPROM, и вычитывать только при проверке сигналов
     
     
 
 */
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// класс обработчика записи настроек сигнала в конфиге - разбирает запись и обрабатывает её
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
struct CoreStoredData;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class SignalHandler
{
  public:

    SignalHandler(uint16_t memoryAddress);

    void analyze();

  private:

      void executeActions(SignalActionsList& actions);
      
      bool compare(const String& sensorName, SignalOperands operand, uint8_t* data, uint8_t dataLength);

      bool compareTemperature(TemperatureData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength);
      bool compareHumidity(HumidityData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength);
      
      bool compareLuminosity(LuminosityData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength);
      
      bool compareDigitalPort(DigitalPortData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength);
      bool compareAnalogPort(AnalogPortData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength);
      
      bool compareUserData(CoreStoredData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength);

      bool compareNumber(int32_t num,SignalOperands operand,int32_t from, int32_t to);

      uint16_t memoryAddress;
  
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// класс сигнала
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class Signal
{
  public:
  
    Signal();
    
   operator bool();

   void reset(); // сбрасывает сигнал
   void raise(); // устанавливает сигнал

   void setNumber(uint8_t signalNumber);


   Signal& operator=(bool val);
   Signal& operator=(uint8_t val);
   Signal& operator=(uint16_t val);
   Signal& operator=(uint32_t val);
   Signal& operator=(int8_t val);
   Signal& operator=(int16_t val);
   Signal& operator=(int32_t val);

  private:

    uint8_t signalNumber;
    
    uint8_t get();
    void set(uint8_t val);
    void offset(uint8_t& byteNum, uint8_t& bitNum);
    
}; 
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<uint16_t> SignalsAddressList;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// класс менеджера сигналов
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class SignalsManager
{
  public:
    SignalsManager();

    Signal& operator[](uint8_t signalNumber);

    void reset();
    void begin();
    void update();

   void pause();
   void resume();

    void addRecord(uint16_t memoryAddress); // добавляем запись в список

  private:

    Signal thisSignal;
    SignalsAddressList addresses;
    uint32_t updateTimer;

    bool bPaused;
  
}; 
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
extern SignalsManager Signals;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_SIGNALS_ENABLED
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
