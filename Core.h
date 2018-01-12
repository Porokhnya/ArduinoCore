#ifndef _CORE_H
#define _CORE_H
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "CoreCommandBuffer.h"
#include "CoreArray.h"
#include "CoreSensor.h"
#include "CoreTransport.h"
#include "CorePinStateAction.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_VERSION F("Core v.1.0")
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef _CORE_DEBUG
  #define DBG(s) Serial << (s)
  #define DBGLN(s) Serial << (s) << ENDL
#else
  #define DBG(s) (void) 0
  #define DBGLN(s) (void) 0
#endif
#define ENDL '\n'
//--------------------------------------------------------------------------------------------------------------------------------------
template <typename T> inline Stream& operator << (Stream &s, T n) { s.print(n); return s; }
//--------------------------------------------------------------------------------------------------------------------------------------
typedef void (*CoreUnhandledCommandsHandler)(const String& str, Stream* outStream); 
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  DummyFirstRecord = 0x1, // первая запись, для проверки вхождения в диапазон
  
  SensorRecord = 0x2, // данные о датчике
  FractDelimiterRecord = 0x3, // данные о разделителе целой и дробной частей
  TemperatureUnitRecord = 0x04, // тип измерений температуры (цельсии или фаренгейты)
  ESPSettingsRecord = 0x05, // данные о настройках ESP
  SensorsUpdateIntervalRecord = 0x06, // интервал опроса датчиков
  RS485SettingsRecord = 0x07, // данные о настройках RS-485
  RS485IncomingPacketRecord = 0x08, // данные об известном пакете RS-485

  DummyLastRecord = 0x09 // последняя запись, для проверки вхождения в диапазон
  
} CoreConfigRecordType;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  UnitCelsius,
  UnitFahrenheit
  
} UnitTemperature;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreConfigIterator
{
  public:
    bool first(const void* address, uint16_t dataSize); // ищет первую запись в конфиге, если не находит - возвращает false
    bool next(); // проходит по всем записям в конфиге

    CoreConfigIterator();

  protected:

    uint16_t dataSize; // размер данных, до которого можно итерировать
    const void* address; // текущий адрес чтения

    virtual byte doRead(const void* startAddress, uint16_t addressOffset) = 0; // потомки в этом методе читают и возвращают байт, передаётся начальный адрес и смещение, по которому надо прочитать
    

  private:

    uint16_t readed;
    byte read();
    bool readRecord();
    void applySensorRecord(const String& sensorName, CoreSensorType type,byte* record);
   
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreEEPROMConfigIterator : public CoreConfigIterator
{
  public:
    CoreEEPROMConfigIterator();

  protected:
    virtual byte doRead(const void* startAddress, uint16_t addressOffset);
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreSensor*> CoreSensorsList;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensors
{
  public:
    CoreSensors() {};

    void add(CoreSensor* sensor) {list.push_back(sensor);};
    size_t size() {return list.size();};
    CoreSensor* get(size_t idx) {return list[idx];};
    
    CoreSensorsList getByType(CoreSensorType type) // возвращает список датчиков по типу их железок
    {
      CoreSensorsList result;

      for(size_t i=0;i<list.size();i++)
      {
        if(list[i]->getType() == type)
          result.push_back(list[i]);
      }

      return result;
    }

    CoreSensor* get(const String& name) // возвращает датчик по его мнемоническому имени
    {
      for(size_t i=0;i<list.size();i++)
      {
         if(list[i]->getName() == name)
          return list[i];
      }
      return NULL;
    }
    
    void clear();

  private:

    CoreSensorsList list;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  unsigned long startTimer;
  uint16_t signalDelay;
  CoreSensor* sensor;
  int storeIndex;
  
} CoreSensorSignalStruct;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreSensorSignalStruct> CoreSensorSignals;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreClass
{
	public:
		CoreClass();

    char FractDelimiter; // разделитель целой и дробной частей
    byte TemperatureUnit; // вид измерения температуры (в цельсиях или фаренгейтах)
    unsigned long SensorsUpdateInterval; // интервал обновления показаний с датчиков


    const char* byteToHexString(byte i); // конвертирует байт в его строковое представление в шестнадцатеричном виде

    void pushToStorage(CoreSensor* sensor); // обновляет показания с датчика в хранилище


    int getPinMode(int pin); // возвращает режим работы пина - вход или выход
    int getPinState(int pin); // возвращает состояние пина, вне зависимости от его режима работы
		
		void setup(CoreUnhandledCommandsHandler func = NULL);
   
    void printVersion(Stream& s); // печатает версию ядра в поток
    void printSupportedSensors(Stream& s); // печатает список поддерживаемых датчиков в поток
    
    int getFreeMemory(); // возвращает кол-во свободной оперативной памяти

   uint8_t memRead(unsigned int address); // читает байт из EEPROM
   void memWrite(unsigned int address, uint8_t val); // пишет байт в EEPROM

   // обрабатывает команду и пишет результат её обработки в вызвавший поток. Команды приходят во внутреннем формате, и в поток ответ пишется тоже во внутреннем формате
   void processCommand(const String& command,Stream* outStream);

   // обрабатывает входящие по Serial команды
   void handleCommands();

   // обновляет состояние ядра (перечитывает показания с датчиков и т.п.)
   void update();

   void begin(); // начинаем работу
   
   void clear(); // очищает все датчики из списка, очищает хранилище, сбрасывает настройки - после вызова этого метода датчиков больше нет в системе

   CoreSensors* Sensors() {return &list;} // возвращает список датчиков

   // грузит конфиг из EEPROM
   bool loadConfig(); 

   // сохраняет настройки конфига из массива в памяти в EEPROM
   void saveConfig(const byte* address, uint16_t sz, bool isInFlashSource);

   private:

   void memInit();
   void initSensors();


   CoreUnhandledCommandsHandler pUnhandled;
   unsigned long lastMillis;

   CoreSensors list;
   CoreSensorSignals signals;
   void signal(uint16_t signalDelay,CoreSensor* sensor,int storeIndex);
   bool waitingSignal(CoreSensor* sensor);
   void updateSignals();
   void readFromSensor(CoreSensor* sensor,int storeIndex);

   // обработчики команд GET и SET

   // Дата/время
   bool setDATETIME(const char* param);
   bool getDATETIME(const char* commandPassed, Stream* pStream);

  // Список поддерживаемых датчиков
   bool getSENSORS(const char* commandPassed, Stream* pStream);

   // Список поддерживаемых транспортов
   bool getTRANSPORT(const char* commandPassed, Stream* pStream);

   // Кол-во свободной памяти
   bool getFREERAM(const char* commandPassed, Stream* pStream);

   // печатает результаты обработки известной команды SET в поток
   bool printBackSETResult(bool isOk, const char* command, Stream* pStream);
   
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreClass Core;
//--------------------------------------------------------------------------------------------------------------------------------------
struct CoreStoredData
{
  byte* data;
  byte dataSize;
  //CoreSensorType sensorType;
  CoreSensor* sensor;

  operator LuminosityData() const; // возвращает данные как освещённость
  operator TemperatureData() const; // возвращает данные, как температуру
  operator HumidityData() const; // возвращает данные, как пару температуры и влажности
  operator DateTimeData() const; // возвращает данные, как дату/время
  operator DigitalPortData() const; // возвращает данные, как состояние цифрового порта
  operator AnalogPortData() const; // возвращает данные, как состояние аналогового порта
  
  bool hasData() const {return data != NULL;}
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreStoredData> CoreDataList;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreDataStoreClass
{
  public:
    CoreDataStoreClass();

    size_t save(CoreSensor* sensor, byte* data, byte dataSize);

    CoreStoredData get(size_t idx) {return list[idx];};
    CoreStoredData get(const String& name);
    CoreStoredData get(CoreSensorType type, uint8_t pin);
    size_t size(){return list.size();}

    CoreDataList getByType(CoreDataType type); // возвращает список показаний по типу
    CoreDataList getBySensor(CoreSensorType type); // возвращает показания по типу железки

    void clear();

  protected:
    friend class CoreClass;
    CoreDataList list;
}; 
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreDataFormatProvider
{
 public:
  CoreDataFormatProvider() {};
   virtual String format(const CoreStoredData& data, size_t sensorIndex, bool showUnits) = 0;
};
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreTextFormatProvider : public CoreDataFormatProvider // выводим показания как простой текст
{
  public:
    CoreTextFormatProvider();
    virtual String format(const CoreStoredData& data, size_t sensorIndex, bool showUnits);
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreDataStoreClass CoreDataStore; // хранилище данных с датчиков
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_HEADER1 0x50
#define CORE_HEADER2 0xED
#define CORE_HEADER3 0x9F
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
