#ifndef _CORE_H
#define _CORE_H
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "CoreConfig.h"
#include "CoreCommandBuffer.h"
#include "CoreArray.h"
#include "CoreSensor.h"
#include "CoreTransport.h"
#include "CorePinScenario.h"
#include "LoRa.h"
#include "Signals.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SD_SUPPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
  #ifdef CORE_SD_USE_SDFAT
    #include <SdFat.h>
    extern SdFat SD;
  #else
    #include <SD.h>
  #endif
//--------------------------------------------------------------------------------------------------------------------------------------
// структура настроек SD
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint8_t CSPin;
  
} SDSettingsStruct;
//--------------------------------------------------------------------------------------------------------------------------------------
extern SDSettingsStruct SDSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
// класс-хелпер для работы с файлами на SD
//--------------------------------------------------------------------------------------------------------------------------------------
class FileUtils
{
  public:
  
  static int countFiles(const String& dirName, bool recursive);
  static void printFilesNames(const String& dirName, bool recursive, Stream* outStream);
  static void printFile(const String& fileName, Stream* outStream);
  
  static String getFileName(
    #ifdef CORE_SD_USE_SDFAT
    SdFile
    #else
    File
    #endif
    &f);
  static bool readLine(
    #ifdef CORE_SD_USE_SDFAT
    SdFile 
    #else
    File
    #endif
    &f, String& result);
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_SD_SUPPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
  void ON_CORE_BEGIN();
}
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
  DummyFirstRecord = 1, // первая запись, для проверки вхождения в диапазон
  
  SensorRecord = 2, // данные о датчике
  FractDelimiterRecord = 3, // данные о разделителе целой и дробной частей
  TemperatureUnitRecord = 4, // тип измерений температуры (цельсии или фаренгейты)
  ESPSettingsRecord = 5, // данные о настройках ESP
  SensorsUpdateIntervalRecord = 6, // интервал опроса датчиков
  RS485SettingsRecord = 7, // данные о настройках RS-485
//  RS485IncomingPacketRecord = 8, // данные об известном пакете RS-485
  LoRaSettingsRecord = 8, // данные по настройкам LoRa
  DeviceIDRecord = 9, // данные по ID устройства
  ClusterIDRecord = 10, // данные о ID кластера, к которому принадлежит группа устройств
  SignalRecord = 11, // запись о сигнале
  WatchdogRecord = 12, // запись настроек ватчдога

  DummyLastRecord = 13 // последняя запись, для проверки вхождения в диапазон
  
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
    bool first(uint16_t address, uint16_t dataSize, Stream* outStream=NULL, bool asHexString=true); // ищет первую запись в конфиге, если не находит - возвращает false
    bool next(); // проходит по всем записям в конфиге

    CoreConfigIterator();

  protected:

    uint16_t dataSize; // размер данных, до которого можно итерировать
    uint16_t address; // текущий адрес чтения

    virtual uint8_t doRead(uint16_t startAddress, uint16_t addressOffset) = 0; // потомки в этом методе читают и возвращают байт, передаётся начальный адрес и смещение, по которому надо прочитать
    

  private:

    uint16_t readed;
    uint8_t read();
    bool readRecord();
    void applySensorRecord(const String& sensorName, CoreSensorType type,uint8_t* record);
    
    Stream* outStream;
    bool asHexString;

    bool writeOut(uint8_t b);
   
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreEEPROMConfigIterator : public CoreConfigIterator
{
  public:
    CoreEEPROMConfigIterator();

  protected:
    virtual uint8_t doRead(uint16_t startAddress, uint16_t addressOffset);
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
  uint16_t timerDelay;
  CoreSensor* sensor;
  int storeIndex;
  
} CoreSensorTimerStruct;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreSensorTimerStruct> CoreSensorTimers;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreClass
{
	public:
		CoreClass();


    void yieldCritical(); // функция, вне очереди обновляющая критические части ядра (например, вычитку из порта ESP, чтобы данные не протухали)

    char FractDelimiter; // разделитель целой и дробной частей
    uint8_t TemperatureUnit; // вид измерения температуры (в цельсиях или фаренгейтах)
    unsigned long SensorsUpdateInterval; // интервал обновления показаний с датчиков
    uint8_t DeviceID; // уникальный адрес устройства
    uint8_t ClusterID; // уникальный адрес кластера

    static uint8_t crc8(const uint8_t *addr, uint8_t len);

    // устанавливает дату/время для всех DS3231
    void setCurrentDateTime(uint8_t dayOfMonth, uint8_t month, uint16_t year, uint8_t hour, uint8_t minute, uint8_t second);

    const char* byteToHexString(uint8_t i); // конвертирует байт в его строковое представление в шестнадцатеричном виде
    static uint8_t hexStringToByte(const char* buff); // конвертирует строковое представление байта в шестнадцатеричном виде в байт

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
   
   void reset(); // очищает все датчики из списка, очищает хранилище, сбрасывает настройки - после вызова этого метода датчиков больше нет в системе

   CoreSensors* Sensors() {return &list;} // возвращает список датчиков

   // грузит конфиг из EEPROM
   bool loadConfig(); 

   // сохраняет настройки конфига из массива в памяти в EEPROM
   void saveConfig(const uint8_t* address, uint16_t sz, bool isInFlashSource);

   private:

   static uint8_t makeNum(char ch);

   void memInit();
   void initSensors();

    bool configLoaded;

   CoreUnhandledCommandsHandler pUnhandled;
   unsigned long lastMillis;

   CoreSensors list;
   CoreSensorTimers sensorTimers;
   void addTimer(uint16_t timerDelay,CoreSensor* sensor,uint16_t storeIndex);
   bool isOnTimer(CoreSensor* sensor);
   void updateTimers();
   void readFromSensor(CoreSensor* sensor,uint16_t storeIndex);

   // обработчики команд GET и SET

   // Дата/время
   bool setDATETIME(const char* param);
   bool getDATETIME(const char* commandPassed, Stream* pStream);

  // Список поддерживаемых датчиков
   bool getSENSORS(const char* commandPassed, Stream* pStream);

   // Список поддерживаемого функционала
   bool getFEATURES(const char* commandPassed, Stream* pStream);

   // Кол-во свободной памяти
   bool getFREERAM(const char* commandPassed, Stream* pStream);

   // Вид процессора
   bool getCPU(const char* commandPassed, Stream* pStream);

   // Вернуть в поток конфиг, закодированный в HEX
   bool getCONFIG(const char* commandPassed, Stream* pStream);

   // вернуть в поток состояние сигналов, закодированное в HEX
   bool getSIGNALS(const char* commandPassed, Stream* pStream);

   // текущий адрес записи конфига
   uint16_t configSaveAddress; 
   // Начать запись конфига
   bool setCONFIGSTART();
   // Записать часть конфига - куски по 50 байт
   bool setCONFIGPART(const char* param);

   // команда установки уровня на пине
   bool setPIN(CommandParser& parser, Stream* outStream);

   // команды для SD
   #ifdef CORE_SD_SUPPORT_ENABLED
    bool getLS(const char* commandPassed, const CommandParser& parser, Stream* pStream);
    bool getFILE(const char* commandPassed, const CommandParser& parser, Stream* pStream);
   #endif

   // перезапустить ядро
   bool wantRestart;
   bool setRESTART();

   // вернуть в поток показания всех датчиков хранилища, закодированные в HEX
   bool getSTORAGE(const char* commandPassed, Stream* pStream);

   // печатает результаты обработки известной команды SET в поток
   bool printBackSETResult(bool isOk, const char* command, Stream* pStream);
   
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreClass Core;
//--------------------------------------------------------------------------------------------------------------------------------------
struct CoreStoredData
{
  uint8_t* data; // сырые данные
  uint8_t dataSize; // размер сырых данных
  CoreSensor* sensor; // датчик, с которого получены данные

  operator LuminosityData() const; // возвращает данные как освещённость
  operator TemperatureData() const; // возвращает данные, как температуру
  operator HumidityData() const; // возвращает данные, как пару температуры и влажности
  operator DateTimeData() const; // возвращает данные, как дату/время
  operator DigitalPortData() const; // возвращает данные, как состояние цифрового порта
  operator AnalogPortData() const; // возвращает данные, как состояние аналогового порта
  
  bool hasData() const {return data != NULL;}

  CoreStoredData()
  {
    data = NULL;
    dataSize = 0;
    sensor = NULL;
  }
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreStoredData> CoreDataList;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreDataStoreClass
{
  public:
    CoreDataStoreClass();

    size_t save(CoreSensor* sensor, uint8_t* data, uint8_t dataSize);

    CoreStoredData get(size_t idx) {return list[idx];}; // возвращает данные по индексу
    CoreStoredData get(const String& sensorMnemonicName); // возвращает данные для датчика по его мнемоническому имени
    CoreStoredData get(CoreSensorType type, uint8_t pin); // возвращает данные для датчика аналогового или цифрового порта по номеру пина
    size_t size(){return list.size();}

    CoreDataList getByType(CoreDataType type); // возвращает список показаний по типу показаний (температура, влажность и т.п.)
    CoreDataList getBySensor(CoreSensorType type); // возвращает писок показаний по типу железки датчика (DS18B20, DHT и т.п.)

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
struct WatchdogSettingsClass
{
  bool WatchdogEnabled;
  uint8_t WatchdogPin;
  uint16_t WatchdogInterval;
  uint16_t WatchdogPulseDuration;
  void reset();
  void update();
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern WatchdogSettingsClass CoreWatchdog;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef void (*CoreDelayedEventHandler)(void* param);
//--------------------------------------------------------------------------------------------------------------------------------------
struct CoreDelayedEventPinChangeArg
{
  uint8_t pin;
  uint8_t level;
  
  CoreDelayedEventPinChangeArg(uint8_t p, uint8_t l)
  {
    pin = p;
    level = l;
  }
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  unsigned long timer;
  unsigned long duration;
  void* param;
  CoreDelayedEventHandler handler;
  
} CoreDelayedEventData;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreDelayedEventData> CoreDelayedEventsList;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreDelayedEventClass
{
  public:
    CoreDelayedEventClass();

    void update();
    void raise(unsigned long raiseDelay,CoreDelayedEventHandler handler, void* param);

    static void CoreDelayedEventPinChange(void* param);

  private:

    CoreDelayedEventsList signals;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreDelayedEventClass CoreDelayedEvent;
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_HEADER1 0xE1
#define CORE_HEADER2 0xED
#define CORE_HEADER3 0x9F
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
