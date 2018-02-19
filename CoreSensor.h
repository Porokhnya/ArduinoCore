#ifndef _CORE_SENSOR_H
#define _CORE_SENSOR_H

#include <Arduino.h>
#include "CoreConfig.h"
#include "CoreArray.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SI7021_ENABLED
  #include "HTU21D.h"
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
// класс абстрактного датчика
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum // тип датчика
{
  Unknown, // неизвестный датчик
  DS18B20, // температурный датчик DS18B20
  DHT, // датчик влажности DHT
  Si7021,  // датчик влажности Si7021
  BH1750, // датчик освещённости BH1750
  DS3231, // датчик часов реального времени
  DS3231Temperature, // температура с датчика часов реального времени
  DigitalPortState, // состояние цифрового порта
  AnalogPortState, // значение с аналогового порта
  UserDataSensor, // датчик "пользовательские данные"
  //TODO: Тут добавлять другие типы!!!
  
} CoreSensorType;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  UnknownType,
  Temperature, // температура
  Luminosity, // освещённость
  Humidity, // температура и влажность
  DateTime, // дата и время
  DigitalPort, // состояние порта
  AnalogPort, // показания аналогового порта
  UserData, // пользовательские данные
  //TODO: Тут добавлять другие типы!!!
  
  
} CoreDataType;
//--------------------------------------------------------------------------------------------------------------------------------------
struct TemperatureData
{
  int8_t Value;
  uint8_t Fract;

  static TemperatureData ConvertToFahrenheit(const TemperatureData& from);

  bool operator==(const TemperatureData& rhs);
  bool operator!=(const TemperatureData& rhs);

  bool operator<(const TemperatureData& rhs);
  bool operator<=(const TemperatureData& rhs);

  bool operator>(const TemperatureData& rhs);
  bool operator>=(const TemperatureData& rhs);
  
  operator String();

private:

  int raw() const;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  TemperatureData Temperature;
  TemperatureData Humidity;

  operator String()
  {
    String result = Temperature;
    result += CORE_HUMIDITY_DELIMITER;
    result += Humidity;
    return result;
  }
  
} HumidityData;
//--------------------------------------------------------------------------------------------------------------------------------------
struct LuminosityData
{
  uint16_t Value;

  bool operator==(const LuminosityData& rhs) {return Value == rhs.Value; }
  bool operator!=(const LuminosityData& rhs) {return Value != rhs.Value; }

  bool operator<(const LuminosityData& rhs) {return Value < rhs.Value; }
  bool operator<=(const LuminosityData& rhs) {return Value <= rhs.Value; }

  bool operator>(const LuminosityData& rhs) {return Value > rhs.Value; }
  bool operator>=(const LuminosityData& rhs) {return Value >= rhs.Value; }

  operator String()
  {
    return String(Value);
  }

  
};
//--------------------------------------------------------------------------------------------------------------------------------------
struct AnalogPortData
{
  uint8_t Pin;
  uint16_t Value;

  bool operator==(const AnalogPortData& rhs) {return Value == rhs.Value; }
  bool operator!=(const AnalogPortData& rhs) {return Value != rhs.Value; }

  bool operator<(const AnalogPortData& rhs) {return Value < rhs.Value; }
  bool operator<=(const AnalogPortData& rhs) {return Value <= rhs.Value; }

  bool operator>(const AnalogPortData& rhs) {return Value > rhs.Value; }
  bool operator>=(const AnalogPortData& rhs) {return Value >= rhs.Value; }

  operator String()
  {
    return String(Value);
  }
  
    
};
//--------------------------------------------------------------------------------------------------------------------------------------
struct DigitalPortData
{
  uint8_t Pin;
  uint8_t Value;

  bool operator==(const DigitalPortData& rhs) {return Value == rhs.Value; }
  bool operator!=(const DigitalPortData& rhs) {return Value != rhs.Value; }

  bool operator<(const DigitalPortData& rhs) {return Value < rhs.Value; }
  bool operator<=(const DigitalPortData& rhs) {return Value <= rhs.Value; }

  bool operator>(const DigitalPortData& rhs) {return Value > rhs.Value; }
  bool operator>=(const DigitalPortData& rhs) {return Value >= rhs.Value; }

  operator String()
  {
    return String(Value);
  }
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
struct DateTimeData
{
  uint8_t day;
  uint8_t month;
  uint16_t year;

  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  uint8_t dayOfWeek;

  DateTimeData();

  static bool isLeapYear(uint16_t year);
  uint32_t unixtime(void) const;
  static DateTimeData fromUnixtime(uint32_t timeInput);
  DateTimeData addDays(long days);

  operator String(); // формируем дату/время в виде строки
  
  bool operator <(const DateTimeData& rhs);
  bool operator <=(const DateTimeData& rhs);

  bool operator >(const DateTimeData& rhs);
  bool operator >=(const DateTimeData& rhs);

  bool operator ==(const DateTimeData& rhs);
  bool operator !=(const DateTimeData& rhs);

  private:
  
    uint16_t date2days(uint16_t _year, uint8_t _month, uint8_t _day) const;
    long time2long(uint16_t days, uint8_t hours, uint8_t minutes, uint8_t seconds) const;

};
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensor
{
public:
  CoreSensor(CoreSensorType type);
  virtual ~CoreSensor() {}

  virtual void begin(uint8_t* configData) = 0; // инициализирует датчик
  virtual void update();  // обновляет внутреннее состояние, вызывается только, если startMeasure возвращает кол-во миллисекунд, отличное от нуля
  virtual uint16_t startMeasure(); // запускает конвертацию с датчика, датчик возвращает кол-во миллисекунд, после истечения которых с него можно читать
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  
  bool isUserDataSensor() {return type == UserDataSensor; } // тестирует - не датчик ли это пользовательского типа с какими-то показаниями? С таких датчиков мы только обновляем хранилище, данные обновляет внешняя среда, а не ядро

  CoreSensorType getType() { return type; } // возвращает тип железки

  static CoreDataType getDataType(CoreSensorType type); // возвращает тип данных с датчика
  static String getUnit(CoreDataType type); // возвращает единицы измерения для типов данных

  String getName() {return mnemonicName;} // мнемоническое имя датчика
  void setName(const String& name) { mnemonicName = name; } 

protected:

  CoreSensorType type;
  String mnemonicName; // мнемоническое имя

};
//--------------------------------------------------------------------------------------------------------------------------------------
// тут идёт имплементация датчиков конкретных типов
//--------------------------------------------------------------------------------------------------------------------------------------
enum { BH1750PowerOff=0x00, BH1750PowerOn=0x01, BH1750Reset = 0x07 };
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum {  BH1750Address1 = 0x23, BH1750Address2 = 0x5C } BH1750Address; // адрес датчика освещенности на шине I2C
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_BH1750_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorBH1750 : public CoreSensor
{
  public:

  CoreSensorBH1750();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  private:
    void writeByte(uint8_t toWrite);
    uint8_t deviceAddress, i2cIndex;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_BH1750_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_DIGITALPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorDigitalPort : public CoreSensor
{
  public:

  CoreSensorDigitalPort();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  private:
    
   uint8_t pin;

  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_DIGITALPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ANALOGPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorAnalogPort : public CoreSensor
{
  public:

  CoreSensorAnalogPort();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  private:
    
   uint8_t pin;

  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_ANALOGPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_USERDATA_SENSOR_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreUserDataSensor : public CoreSensor
{
  public:

  CoreUserDataSensor();
  ~CoreUserDataSensor();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  void setData(uint8_t* dt,uint8_t sz);

  // установка/получение пользовательского типа данных
  void setUserDataType(CoreDataType dt) {userDataType = dt;}
  CoreDataType getUserDataType() {return userDataType;}
  

  private:
    
   uint8_t dataSize;
   uint8_t* data;

   CoreDataType userDataType;

  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_USERDATA_SENSOR_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SI7021_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorSi7021 : public CoreSensor
{
  public:

  CoreSensorSi7021();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  private:
    
    HTU21D sensor;
    uint8_t i2cIndex;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_SI7021_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum { DHT_11, DHT_2x } DHTType; // тип датчика, который опрашиваем, поскольку у DHT11 немного другой формат данных
enum { DHT2x_WAKEUP=1, DHT11_WAKEUP=18 }; // таймауты инициализации для разных типов датчиков
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_DHT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorDHT : public CoreSensor
{
  public:

  CoreSensorDHT();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  private:

    uint8_t pin;
    DHTType sensorType;

  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_DHT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_DS3231_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
enum { DS3231Address = 0x68 }; // адрес датчика
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorDS3231 : public CoreSensor
{
  public:

  CoreSensorDS3231(bool tempOnly);

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика
  void setTime(uint8_t second, uint8_t minute, uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month, uint16_t year);

  private:

    uint8_t dec2bcd(uint8_t val);
    uint8_t bcd2dec(uint8_t val);
    
    bool isTempOnly;
    uint8_t i2cIndex;
  
}; 
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_DS3231_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_DS18B20_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#include <OneWire.h> // подключаем библиотеку OneWire
//--------------------------------------------------------------------------------------------------------------------------------------
// менеджер линии, к которому обращаются непосредственно классы датчиков DS18B20 -
// у нас может быть много датчиков на одной линии, поэтому запросы на конвертацию от класса датчика
// не должны дублироваться, менеджер должен уметь читать данные конкретного датчика на шине по его индексу.
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  bool inited : 1;
  bool inConversion : 1;
  bool isFirstTimeConversion : 1;
  uint8_t pad : 5;
  
} CoreDS18B20LineManagerFlags;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  temp9bit = 0x1F,
  temp10bit = 0x3F,
  temp11bit = 0x5F,
  temp12bit = 0x7F
  
} CoreDS18B20Resolution;

//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorDS18B20;
typedef Vector<uint8_t*> CoreDS18B20AddressList;
class CoreDS18B20LineManager 
{
public:
  CoreDS18B20LineManager(uint8_t pin);
  ~CoreDS18B20LineManager();

  uint16_t startConversion();
  bool readTemperature(int8_t& tempValue, uint8_t& tempFract, CoreSensorDS18B20* sensor);

  uint8_t catchSensor(CoreSensorDS18B20* sensor); // захватываем ещё один датчик во владение

  uint8_t getPin() {return pin;}


private:
  void begin(CoreDS18B20Resolution res = temp12bit);

  bool findAddress(OneWire& ow, uint8_t* output,CoreSensorDS18B20* sensor);
  bool isGoodAddress(const uint8_t* address);
  void clearAddresses();
  bool addressExists(const uint8_t* address, uint8_t*& existing);

  uint8_t* addAddress(const uint8_t* address, size_t sensorIndex);
  void serializeAddress(const uint8_t* address,CoreSensorDS18B20* sensor);
  
  CoreDS18B20LineManagerFlags flags;
  uint8_t pin, sensorsCount, sensorCounter;
  CoreDS18B20AddressList addresses;  
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreDS18B20LineManager*> CoreDS18B20LineManagersList;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreDS18B20DispatcherClass
{
  public:
    CoreDS18B20DispatcherClass();

    CoreDS18B20LineManager* add(uint8_t pin);
    void clear();

  private:
    CoreDS18B20LineManagersList list;
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreDS18B20DispatcherClass CoreDS18B20Dispatcher;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorDS18B20 : public CoreSensor
{
  public:

  CoreSensorDS18B20();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  virtual uint16_t startMeasure(); // запускает конвертацию с датчика, датчик возвращает кол-во миллисекунд, после истечения которых с него можно читать  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  uint8_t getIndex() {return myIndex;}
  void setAddress(uint8_t* address) {myAddress = address;}
  bool getAddress(uint8_t* output)
  {
    if(!myAddress)
      return false;

      memcpy(output,myAddress,8);
      
      return true;
  }

  bool isBroken()
  {
    bool result = false;
    badReadingAttempts++;
    result = badReadingAttempts > 5;
    
    if(result) // если мы уже поломаты - нам сбросят адрес, и можно будет пытаться найти новый датчик на шине
      badReadingAttempts = 0;
      
    return result; // через N попыток чтения мы сбросим этот адрес в 0
  }

  private:

  CoreDS18B20LineManager* myManager; // менеджер, который обслуживает нашу линию
  uint8_t myIndex; // индекс датчика на линии
  uint8_t* myAddress;
  uint8_t badReadingAttempts;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_DS18B20_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
// фабрика датчиков
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorsFactory
{
  public:

  static CoreSensor* createSensor(CoreSensorType type);

  private:
    CoreSensorsFactory();
 
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
