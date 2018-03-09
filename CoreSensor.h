#ifndef _CORE_SENSOR_H
#define _CORE_SENSOR_H

#include <Arduino.h>
#include "CoreConfig.h"
#include "CoreArray.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SI7021_ENABLED
  #include "CoreHTU21D.h"
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
// типы данных и структуры
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum // тип датчика
{
  Unknown = 0, // неизвестный датчик
  DS18B20 = 1, // температурный датчик DS18B20
  DHT = 2, // датчик влажности DHT
  Si7021 = 3,  // датчик влажности Si7021
  BH1750 = 4, // датчик освещённости BH1750
  DS3231 = 5, // датчик часов реального времени
  DS3231Temperature = 6, // температура с датчика часов реального времени
  DigitalPortState = 7, // состояние цифрового порта
  AnalogPortState = 8, // значение с аналогового порта
  UserDataSensor = 9, // датчик "пользовательские данные"
  MAX6675 = 10, // термопара MAX6675
  BMP180 = 11, // датчик BMP085 или BMP180
  MAX44009 = 12, // датчик освещённости MAX44009
  HCSR04 = 13, // УЗ-датчик расстояния HC-SR04
  //TODO: Тут добавлять другие типы!!!
  
} CoreSensorType;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  UnknownType = 0,
  Temperature = 1, // температура
  Luminosity = 2, // освещённость
  Humidity = 3, // температура и влажность
  DateTime = 4, // дата и время
  DigitalPort = 5, // состояние порта
  AnalogPort = 6, // показания аналогового порта
  UserData = 7, // пользовательские данные
  Pressure = 8, // давление
  Altitude = 9, // высота над уровнем моря
  Barometric = 10, // данные с барометрических датчиков
  Distance = 11, // расстояние в мм
  
  //TODO: Тут добавлять другие типы!!!
  
  
} CoreDataType;
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push, 1)
//--------------------------------------------------------------------------------------------------------------------------------------
struct DistanceData
{
  uint32_t Value;
  uint8_t Fract;

  DistanceData()
  {
    Value = Fract = 0;
  }

  DistanceData(uint32_t v, uint8_t f)
  {
    Value = v;
    Fract = f;
  }

  bool operator==(const DistanceData& rhs);
  bool operator!=(const DistanceData& rhs);

  bool operator<(const DistanceData& rhs);
  bool operator<=(const DistanceData& rhs);

  bool operator>(const DistanceData& rhs);
  bool operator>=(const DistanceData& rhs);
  
  operator String();

private:

  uint32_t raw() const;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
struct TemperatureData
{
  int16_t Value;
  uint8_t Fract;

  TemperatureData()
  {
    Value = Fract = 0;
  }

  TemperatureData(int16_t v, uint16_t f)
  {
    Value = v;
    Fract = f;
  }

  static TemperatureData ConvertToFahrenheit(const TemperatureData& from);

  bool operator==(const TemperatureData& rhs);
  bool operator!=(const TemperatureData& rhs);

  bool operator<(const TemperatureData& rhs);
  bool operator<=(const TemperatureData& rhs);

  bool operator>(const TemperatureData& rhs);
  bool operator>=(const TemperatureData& rhs);
  
  operator String();

private:

  int32_t raw() const;
  
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
// для высоты над уровнем моря вполне можно использовать структуру температуры, т.к. на 32767 метров навряд ли кто с датчиком улетит. 
// Ну и погрузиться на -32767 метров на Земле - тоже проблематично, Марианская впадина негодуэ :). Поэтому делаем typedef.
typedef struct TemperatureData AltitudeData;
//--------------------------------------------------------------------------------------------------------------------------------------
struct PressureData // структура, описывающая давление (в паскалях)
{
  uint8_t isInPA; // тип показаний - в Паскалях или мм. рт. ст.
  int32_t Value;
  uint8_t Fract;

  PressureData()
  {
    isInPA = true;
    Value = Fract = 0;
  }

  PressureData(uint8_t isPA, int32_t v, uint8_t f)
  {
    isInPA = isPA;
    Value = v;
    Fract = f;
  }

  bool operator==(const PressureData& rhs);
  bool operator!=(const PressureData& rhs);

  bool operator<(const PressureData& rhs);
  bool operator<=(const PressureData& rhs);

  bool operator>(const PressureData& rhs);
  bool operator>=(const PressureData& rhs);

  static PressureData ConvertToMmHg(const PressureData& from); // конвертирует паскали в мм. рт. ст.
  
  operator String(); 

private:

  int32_t raw() const;   
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
struct BarometricData // структура данных для барометрических датчиков, типа BMP180
{
   TemperatureData Temperature;
   PressureData Pressure;
   AltitudeData Altitude;
};
//--------------------------------------------------------------------------------------------------------------------------------------
struct LuminosityData
{
  int32_t Value;

  LuminosityData()
  {
    Value = 0;
  }

  LuminosityData(int32_t v)
  {
    Value = v;
  }

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

  AnalogPortData()
  {
    Pin = Value = 0;
  }

  AnalogPortData(uint8_t p, uint16_t v)
  {
    Pin = p;
    Value = v;
  }

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

  DigitalPortData()
  {
    Pin = Value = 0;
  }

  DigitalPortData(uint8_t p, uint8_t v)
  {
    Pin = p;
    Value = v;
  }

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
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
// класс абстрактного датчика
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
  static String getUnit(CoreDataType type, uint8_t anyFlag=0); // возвращает единицы измерения для типов данных

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
enum { MAX44009_ADDRESS1 = 0x4A, MAX44009_ADDRESS2 = 0x4B };
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_MAX44009_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorMAX44009 : public CoreSensor
{
  public:

  CoreSensorMAX44009();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  virtual uint16_t startMeasure()
  {
    return 810; // у нас минимум 800 мс на конвертацию
  }
  
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  private:
    uint8_t deviceAddress, i2cIndex;

    float readLuminosity();
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_MAX44009_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_BMP180_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#define BMP085_I2CADDR 0x77

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4 
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD       0x2E
#define BMP085_READPRESSURECMD  0x34
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorBMP180 : public CoreSensor
{
  public:

  CoreSensorBMP180();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
 
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  private:

    bool doBegin(uint8_t mode = BMP085_ULTRAHIGHRES);  // by default go highres
    float readTemperature(void);
    int32_t readPressure(void);
    int32_t readSealevelPressure(float altitude_meters = 0);
    float readAltitude(float sealevelPressure = 101325); // std atmosphere
    uint16_t readRawTemperature(void);
    uint32_t readRawPressure(void);
    
    int32_t computeB5(int32_t UT);
    uint8_t read8(uint8_t addr);
    uint16_t read16(uint8_t addr);
    void write8(uint8_t addr, uint8_t data);
  
    uint8_t oversampling;
  
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;

    uint8_t i2cIndex;
    bool isInPascalsMeasure; // флаг - меряем в паскалях или в мм.рт.ст
    int32_t homeplacePressure; // нормальное давление для нашего местоположения (для расчёта высоты над уровнем моря)
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_BMP180_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_HCSR04_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorHCSR04 : public CoreSensor
{
  public:

  CoreSensorHCSR04();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  private:
    
   uint8_t triggerPin, echoPin;

  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_HCSR04_ENABLED
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
#ifdef CORE_MAX6675_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSensorMAX6675 : public CoreSensor
{
  public:

  CoreSensorMAX6675();

  virtual void begin(uint8_t* configData); // инициализирует датчик
  virtual bool read(uint8_t* buffer); // читает с датчика, возвращает false в случае, если с датчика не удалось прочитать
  
  virtual uint8_t getDataSize(); // возвращает размер данных буфера показаний с датчика

  private:

    uint8_t cs;
    bool slow;

    double readCelsius();

  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_MAX6675_ENABLED
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
