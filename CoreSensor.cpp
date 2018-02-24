#include "CoreSensor.h"
#include "Core.h"
//--------------------------------------------------------------------------------------------------------------------------------------
bool __isSPIInited = false;
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined(CORE_DS3231_ENABLED) || defined(CORE_BH1750_ENABLED) || defined(CORE_SI7021_ENABLED) || defined(CORE_BMP180_ENABLED) || defined(CORE_MAX44009_ENABLED)
#include <Wire.h>
//--------------------------------------------------------------------------------------------------------------------------------------
TwoWire* getWireInterface(uint8_t i2cIndex)
{
  #if TARGET_BOARD == DUE_BOARD
    if(i2cIndex == 1)
    {
      static bool _isWire1Inited = false;
      if(!_isWire1Inited)
      {
        _isWire1Inited = true;
        Wire1.begin();
      }
      return &Wire1;
    }
  #endif

  static bool _isWireInited = false;
  if(!_isWireInited)
  {
    _isWireInited = true;
    Wire.begin();
  }

  return &Wire;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
// DateTimeData
//--------------------------------------------------------------------------------------------------------------------------------------
DateTimeData::DateTimeData()
{
  day = 0;
  month = 0;
  year = 0;
  
  hour = 0;
  minute = 0;
  second = 0;

  dayOfWeek = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DateTimeData::operator <(const DateTimeData& rhs)
{
  uint32_t this_ut = unixtime();
  uint32_t rhs_ut = rhs.unixtime();

  return (this_ut < rhs_ut);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DateTimeData::operator <=(const DateTimeData& rhs)
{
  uint32_t this_ut = unixtime();
  uint32_t rhs_ut = rhs.unixtime();

  return (this_ut <= rhs_ut);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DateTimeData::operator >(const DateTimeData& rhs)
{
  uint32_t this_ut = unixtime();
  uint32_t rhs_ut = rhs.unixtime();

  return (this_ut > rhs_ut);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DateTimeData::operator >=(const DateTimeData& rhs)
{
  uint32_t this_ut = unixtime();
  uint32_t rhs_ut = rhs.unixtime();

  return (this_ut >= rhs_ut);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DateTimeData::operator ==(const DateTimeData& rhs)
{
  uint32_t this_ut = unixtime();
  uint32_t rhs_ut = rhs.unixtime();

  return (this_ut == rhs_ut);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DateTimeData::operator !=(const DateTimeData& rhs)
{
  return !operator==(rhs);
}
//--------------------------------------------------------------------------------------------------------------------------------------
DateTimeData::operator String()
{
 String result;

    if(day < 10)
      result = '0';
    result += day;
    
    result += '.';

    if(month < 10)
      result += '0';
    result += month;
    
    result += '.';

    result += year;
    
    result += ' ';

    if(hour < 10)
      result += '0';
    result += hour;
    
    result += ':';

    if(minute < 10)
      result += '0';
    result += minute;
    
    result += ':';

    if(second < 10)
      result += '0';
    result += second;
    
    return result;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
const uint8_t daysArray [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };
//--------------------------------------------------------------------------------------------------------------------------------------
uint32_t DateTimeData::unixtime(void) const
{
  uint32_t u;

  u = time2long(date2days(this->year, this->month, this->day), this->hour, this->minute, this->second);
  u += 946684800; // + 01.01.2000 00:00:00 unixtime

  return u;
}
//--------------------------------------------------------------------------------------------------------------------------------------
DateTimeData DateTimeData::addDays(long days)
{
  uint32_t ut = unixtime();
  long diff = days*86400;
  ut += diff;  
  return fromUnixtime(ut);
}
//--------------------------------------------------------------------------------------------------------------------------------------
DateTimeData DateTimeData::fromUnixtime(uint32_t time)
{
  DateTimeData result;

  uint8_t year;
  uint8_t month, monthLength;
  unsigned long days;

  result.second = time % 60;
  time /= 60; // now it is minutes
  result.minute = time % 60;
  time /= 60; // now it is hours
  result.hour = time % 24;
  time /= 24; // now it is days
  
  year = 0;  
  days = 0;
  while((unsigned)(days += (isLeapYear(year) ? 366 : 365)) <= time) {
    year++;
  }
  result.year = year + 1970; // year is offset from 1970 
  
  days -= isLeapYear(year) ? 366 : 365;
  time  -= days; // now it is days in this year, starting at 0
  
  days=0;
  month=0;
  monthLength=0;
  for (month=0; month<12; month++) 
  {
    if (month==1) 
    { // february
      if (isLeapYear(year)) 
      {
        monthLength=29;
      } 
      else 
      {
        monthLength=28;
      }
    } 
    else 
    {
      monthLength = pgm_read_byte(daysArray + month);
    }
    
    if (time >= monthLength) 
    {
      time -= monthLength;
    } 
    else 
    {
        break;
    }
  }
  result.month = month + 1;  // jan is month 1  
  result.day = time + 1;     // day of month  

    int dow;
    byte mArr[12] = {6,2,2,5,0,3,5,1,4,6,2,4};
    dow = (result.year % 100);
    dow = dow*1.25;
    dow += result.day;
    dow += mArr[result.month-1];
    
    if (isLeapYear(result.year) && (result.month<3))
     dow -= 1;
     
    while (dow>7)
     dow -= 7;

   result.dayOfWeek = dow;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DateTimeData::isLeapYear(uint16_t year)
{
  return (year % 4 == 0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t DateTimeData::date2days(uint16_t _year, uint8_t _month, uint8_t _day) const
{
    _year = _year - 2000;

    uint16_t days16 = _day;

    for (uint8_t i = 1; i < _month; ++i)
    {
        days16 += pgm_read_byte(daysArray + i - 1);
    }

    if ((_month == 2) && isLeapYear(_year))
    {
        ++days16;
    }

    return days16 + 365 * _year + (_year + 3) / 4 - 1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
long DateTimeData::time2long(uint16_t days, uint8_t hours, uint8_t minutes, uint8_t seconds) const
{    
    return ((days * 24L + hours) * 60 + minutes) * 60 + seconds;
}
//--------------------------------------------------------------------------------------------------------------------------------------
// DistanceData
//--------------------------------------------------------------------------------------------------------------------------------------
uint32_t DistanceData::raw() const
{
  uint32_t result = Value*100;
  result += Fract;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DistanceData::operator==(const DistanceData& rhs)
{
  uint32_t this_val = raw();
  uint32_t rhs_val = rhs.raw();

   return (this_val == rhs_val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DistanceData::operator!=(const DistanceData& rhs)
{
  return !operator==(rhs);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DistanceData::operator<(const DistanceData& rhs)
{
  uint32_t this_val = raw();
  uint32_t rhs_val = rhs.raw();

   return (this_val < rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DistanceData::operator<=(const DistanceData& rhs)
{
  uint32_t this_val = raw();
  uint32_t rhs_val = rhs.raw();

   return (this_val <= rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DistanceData::operator>(const DistanceData& rhs)
{
  uint32_t this_val = raw();
  uint32_t rhs_val = rhs.raw();

   return (this_val > rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DistanceData::operator>=(const DistanceData& rhs)
{
  uint32_t this_val = raw();
  uint32_t rhs_val = rhs.raw();

   return (this_val >= rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
DistanceData::operator String()
{
    String result;
    result = Value;
    
    result += Core.FractDelimiter;
    if(Fract < 10)
      result += '0';

     result += Fract;

     return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// TemperatureData
//--------------------------------------------------------------------------------------------------------------------------------------
int32_t TemperatureData::raw() const
{
  int32_t result = abs(Value)*100;
  result += Fract;

  if(Value < 0)
    result = -result;

    return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TemperatureData::operator==(const TemperatureData& rhs)
{
  int32_t this_val = raw();
  int32_t rhs_val = rhs.raw();

   return (this_val == rhs_val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TemperatureData::operator!=(const TemperatureData& rhs)
{
  return !operator==(rhs);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TemperatureData::operator<(const TemperatureData& rhs)
{
  int32_t this_val = raw();
  int32_t rhs_val = rhs.raw();

   return (this_val < rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TemperatureData::operator<=(const TemperatureData& rhs)
{
  int32_t this_val = raw();
  int32_t rhs_val = rhs.raw();

   return (this_val <= rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TemperatureData::operator>(const TemperatureData& rhs)
{
  int32_t this_val = raw();
  int32_t rhs_val = rhs.raw();

   return (this_val > rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TemperatureData::operator>=(const TemperatureData& rhs)
{
  int32_t this_val = raw();
  int32_t rhs_val = rhs.raw();

   return (this_val >= rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
TemperatureData::operator String()
{
    String result;
    result = Value;
    
    result += Core.FractDelimiter;
    if(Fract < 10)
      result += '0';

     result += Fract;

     return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
TemperatureData TemperatureData::ConvertToFahrenheit(const TemperatureData& from)
{  
    TemperatureData result;
      
    int32_t rawC = from.Value*100 + from.Fract;
    int32_t rawF = (rawC*9)/5 + 3200;

    result.Value = rawF/100;
    result.Fract = rawF%100;

    return result;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// PressureData
//--------------------------------------------------------------------------------------------------------------------------------------
PressureData PressureData::ConvertToMmHg(const PressureData& from)
{
  PressureData result;
  
  int32_t rawP = from.Value*100 + from.Fract;
  int32_t rawMm = (rawP*1.0)/133.3;

  result.isInPA = false;
  result.Value = rawMm/100;
  result.Fract = rawMm%100;

  return result;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
int32_t PressureData::raw() const
{
  int32_t result = abs(Value)*100;
  result += Fract;

  if(Value < 0)
    result = -result;

    return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool PressureData::operator==(const PressureData& rhs)
{
  int32_t this_val = raw();
  int32_t rhs_val = rhs.raw();

   return (this_val == rhs_val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool PressureData::operator!=(const PressureData& rhs)
{
  return !operator==(rhs);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool PressureData::operator<(const PressureData& rhs)
{
  int32_t this_val = raw();
  int32_t rhs_val = rhs.raw();

   return (this_val < rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool PressureData::operator<=(const PressureData& rhs)
{
  int32_t this_val = raw();
  int32_t rhs_val = rhs.raw();

   return (this_val <= rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool PressureData::operator>(const PressureData& rhs)
{
  int32_t this_val = raw();
  int32_t rhs_val = rhs.raw();

   return (this_val > rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool PressureData::operator>=(const PressureData& rhs)
{
  int32_t this_val = raw();
  int32_t rhs_val = rhs.raw();

   return (this_val >= rhs_val);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
PressureData::operator String()
{
    String result;
    result = Value;
    
    result += Core.FractDelimiter;
    if(Fract < 10)
      result += '0';

     result += Fract;

     return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreSensorsFactory
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensor* CoreSensorsFactory::createSensor(CoreSensorType type)
{
  switch(type)
  {
    case BH1750:
    
    #ifdef CORE_BH1750_ENABLED
      return new CoreSensorBH1750();
    #else
      return NULL;
    #endif

    case MAX44009:
    
    #ifdef CORE_MAX44009_ENABLED
      return new CoreSensorMAX44009();
    #else
      return NULL;
    #endif

    case HCSR04:
    
    #ifdef CORE_HCSR04_ENABLED
      return new CoreSensorHCSR04();
    #else
      return NULL;
    #endif
    

    case Si7021:
    
    #ifdef CORE_SI7021_ENABLED
      return new CoreSensorSi7021(); 
    #else
      return NULL;
    #endif

    case DHT:

    #ifdef CORE_DHT_ENABLED
      return new CoreSensorDHT();
    #else
      return NULL;
    #endif

    case DS3231:
  
    #ifdef CORE_DS3231_ENABLED
      return new CoreSensorDS3231(false);
    #else
      return NULL;
    #endif

    case DS3231Temperature:
  
    #ifdef CORE_DS3231_ENABLED
      return new CoreSensorDS3231(true);
    #else
      return NULL;
    #endif

    case MAX6675:
    
    #ifdef CORE_MAX6675_ENABLED
      return new CoreSensorMAX6675();
    #else
      return NULL;
    #endif

    case BMP180:
    
    #ifdef CORE_BMP180_ENABLED
      return new CoreSensorBMP180();
    #else
      return NULL;
    #endif
    

    case DS18B20:

    #ifdef CORE_DS18B20_ENABLED
      return new CoreSensorDS18B20();
    #else
      return NULL;
    #endif

    case DigitalPortState:
    
    #ifdef CORE_DIGITALPORT_ENABLED
      return new CoreSensorDigitalPort();
    #else
      return NULL;
    #endif 

    case AnalogPortState:
    
    #ifdef CORE_ANALOGPORT_ENABLED
      return new CoreSensorAnalogPort();
    #else
      return NULL;
    #endif            

    case UserDataSensor:
    
    #ifdef CORE_USERDATA_SENSOR_ENABLED
      return new CoreUserDataSensor();
    #else
      return NULL;
    #endif            

    //TODO: Тут добавлять создание других датчиков!!!
      
    default:
      return NULL;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreSensor
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensor::CoreSensor(CoreSensorType t)
{
  type = t;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensor::update()
{
  // nop
}
//--------------------------------------------------------------------------------------------------------------------------------------
String CoreSensor::getUnit(CoreDataType type, uint8_t anyFlag)
{
  switch(type)
  {
    case Temperature:
      return F("*"); // градусы
      
    case Humidity:
      return F("%"); // проценты

    case Luminosity:
      return F(" lux"); //люксы

    case UnknownType:
    case DateTime:
    case DigitalPort:      
    case AnalogPort:
    case UserData:
    case Barometric:
      return "";

    case Altitude:
      return "m";

    case Pressure:
    
      if(anyFlag)
        return "Pa";
      else
        return "mm";

    case Distance:
      return "mm";
  }

  return "";
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDataType CoreSensor::getDataType(CoreSensorType type)
{
  switch(type)
  {
    case DS18B20:
    case DS3231Temperature:
    case MAX6675:
      return Temperature;
      
    case DHT:
    case Si7021:
      return Humidity;
      
    case BH1750:
    case MAX44009:
      return Luminosity;

    case HCSR04:
      return Distance;

    case DS3231:
      return DateTime;

    case DigitalPortState:
      return DigitalPort;

    case AnalogPortState:
      return AnalogPort;

    case UserDataSensor:
      return UserData;

    case Unknown:
      return UnknownType;

    case BMP180:
      return Barometric;


  }
  return UnknownType;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensor::getDataSize()
{
  return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t CoreSensor::startMeasure()
{
  return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensor::read(uint8_t* buffer)
{
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_BH1750_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreSensorBH1750
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorBH1750::CoreSensorBH1750() : CoreSensor(BH1750)
{
 i2cIndex = 0; 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorBH1750::writeByte(uint8_t toWrite)
{
  TwoWire* wire = getWireInterface(i2cIndex);
  wire->beginTransmission(deviceAddress);
  wire->write(toWrite);
  wire->endTransmission();  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorBH1750::begin(uint8_t* configData)
{
  i2cIndex = *configData++;
  deviceAddress = *configData;

  writeByte(0x01); // включаем датчик
  writeByte(0x10); // режим Continuous High Resolution
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorBH1750::getDataSize()
{
  return sizeof(LuminosityData);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorBH1750::read(uint8_t* buffer)
{
  bool result = false;
  uint16_t curLuminosity = 0;

  TwoWire* wire = getWireInterface(i2cIndex);

  wire->beginTransmission(deviceAddress); // начинаем опрос датчика освещенности
  
 if(wire->requestFrom((int)deviceAddress, 2) == 2)// ждём два байта
 {
  // читаем два байта
  curLuminosity = wire->read();
  curLuminosity <<= 8;
  curLuminosity |= wire->read();
  curLuminosity = curLuminosity/1.2; // конвертируем в люксы

  LuminosityData lum;
  lum.Value = curLuminosity;
  memcpy(buffer,&lum,sizeof(LuminosityData));
  
  result = true;
 }

  wire->endTransmission();
  return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_BH1750_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_MAX44009_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
// CORE_MAX44009_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorMAX44009::CoreSensorMAX44009() : CoreSensor(MAX44009)
{
 i2cIndex = 0; 
 deviceAddress = MAX44009_ADDRESS1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorMAX44009::begin(uint8_t* configData)
{
  i2cIndex = *configData++;
  deviceAddress = *configData;

  TwoWire* wire = getWireInterface(i2cIndex);

  wire->beginTransmission(deviceAddress);
  
  // выбираем регистр конфигурации
  wire->write(0x02);
  // пишем в него - непрерывный режим измерения, время интегрирования - 800 ms
  wire->write(0x40);
  
  wire->endTransmission();   
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorMAX44009::getDataSize()
{
  return sizeof(LuminosityData);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorMAX44009::read(uint8_t* buffer)
{

  TwoWire* wire = getWireInterface(i2cIndex);
     
  unsigned int data[2] = {0};

  wire->beginTransmission(deviceAddress);
  
  // регистр данных
  wire->write(0x03);
  wire->endTransmission();

  // ждём два байта
  if(wire->requestFrom(deviceAddress, uint8_t(2)) == uint8_t(2))
  {
    data[0] = wire->read();
    data[1] = wire->read();
  }
  else
    return false;

  // Convert the data to lux
  int exponent = (data[0] & 0xF0) >> 4;
  int mantissa = ((data[0] & 0x0F) << 4) | (data[1] & 0x0F);
  float luminosity = pow(2, exponent) * mantissa * 0.045;

  LuminosityData lum;
  lum.Value = luminosity; // пока отбрасываем дробную часть
  memcpy(buffer,&lum,sizeof(LuminosityData));    

  return true;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_MAX44009_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_BMP180_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreSensorBMP180
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorBMP180::CoreSensorBMP180() : CoreSensor(BMP180)
{
 i2cIndex = 0;
 homeplacePressure = 101500;
 isInPascalsMeasure = true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorBMP180::begin(uint8_t* configData)
{
  i2cIndex = *configData++; // читаем индекс I2C
  isInPascalsMeasure = *configData++; // читаем вид измерений (в паскалях или мм.рт.ст)
  memcpy(&homeplacePressure,configData,sizeof(homeplacePressure)); // читаем нормальное давление для нашего местоположения

  doBegin();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorBMP180::getDataSize()
{
  return sizeof(BarometricData);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorBMP180::read(uint8_t* buffer)
{
  bool result = true;

  float temp = readTemperature();
  int32_t pressurePascals = readPressure();
  float altitude = readAltitude(homeplacePressure);

  // сырые данные прочитали, теперь переводим их в нормализованную форму
  BarometricData data;

  // преобразуем температуру во внутреннее представление
  int32_t iTemp = temp*100;
  data.Temperature.Value = iTemp/100;
  data.Temperature.Fract = abs(iTemp%100);

  if(Core.TemperatureUnit == UnitFahrenheit) // измеряем в фаренгейтах
   {
     data.Temperature = TemperatureData::ConvertToFahrenheit(data.Temperature);
   }

   // теперь работаем с давлением. Оно у нас по умолчанию в паскалях
   data.Pressure.Value = pressurePascals;
   data.Pressure.Fract = 0;

   if(!isInPascalsMeasure)
   {
      // тут меряем в мм.рт.ст. - конвертируем
      data.Pressure = PressureData::ConvertToMmHg(data.Pressure);
   }
   // сохраняем признак, как измеряем
   data.Pressure.isInPA = isInPascalsMeasure;

   // теперь получаем высоту
   int iAltitude = altitude*100;
   data.Altitude.Value = iAltitude/100;
   data.Altitude.Fract = abs(iAltitude%100);

   // теперь копируем всё в буфер в памяти
   memcpy(buffer,&data,sizeof(data));
  
  return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorBMP180::doBegin(uint8_t mode) 
{
  if (mode > BMP085_ULTRAHIGHRES) 
    mode = BMP085_ULTRAHIGHRES;
    
  oversampling = mode;

  TwoWire* wire = getWireInterface(i2cIndex);

  wire->begin();

  if (read8(0xD0) != 0x55) 
    return false;

  /* read calibration data */
  ac1 = read16(BMP085_CAL_AC1);
  ac2 = read16(BMP085_CAL_AC2);
  ac3 = read16(BMP085_CAL_AC3);
  ac4 = read16(BMP085_CAL_AC4);
  ac5 = read16(BMP085_CAL_AC5);
  ac6 = read16(BMP085_CAL_AC6);

  b1 = read16(BMP085_CAL_B1);
  b2 = read16(BMP085_CAL_B2);

  mb = read16(BMP085_CAL_MB);
  mc = read16(BMP085_CAL_MC);
  md = read16(BMP085_CAL_MD);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int32_t CoreSensorBMP180::computeB5(int32_t UT) 
{
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t CoreSensorBMP180::readRawTemperature(void) 
{
  write8(BMP085_CONTROL, BMP085_READTEMPCMD);
  delay(5);
  return read16(BMP085_TEMPDATA);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint32_t CoreSensorBMP180::readRawPressure(void) 
{
  uint32_t raw;

  write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP085_ULTRALOWPOWER) 
    delay(5);
  else if (oversampling == BMP085_STANDARD) 
    delay(8);
  else if (oversampling == BMP085_HIGHRES) 
    delay(14);
  else 
    delay(26);

  raw = read16(BMP085_PRESSUREDATA);

  raw <<= 8;
  raw |= read8(BMP085_PRESSUREDATA+2);
  raw >>= (8 - oversampling);

  return raw;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int32_t CoreSensorBMP180::readPressure(void) 
{
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = readRawTemperature();
  UP = readRawPressure();

  B5 = computeB5(UT);

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

  if (B7 < 0x80000000) 
  {
    p = (B7 * 2) / B4;
  } 
  else 
  {
    p = (B7 / B4) * 2;
  }
  
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791)>>4);

  return p;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int32_t CoreSensorBMP180::readSealevelPressure(float altitude_meters) 
{
  float pressure = readPressure();
  return (int32_t)(pressure / pow(1.0-altitude_meters/44330, 5.255));
}
//--------------------------------------------------------------------------------------------------------------------------------------
float CoreSensorBMP180::readTemperature(void) 
{
  int32_t UT, B5;     // following ds convention
  float temp;

  UT = readRawTemperature();

  B5 = computeB5(UT);
  temp = (B5+8) >> 4;
  temp /= 10;
  
  return temp;
}
//--------------------------------------------------------------------------------------------------------------------------------------
float CoreSensorBMP180::readAltitude(float sealevelPressure) 
{
  float altitude;

  float pressure = readPressure();

  altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

  return altitude;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorBMP180::read8(uint8_t a) 
{
  uint8_t ret;

  TwoWire* wire = getWireInterface(i2cIndex);

  wire->beginTransmission(BMP085_I2CADDR); // start transmission to device 
  wire->write(a); // sends register address to read from
  wire->endTransmission(); // end transmission
  
  wire->beginTransmission(BMP085_I2CADDR); // start transmission to device 
  wire->requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
  ret = wire->read(); // receive DATA
  wire->endTransmission(); // end transmission

  return ret;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t CoreSensorBMP180::read16(uint8_t a) 
{
  uint16_t ret;

  TwoWire* wire = getWireInterface(i2cIndex);

  wire->beginTransmission(BMP085_I2CADDR); // start transmission to device 
  wire->write(a); // sends register address to read from
  wire->endTransmission(); // end transmission
  
  wire->beginTransmission(BMP085_I2CADDR); // start transmission to device 
  wire->requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
  ret = wire->read(); // receive DATA
  ret <<= 8;
  ret |= wire->read(); // receive DATA
  wire->endTransmission(); // end transmission

  return ret;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorBMP180::write8(uint8_t a, uint8_t d) 
{
  TwoWire* wire = getWireInterface(i2cIndex);
  
  wire->beginTransmission(BMP085_I2CADDR); // start transmission to device 
  wire->write(a); // sends register address to read from
  wire->write(d);  // write data
  wire->endTransmission(); // end transmission
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_BMP180_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SI7021_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorSi7021::CoreSensorSi7021() : CoreSensor(Si7021)
{
  i2cIndex = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorSi7021::begin(uint8_t* configData)
{
  i2cIndex = *configData;
  sensor.begin(i2cIndex);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorSi7021::getDataSize()
{
  return sizeof(TemperatureData)*2;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorSi7021::read(uint8_t* buffer)
{
  float temperature = sensor.readTemperature();
  float humidity = sensor.readHumidity();

  uint8_t humError = (uint8_t) humidity;
  uint8_t tempError = (uint8_t) temperature;
  bool hasData = true;

  if(humError == HTU21D_ERROR || tempError == HTU21D_ERROR)
  {
    hasData = false;
  }
  else
  {    
    int32_t iTmp = temperature*100;

    int16_t tValue = iTmp/100;
    uint8_t tFract = abs(iTmp)%100;   

    if(tValue < -40 || tValue > 125) // плохая, негодная температура
      hasData = false;
    else
    {
      TemperatureData temp;
      temp.Value = tValue;
      temp.Fract = tFract;
      if(Core.TemperatureUnit == UnitFahrenheit) // измеряем в фаренгейтах
      {
        temp = TemperatureData::ConvertToFahrenheit(temp);
      }

      memcpy(buffer,&temp,sizeof(TemperatureData));
      
    }

      if(hasData)
      {
          iTmp = humidity*100;
          
          tValue = iTmp/100;
          tFract = abs(iTmp)%100;
      
          if(tValue < 0 || tValue > 100) // плохая, негодная влажность
            hasData = false;
          else
          {
            TemperatureData humidity;
            humidity.Value = tValue;
            humidity.Fract = tFract;

            memcpy((buffer + sizeof(TemperatureData)),&humidity,sizeof(TemperatureData));
          }
        
      } // hasData
    
  } // no reading errors

   return hasData;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_SI7021_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_HCSR04_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorHCSR04::CoreSensorHCSR04() : CoreSensor(HCSR04)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorHCSR04::getDataSize()
{
  return sizeof(DistanceData);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorHCSR04::begin(uint8_t* configData)
{
  triggerPin = *configData++;
  echoPin = *configData;

  pinMode(triggerPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorHCSR04::read(uint8_t* buffer)
{
  DistanceData data;

  int32_t duration;
  float mm;
  
  digitalWrite(triggerPin, LOW); 
  delayMicroseconds(2); 
  
  digitalWrite(triggerPin, HIGH); 
  delayMicroseconds(10); 
  
  digitalWrite(triggerPin, LOW); 
  duration = pulseIn(echoPin, HIGH); 
  
  mm = duration / 5.8;

  uint32_t raw = mm*100;
  data.Value = raw/100;
  data.Fract = raw%100;

  memcpy(buffer,&data,sizeof(DistanceData));
  
  return true;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_HCSR04_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_DIGITALPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorDigitalPort::CoreSensorDigitalPort() : CoreSensor(DigitalPortState)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorDigitalPort::getDataSize()
{
  return sizeof(DigitalPortData);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorDigitalPort::begin(uint8_t* configData)
{
  pin = *configData;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorDigitalPort::read(uint8_t* buffer)
{
  DigitalPortData data;
  data.Pin = pin;
  data.Value = Core.getPinState(pin);

  memcpy(buffer,&data,sizeof(DigitalPortData));
  
  return true;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_DIGITALPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ANALOGPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorAnalogPort::CoreSensorAnalogPort() : CoreSensor(AnalogPortState)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorAnalogPort::getDataSize()
{
  return sizeof(AnalogPortData);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorAnalogPort::begin(uint8_t* configData)
{
  pin = *configData;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorAnalogPort::read(uint8_t* buffer)
{
 AnalogPortData data;
 data.Pin = pin;
 data.Value = analogRead(pin);

 memcpy(buffer,&data,sizeof(AnalogPortData));
    
  return true;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_ANALOGPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_USERDATA_SENSOR_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreUserDataSensor::CoreUserDataSensor() : CoreSensor(UserDataSensor)
{
  data = NULL;
  dataSize = 0;
  userDataType = UserData;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreUserDataSensor::~CoreUserDataSensor()
{
  delete [] data;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreUserDataSensor::getDataSize()
{
  return dataSize;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreUserDataSensor::setData(uint8_t* dt,uint8_t sz)
{
  delete [] data;
  data = NULL;
  dataSize = sz;
  
  if(sz && dt)
  {
    data = new uint8_t[sz];
    memcpy(data,dt,sz);
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreUserDataSensor::begin(uint8_t* configData)
{
  delete [] data;
  data = NULL;
  dataSize = *configData;
  if(dataSize)
  {
    data = new uint8_t[dataSize];
    memset(data,0,dataSize);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreUserDataSensor::read(uint8_t* buffer)
{
  if(!data || !dataSize || !buffer)
    return false;
    
   memcpy(buffer,data,dataSize);
  
  return true;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_USERDATA_SENSOR_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_DHT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorDHT::CoreSensorDHT() : CoreSensor(DHT)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorDHT::getDataSize()
{
  return sizeof(TemperatureData)*2;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorDHT::begin(uint8_t* configData)
{
  sensorType = (DHTType) *configData++;
  pin = *configData;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorDHT::read(uint8_t* buffer)
{

  uint8_t wakeup_delay = DHT2x_WAKEUP;
  
  if(sensorType == DHT_11)
    wakeup_delay = DHT11_WAKEUP;

  const uint32_t mstcc = ( F_CPU / 40000 ); // сторож таймаута - 100us

  uint8_t bit = digitalPinToBitMask(pin);
  #if TARGET_BOARD == MEGA_BOARD || TARGET_BOARD == ATMEGA328_BOARD
  uint8_t 
  #elif TARGET_BOARD == DUE_BOARD
  Pio*
  #elif TARGET_BOARD == ESP_BOARD
    #error "NOT IMPLEMENTED!!!" 
  #else
    #error "Unknown target board!"
  #endif
  port = digitalPinToPort(pin);
  
  volatile 
  #if TARGET_BOARD == MEGA_BOARD || TARGET_BOARD == ATMEGA328_BOARD
  uint8_t*
  #elif TARGET_BOARD == DUE_BOARD
  RoReg*
  #elif TARGET_BOARD == ESP_BOARD
    #error "NOT IMPLEMENTED!!!" 
  #else
    #error "Unknown target board!"
  #endif  
  PIR = portInputRegister(port);

  // начинаем читать с датчика
  pinMode(pin,OUTPUT);
  digitalWrite(pin,LOW); // прижимаем к земле
  delay(wakeup_delay); // и ждём, пока датчик прочухается
  digitalWrite(pin,HIGH); // поднимаем линию
  delayMicroseconds(40); // ждём 40us, как написано в даташите
  pinMode(pin, INPUT_PULLUP); // переводим пин на чтение

  // тут должны проверить последовательность, которую выдал датчик:
  // если линия прижата на 80us, затем поднята на 80us - значит,
  // датчик готов выдавать данные
  
  uint32_t tmout_guard = mstcc;
  
  while ((*PIR & bit) == LOW ) // читаем, пока низкий уровень на пине
  {
    if(!--tmout_guard)
     return false; // таймаут поймали
  }
  tmout_guard = mstcc;
  while ((*PIR & bit) != LOW ) // читаем, пока высокий уровень на пине
  {
    if(!--tmout_guard)
     return false; // таймаут поймали
  }

  // считаем, что теперь пойдут данные. нам надо получить 40 бит, т.е. 5 байт.
  uint8_t bytes[5] = {0}; // байты, в которые мы будем принимать данные
  for(uint8_t i=0;i<5;i++)
    bytes[i] = 0;
  
  uint8_t idx = 0; // индекс текущего байта
  uint8_t bitmask = 0x80; // старший бит байта установлен в единичку, его будем двигать вниз
  
  for(uint8_t i=0;i<40;i++)
  {
      // сначала ждём 50us, говорящие, что пойдёт следующий бит
      tmout_guard = mstcc;
      while ((*PIR & bit) == LOW )
      {
        if(!--tmout_guard)
            return false; // таймаут поймали
      } // while

      // теперь принимаем бит. Если время подтянутой вверх линии более 40us - это единица, иначе - ноль.

      tmout_guard = mstcc;
      uint32_t tMicros = micros();
      while ((*PIR & bit) != LOW )
      {
        if(!--tmout_guard)
            return false; // таймаут поймали
      } // while

      if(micros() - tMicros > 40) // единичка
      {
        bytes[idx] |= bitmask;
      }

      // сдвигаем маску вправо
      bitmask >>= 1;
      
      if(!bitmask) // дошли до конца байта
      {
        bitmask = 0x80;
        idx++; // читаем в следующий байт
      }
        
  } // for

  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH); // поднимаем линию, говоря датчику, что он свободен


  int16_t temperatureValue = 0;
  uint8_t temperatureFract = 0;
  
  int16_t humidityValue = 0;
  uint8_t humidityFract = 0;


  // проверяем принятые данные
  switch(sensorType)
  {
    case DHT_11:
    {
      uint8_t crc = bytes[0] + bytes[2];
      if(crc != bytes[4]) // чексумма не сошлась
        return false;

     // сохраняем данные
     temperatureValue = bytes[2]; // температура
     humidityValue = bytes[0]; // влажность
     
    }
    break;

    case DHT_2x:
    {
      uint8_t crc = bytes[0] + bytes[1] + bytes[2] + bytes[3];
      if(crc != bytes[4]) // чексумма не сошлась
        return false;

     // сохраняем данные
      unsigned long rh = ((bytes[0] << 8) + bytes[1])*10;
      // влажность
      humidityValue = rh/100;
      humidityFract = rh%100;

     long temp = (((bytes[2] & 0x7F) << 8) + bytes[3])*10;
      
      temperatureValue =  temp/100;
      if(bytes[2] & 0x80) // температура ниже нуля
        temperatureValue = -temperatureValue;
        
      temperatureFract = temp%100;
           
    }
    break;
  } // switch
 
  if(humidityValue < 0 || humidityValue > 100)
  {
    return false;
  }

  if(temperatureValue < -40 || temperatureValue > 80)
  {
    return false;
  }

 TemperatureData temp;
 temp.Value = temperatureValue;
 temp.Fract = temperatureFract;

 if(Core.TemperatureUnit == UnitFahrenheit) // измеряем в фаренгейтах
 {
  temp = TemperatureData::ConvertToFahrenheit(temp);
 }  
  memcpy(buffer,&temp,sizeof(TemperatureData));

  temp.Value = humidityValue;
  temp.Fract = humidityFract;
  memcpy((buffer + sizeof(TemperatureData)),&temp,sizeof(TemperatureData));
  
  return true;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_DHT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_MAX6675_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#include <SPI.h>
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorMAX6675::CoreSensorMAX6675() : CoreSensor(MAX6675)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorMAX6675::getDataSize()
{
  return sizeof(TemperatureData);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorMAX6675::begin(uint8_t* configData)
{
  cs = *configData++;
  slow = *configData == 1;

  if(!__isSPIInited)
  {
    __isSPIInited = true;
    SPI.begin();
  }
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
double CoreSensorMAX6675::readCelsius() 
{
  uint16_t v;

  digitalWrite(cs, LOW);

  delayMicroseconds(1000);

  #if TARGET_BOARD == DUE_BOARD
  
    // поддержка медленного режима под Due
    
    uint8_t currentDivider = SPI_CLOCK_DIV4;
    if(slow)
    {
      uint8_t spiChannel = BOARD_PIN_TO_SPI_CHANNEL(SPI_INTERFACE_ID);
      uint32_t csr = SPI_INTERFACE->SPI_CSR[spiChannel];
      currentDivider = (csr & SPI_CSR_SCBR_Msk) >> 8;
      SPI.setClockDivider(SPI_CLOCK_DIV128);
    }
  #else

     // AVR
     uint8_t oldSPCR = SPCR;
    
    if(slow)
    {
        SPCR |= 3; // As slow as possible (clock/128 or clock/64 depending on SPI2X)
    }
  #endif
  
  v = SPI.transfer16(0);

  #if TARGET_BOARD == DUE_BOARD  
  
    // поддержка медленного режима под Due
    
    if(slow)
    {
      SPI.setClockDivider(currentDivider);
    }
    
  #else
  
    // AVR    
    if(slow)
    {
        SPCR = oldSPCR;
    }
    
  #endif
  
  digitalWrite(cs, HIGH);
  // CSB Rise to Output Disable
  delayMicroseconds(1000);

  if (v & 0x4) 
  {
    // не найдено термопары
    return NAN;
  }

  v >>= 3;

  return v*0.25;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorMAX6675::read(uint8_t* buffer)
{

  double val = readCelsius();
  
  if(isnan(val))
    return false;

  int32_t iVal = val*100;

  TemperatureData temp;
    
  temp.Value = iVal/100;
  temp.Fract = abs(iVal)%100;

  memcpy(buffer,&temp,sizeof(TemperatureData));
  
  return true;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_MAX6675_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_DS3231_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorDS3231::CoreSensorDS3231(bool tempOnly) : CoreSensor(DS3231)
{
  isTempOnly = tempOnly;
  i2cIndex = 0;
  
  if(isTempOnly)
    type = DS3231Temperature;
 else
    type = DS3231;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorDS3231::getDataSize()
{
  if(!isTempOnly)
    return sizeof(DateTimeData); // дата/время, день недели
  else
    return sizeof(TemperatureData); // только температура
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorDS3231::begin(uint8_t* configData)
{
  i2cIndex = *configData;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorDS3231::read(uint8_t* buffer)
{
  TwoWire* wire = getWireInterface(i2cIndex);
  
  if(!isTempOnly)
  {  
      wire->beginTransmission(DS3231Address);
      wire->write(0); // говорим, что мы собираемся читать с регистра 0
      
      if(wire->endTransmission() != 0) // ошибка
        return false;
    
      
      if(wire->requestFrom(DS3231Address, 7) == 7) // читаем 7 байт, начиная с регистра 0
      {
          
          uint8_t second = bcd2dec(wire->read() & 0x7F);
          uint8_t minute = bcd2dec(wire->read());
          uint8_t hour = bcd2dec(wire->read() & 0x3F);
          uint8_t dayOfWeek = bcd2dec(wire->read());
          uint8_t dayOfMonth = bcd2dec(wire->read());
          uint8_t month = bcd2dec(wire->read());
          uint16_t year = bcd2dec(wire->read());     
          year += 2000; // приводим время к нормальному формату
    
          // теперь сохраняем в буфер, в нём всё лежит так:
          /*
            день - 1 байт
            месяц - 1 байт
            год - 2 байта
            час - 1 байт
            минута - 1 байт
            секунда - 1 байт
            день недели - 1 байт
           */
    
           *buffer++ = dayOfMonth;
           *buffer++ = month;
           uint8_t* p = (uint8_t*)&year;
           *buffer++ = *p++;
           *buffer++ = *p;
           *buffer++ = hour;
           *buffer++ = minute;
           *buffer++ = second;
           *buffer++ = dayOfWeek;
          
    
          return true;
      } // if

  }
  else
  {
    // только температура

         // читаем температуру
         wire->beginTransmission(DS3231Address);
         wire->write(0x11);
         
          if(wire->endTransmission() != 0) // ошибка
            return false;        
        
        if(wire->requestFrom(DS3231Address, 2) == 2)
          {
    
            union int16_byte {
               int i;
               uint8_t b[2];
           } rtcTemp;
               
            rtcTemp.b[1] = wire->read();
            rtcTemp.b[0] = wire->read();
        
            long tempC100 = (rtcTemp.i >> 6) * 25;

            TemperatureData temp;
            
            temp.Value = tempC100/100;
            temp.Fract = abs(tempC100 % 100);
            if(Core.TemperatureUnit == UnitFahrenheit) // измеряем в фаренгейтах
            {
             temp = TemperatureData::ConvertToFahrenheit(temp); 
            }
            
            memcpy(buffer,&temp,sizeof(TemperatureData));
 

            return true;
            
          }    
  } // else isTempOnly
  
  return false;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorDS3231::dec2bcd(uint8_t val)
{
  return( (val/10*16) + (val%10) );
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorDS3231::bcd2dec(uint8_t val)
{
  return( (val/16*10) + (val%16) );
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorDS3231::setTime(uint8_t second, uint8_t minute, uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month, uint16_t year)
{
  while(year > 100) // приводим к диапазону 0-99
    year -= 100;

  TwoWire* wire = getWireInterface(i2cIndex);    
 
  wire->beginTransmission(DS3231Address);
  
  wire->write(0); // указываем, что начинаем писать с регистра секунд
  wire->write(dec2bcd(second)); // пишем секунды
  wire->write(dec2bcd(minute)); // пишем минуты
  wire->write(dec2bcd(hour)); // пишем часы
  wire->write(dec2bcd(dayOfWeek)); // пишем день недели
  wire->write(dec2bcd(dayOfMonth)); // пишем дату
  wire->write(dec2bcd(month)); // пишем месяц
  wire->write(dec2bcd(year)); // пишем год
  
  wire->endTransmission();

  delay(10); // немного подождём для надёжности  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_DS3231_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_DS18B20_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDS18B20LineManager::CoreDS18B20LineManager(uint8_t _pin)
{
  pin = _pin;
  flags.inited = false;
  flags.inConversion = false;
  flags.isFirstTimeConversion = true;
  sensorsCount = 0;
  sensorCounter = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDS18B20LineManager::~CoreDS18B20LineManager()
{
  clearAddresses();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDS18B20LineManager::clearAddresses()
{
  for(size_t i=0;i<addresses.size();i++)
  {
    delete [] addresses[i];
  }

  addresses.empty();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDS18B20LineManager::begin(CoreDS18B20Resolution res)
{
  
  if(!pin)
    return;
  
  if(flags.inited) // уже проинициализированы
  {
    return;
  }

   flags.inited = true; // запомнили, что уже вызывали эту функцию


   OneWire ow(pin);
 
  if(!ow.reset()) // нет датчиков на линии
  {
    return;  
  }

   ow.write(0xCC); // пофиг на адреса (SKIP ROM)
   ow.write(0x4E); // запускаем запись в scratchpad

   ow.write(0); // верхний температурный порог 
   ow.write(0); // нижний температурный порог
   ow.write(res); // разрешение датчика

   ow.reset();
   ow.write(0xCC); // пофиг на адреса (SKIP ROM)
   ow.write(0x48); // COPY SCRATCHPAD
   delay(10);
   ow.reset(); 

}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t CoreDS18B20LineManager::startConversion()
{
  // дёргается из startMeasure класса датчика, мы здесь должны смотреть - все ли датчики,
  // линию для которых мы предоставляем, получили время для окончания конвертации.
  // если все - просто возвращаем 0, т.к. после первого вызова конвертации уже не надо ждать - 
  // вернётся однажды замеренная температура, а датчик сам обновит температуру.
  
  // единственная тонкость - интервал между опросом всех датчиков в системе не должен быть меньше,
  // чем время конвертации - в этом случае мы возвращаем разницу, чтобы конвертация гарантированно
  // закончилась до момента начала следующего цикла опроса датчиков
  
  if(!pin)
  {
    return 0;
  }

  if(flags.isFirstTimeConversion) // перед первым вызовом конвертации инициализируемся
  {
    flags.isFirstTimeConversion = false;
    begin();
  }

  sensorCounter++; // запоминаем, сколько датчиков вызвало эту функцию
  if(sensorCounter > sensorsCount)
    sensorCounter = sensorsCount+1;

  uint16_t neededConversionTime = 800; // 800 миллисекунд для первой конвертации - мы должны подождать

  // теперь рассчитываем время конвертации
  if(sensorCounter > sensorsCount)
  {
    // все датчики один разочек уже подождали конвертации, поэтому можно возвращать 0
    if(Core.SensorsUpdateInterval < neededConversionTime)
    {
      // случай, когда обновление показаний происходит чаще, чем время конвертации - 
      // мы должны возвращать разницу
      neededConversionTime =  neededConversionTime - Core.SensorsUpdateInterval;
    }
    else
    {
      // тут всё норм, время между опросами датчиков больше времени конвертации,
      // поэтому можем возвращать 0, и на следующем опросе нам вернётся
      // уже обновлённая температура, зато в этом цикле мы прочитаем
      // без сидения в таймере задержки
      neededConversionTime = 0;
    }
    
  }

  if(flags.inConversion) // уже в процессе конвертации
  {
    return neededConversionTime;
  }

  flags.inConversion = true;

  OneWire ow(pin);

  if(!ow.reset()) // нет датчиков на линии
  {
    return neededConversionTime;
  }

  ow.write(0xCC); // пофиг на адреса (SKIP ROM)
  ow.write(0x44); // запускаем преобразование

  ow.reset(); 

  return neededConversionTime;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreDS18B20LineManager::catchSensor(CoreSensorDS18B20* sensor)
{
  sensorsCount++; // просто запоминаем, сколько датчиков на нашей линии
  uint8_t sensorIndex = (sensorsCount - 1);

  // тут нам надо искать в сохранённых адресах в EEPROM, и привязывать датчик к сохранённому адресу!!!
  uint16_t readAddress = CORE_DS18B20_BINDING_ADDRESS;
  if(Core.memRead(readAddress) != CORE_HEADER1)
  {
    return sensorIndex;
  }
    
   readAddress++;
   
  if(Core.memRead(readAddress) != CORE_HEADER2)
  {
    return sensorIndex;
  }

   readAddress++;

  if(Core.memRead(readAddress) != CORE_HEADER3)
  {
    return sensorIndex;
  }

   readAddress++;

   // мы находимся на начале списка привязок - считаем смещение, у нас одна запись - 8 байт.
   // поскольку датчики добавляются так, как они прописаны в конфиге - последовательно,
   // то нам достаточно просто высчитать смещение чтения в зависимости от индекса датчика
   readAddress += sensorIndex*8;

   uint8_t address[8] = {0};
   for(int i=0;i<8;i++)
   {
    address[i] = Core.memRead(readAddress);
    readAddress++;
   }

   if(OneWire::crc8(address,7) == address[7])
   {

     // адрес сохранён, назначаем его датчику
     uint8_t* newAddress = addAddress(address,sensorIndex);
     sensor->setAddress(newAddress);
   }

  return sensorIndex;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreDS18B20LineManager::addressExists(const uint8_t* address, uint8_t*& existing)
{

  for(size_t i=0;i<addresses.size();i++)
  {
    existing = addresses[i];

    if(!memcmp(existing, address,8))
    {
      return true;
    }
  } // for

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t* CoreDS18B20LineManager::addAddress(const uint8_t* address, size_t sensorIndex)
{
  uint8_t* existing;
  if(addressExists(address,existing))
    return existing;

  if(OneWire::crc8(address,7) != address[7]) // плохой адрес
  {
    return NULL;
  }

  if(addresses.size() <= sensorIndex)
  {
    // нет соответствия - индекс датчика/индекс адреса, добавляем
  
    uint8_t* newAddress = new uint8_t[8];
    memcpy(newAddress,address,8);
    addresses.push_back(newAddress);    
  
    return newAddress;
  }
  else
  {
    // есть соответствие, обновляем
     memcpy(addresses[sensorIndex],address,8);
     return addresses[sensorIndex];
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreDS18B20LineManager::findAddress(OneWire& ow, uint8_t* output, CoreSensorDS18B20* sensor)
{
  // тут смотрим - есть ли у датчика сохранённый адрес с шины?
  if(sensor->getAddress(output)) // адрес уже есть у датчика, не надо искать
  {
    return true;
  }
  
   // сбрасываем поиск
  ow.reset_search(); 

  // если мы ищем здесь, значит, у датчика нет адреса, надо пройти всю шину,
  // получить адреса последовательно, проверить - если адреса нет в известных,
  // значит, надо добавлять в известные, и сохранять этот адрес у датчика.

  while (ow.search(output)) 
  {
    if(isGoodAddress(output))
    {
      // адрес валидный, надо проверить, есть ли он уже в списке
      uint8_t* existing;
      if(!addressExists(output,existing))
      {
        // адреса нет в списке, можно добавлять к нам
        uint8_t* newAddress = addAddress(output,sensor->getIndex());

        // и говорим датчику его адрес
        sensor->setAddress(newAddress);

        // и тут нам надо сохранять этот адрес в EEPROM, потому что мы в будущем должны его вычитать, сохраняя жёсткую привязку адреса к индексу датчика в системе
        serializeAddress(newAddress,sensor);
        
        break;
      }
    } // good address

  } // while  

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDS18B20LineManager::serializeAddress(const uint8_t* address,CoreSensorDS18B20* sensor)
{  
  uint16_t writeAddress = CORE_DS18B20_BINDING_ADDRESS;

  // пишем заголовок
  Core.memWrite(writeAddress,CORE_HEADER1);
  writeAddress++;

  Core.memWrite(writeAddress,CORE_HEADER2);
  writeAddress++;

  Core.memWrite(writeAddress,CORE_HEADER3);
  writeAddress++;

  // вычисляем смещение хранения данных для датчика
  writeAddress += 8*sensor->getIndex();

  // и теперь пишем сам адрес
  for(int i=0;i<8;i++)
  {
    Core.memWrite(writeAddress,*address);
    writeAddress++;
    address++;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreDS18B20LineManager::isGoodAddress(const uint8_t* address)
{
  return (OneWire::crc8(address, 7) == address[7]);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreDS18B20LineManager::readTemperature(int8_t& tempValue, uint8_t& tempFract, CoreSensorDS18B20* sensor)
{ 
   
  if(!pin)
    return false;


  uint8_t sensorIndex = sensor->getIndex();
  
  // какой-то из датчиков попросил прочитать температуру
  // соответственно, мы должны сбросить флаг конвертации, чтобы следующий вызов startConversion прошёл успешно
  if(sensorIndex+1 == sensorsCount )
    flags.inConversion = false;

  // нам надо найти адрес датчика на шине по его индексу
  uint8_t address[8];

  OneWire ow(pin);

  if(!ow.reset()) // нет датчиков, ни одна собака не откликнулась
    return false;


  if(!findAddress(ow,address,sensor))
  {
    return false;
  }

  ow.reset();
  ow.select(address);
  ow.write(0xBE);

  uint8_t data[9] = {0};

  for(uint8_t i=0;i<9;i++)
    data[i] = ow.read();

 if (OneWire::crc8( data, 8) != data[8]) // проверяем контрольную сумму
 {

      if(sensor->isBroken()) // если датчик не отвечал N попыток чтения - он считается сломавшимся
      {
        // сбрасываем ему адрес, и теперь для этого слота будет происходить поиск нового адреса на шине
        sensor->setAddress(NULL);
        memset(addresses[sensor->getIndex()],0,8);
        serializeAddress(addresses[sensor->getIndex()],sensor);
      }
      
      return false;      
 }

  int loByte = data[0];
  int hiByte = data[1];

  int temp = (hiByte << 8) + loByte;
  
  bool isNegative = (temp & 0x8000);
  
  if(isNegative)
    temp = (temp ^ 0xFFFF) + 1;

  // DS18B20 ONLY !!!
  int tc_100 = (6 * temp) + temp/4;

  tempValue = tc_100/100;
  if(isNegative)
    tempValue = -tempValue;
    
  tempFract = tc_100 % 100;

  if(tempValue < -55 || tempValue > 125)
  {
    return false;
  }

 if(Core.TemperatureUnit == UnitFahrenheit) // измеряем в фаренгейтах
 {
  TemperatureData fahren = {tempValue, tempFract}; 
  fahren = TemperatureData::ConvertToFahrenheit(fahren);
  tempValue = fahren.Value;
  tempFract = fahren.Fract;
 }    

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDS18B20DispatcherClass::CoreDS18B20DispatcherClass()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDS18B20DispatcherClass::clear()
{  
  for(size_t i=0;i<list.size();i++)
  {
    delete list[i];
  }

  list.empty();
  //while(list.size())
  //  list.pop();
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDS18B20LineManager* CoreDS18B20DispatcherClass::add(uint8_t pin)
{    
  for(size_t i=0;i<list.size();i++)
  {
    if(list[i]->getPin() == pin)
    {
      return list[i];
    }
  }

  CoreDS18B20LineManager* manager = new CoreDS18B20LineManager(pin);
  list.push_back(manager);

  return manager;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSensorDS18B20::CoreSensorDS18B20() : CoreSensor(DS18B20)
{
 myIndex = 0;
 myAddress = NULL;
 badReadingAttempts = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensorDS18B20::begin(uint8_t* configData)
{ 
  // запоминаем менеджера нашей линии
  myManager = CoreDS18B20Dispatcher.add(*configData);
  myIndex = myManager->catchSensor(this);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t CoreSensorDS18B20::startMeasure()
{
  // просим менеджера линии начать конвертацию
  return myManager->startConversion();
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreSensorDS18B20::getDataSize()
{
  return sizeof(TemperatureData);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSensorDS18B20::read(uint8_t* buffer)
{

  int8_t temp;
  uint8_t tempFract;
  bool result = myManager->readTemperature(temp,tempFract,this);
  if(result)
  {
    TemperatureData tempStruct;
    tempStruct.Value = temp;
    tempStruct.Fract = tempFract;
    
    memcpy(buffer,&tempStruct,sizeof(TemperatureData));

  }
  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDS18B20DispatcherClass CoreDS18B20Dispatcher;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_DS18B20_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------



