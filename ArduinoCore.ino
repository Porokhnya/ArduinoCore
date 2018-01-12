//--------------------------------------------------------------------------------------------------------------------------------------
// определяем наши альясы для пинов
#define power_200  A1               // Контроль источника питания 200в
#define power_3V3  A2              // Контроль источника питания +3.3в
#define power_5V0  A3              // Контроль источника питания +5.0в
//--------------------------------------------------------------------------------------------------------------------------------------
#include "Core.h"
#include "CoreConfig.h"
//--------------------------------------------------------------------------------------------------------------------------------------
CorePinStateAction led13Blinker; // будем мигать на 13 пине светодиодом
//--------------------------------------------------------------------------------------------------------------------------------------
void unhandledCommandHandler(const String& command, Stream* outStream)
{
  outStream->print(CORE_COMMAND_ANSWER_ERROR);
  outStream->println(F("UNKNOWN_COMMAND"));
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ramFree()
{
  Serial.print(F("RAM free: "));
  Serial.println(Core.getFreeMemory());  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// UNIT TEST
//--------------------------------------------------------------------------------------------------------------------------------------
const byte unitTestConfig[] PROGMEM = { // тестовый конфиг в памяти

  // заголовок конфига
  CORE_HEADER1,
  CORE_HEADER2,
  CORE_HEADER3,

    //---------------------------------------
      SensorRecord, // данные о датчике
      '\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      BH1750, // датчик BH1750
      2, // длина данных
      0, // индекс I2C (0 - Wire, 1 - Wire1)
      BH1750Address1, // на первом адресе на шине I2C
    //---------------------------------------
      SensorRecord, // данные о датчике
      '\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      BH1750, // датчик BH1750
      2, // длина данных
      0, // индекс I2C (0 - Wire, 1 - Wire1)
      BH1750Address2, // на втором адресе на шине I2C
    //---------------------------------------
    //TODO: ДЛЯ SI7021 СДЕЛАТЬ ВОЗМОЖНОСТЬ ИМЕТЬ ИХ НЕСКОЛЬКО НА ШИНЕ I2C !!!
      SensorRecord, // данные о датчике
      '\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      Si7021, // датчик Si7021
      1,  // длина данных
      0, // индекс I2C (0 - Wire, 1 - Wire1)
    //---------------------------------------
      SensorRecord, // данные о датчике
      '\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      DHT, // датчик DHT
      2, // длина данных
      DHT_2x, // тип датчика - DHT21
      5, // пин датчика - 5
    //---------------------------------------
      SensorRecord, // данные о датчике
      'R','T','C','\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      DS3231, // датчик часов реального времени DS3231 на шине I2C
      1, // длина данных
      0, // индекс I2C (0 - Wire, 1 - Wire1)
    //---------------------------------------
      SensorRecord, // данные о датчике
      '\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      DS3231Temperature, // температура с датчика часов реального времени DS3231 на шине I2C
      1, // длина данных
      0, // индекс I2C (0 - Wire, 1 - Wire1)
    //---------------------------------------
      SensorRecord, // данные о датчике
      'D', 'S', '1', '\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      DS18B20, // датчик DS18B20
      1, // длина данных
      6, // пин датчика - 6
    //---------------------------------------
      SensorRecord, // данные о датчике
      'D', 'S', '2', '\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      DS18B20, // датчик DS18B20
      1, // длина данных
      6, // пин датчика - 6
    //---------------------------------------
      SensorRecord, // данные о датчике
      '\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      DigitalPortState, // состояние цифрового порта
      1, // длина данных
      13, // пин датчика - 13
    //---------------------------------------
      SensorRecord, // данные о датчике
      '\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      AnalogPortState, // состояние аналогового порта
      1, // длина данных
      power_200, // пин датчика - power_200
    //---------------------------------------
      SensorRecord, // данные о датчике
      '\0', // набор байт, заканчивающийся нулём, содержащий мнемоническое имя датчика (для удобного обращения по имени)
      AnalogPortState, // состояние аналогового порта
      1, // длина данных
      power_3V3, // пин датчика - power_3V3
    //---------------------------------------
      SensorRecord, // данные о датчике
      '5', 'V', '\0', // даём датчику имя "5V"
      AnalogPortState, // состояние аналогового порта
      1, // длина данных
      power_5V0, // пин датчика - power_5V0
    //---------------------------------------
      ESPSettingsRecord, // настройки ESP
      0, // флаг - поднимать ли точку доступа (1 - поднимать, 0 - не поднимать)
      '\0',  // имя точки доступа (набор байт, заканчивающийся нулевым байтом)
      '\0', // пароль точки доступа (набор байт, заканчивающийся нулевым байтом)
      1, // флаг - коннектиться ли к роутеру (1 - коннектиться, 0 - не коннектиться)
      'F', 'l', 'y', '\0', // SSID роутера (набор байт, заканчивающийся нулевым байтом)
      'H', 't', 'R', 'j', '7', 'U', 'J', 'Q', '\0', // пароль роутера (набор байт, заканчивающийся нулевым байтом)
      12, // скорость работы с ESP (1 - 9600, 2 - 19200, 4 - 38400, 6 - 57600, 12 - 115200)
      1, // номер Serial, который используется для работы с ESP (1 - Serial1, 2 - Serial2, 3 - Serial3)
      0, // использовать ли пин пересброса питания при зависании ESP (0 - не использовать, 1 - использовать)
      0, // номер пина для пересброса питания ESP
      30, // кол-во секунд, по истечении которых модем считается зависшим (не пришёл ответ на команду)
      2, // сколько секунд держать питание выключенным при перезагрузке ESP, если он завис
      2, // сколько секунд ждать загрузки модема при инициализации/переинициализации
      1, // уровень для включения питания (1 - HIGH, 0 - LOW)       
    //---------------------------------------
      FractDelimiterRecord, // // разделитель целой и дробной частей
      ',', // у нас этим разделителем будет запятая
    //---------------------------------------
      TemperatureUnitRecord, // // вид измеряемой температуры (цельсии - UnitCelsius или фаренгейты - UnitFahrenheit)
      UnitCelsius, // у нас будут цельсии
    //---------------------------------------
      SensorsUpdateIntervalRecord, // интервал опроса датчиков, в секундах
      5, // у нас будет 5 секунд между опросами
    //---------------------------------------
  
  // окончание конфига
  CORE_HEADER1
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
/*
#ifdef _CORE_DEBUG
//--------------------------------------------------------------------------------------------------------------------------------------
void myConnect(CoreTransportClient& client)
{
  Serial.print(F("Client #"));
  Serial.print(client.getID());
  if(client.connected())
    Serial.println(F(" connected."));
  else
    Serial.println(F(" disconnected."));
}
//--------------------------------------------------------------------------------------------------------------------------------------
void myWriteDone(CoreTransportClient& client, bool isWriteSucceeded)
{
  Serial.print(F("Client #"));
  Serial.print(client.getID());
  Serial.println(F(" write done."));
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif  // _CORE_DEBUG
*/
//--------------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(CORE_COMMUNICATION_SPEED);


/*
  // тестируем внешний транспорт
  #ifdef _CORE_DEBUG
  
    CoreSerialTransport transport;
    transport.init({myConnect,NULL, myWriteDone});

    CoreTransportClient* client = transport.getFreeClient();

    if(client)
    {
      client->connect("",0);
      const char* str = "Hello, world!\n";
      client->write((const uint8_t*) str,strlen(str));
    }
    else
    {
      Serial.println(F("NO FREE CLIENT FOUND!!!"));
    }
    
  #endif 
*/  

  // будем для теста мигать встроенным светодиодом, проверяя корректность отработки слежения за портом
  led13Blinker.init(LED_BUILTIN, 23, 100, 1200); // мигаем светодиодом на 13 пине, держа 100 мс включенным, и 1200 мс - выключенным. Мигаем 23 раза, потом - перестаёт мигать.
  
  
  // все необработанные данные из Serial будут перенаправлены в функцию unhandledCommandHandler
  Core.setup(unhandledCommandHandler);

  // печатаем версию ядра
  Core.printVersion(Serial);

  // печатаем информацию о поддерживаемых датчиках
  //Serial.print(F("Supported sensors: "));
  //Core.printSupportedSensors(Serial);
  

  ramFree();

      // грузим конфиг из flash в EEPROM
  Core.saveConfig(unitTestConfig,sizeof(unitTestConfig),true);
  if(Core.loadConfig())
  {
    Serial.println(F("LOADED FROM EEPROM!"));
  }     
  else
  {
    Serial.println(F("ERROR LOADING CONFIG!!!"));
  }


  /*

  // пробуем загрузить конфиг из EEPROM, если не получится - грузим из флеша конфиг по умолчанию
  if(Core.loadConfig())
  {
    Serial.println(F("LOADED FROM EEPROM!"));
  }
  else
  {
    // грузим конфиг из flash в EEPROM
    Core.saveConfig(unitTestConfig,sizeof(unitTestConfig),true);     
    Serial.println(F("SAVED FROM FLASH TO EEPROM!"));

    // перезагружаем из EEPROM
    if(Core.loadConfig())
    {
      Serial.println(F("LOADED FROM EEPROM AFTER SAVE!"));
    }
  }
*/


  ramFree();
    

}
//--------------------------------------------------------------------------------------------------------------------------------------
unsigned long lastMillis = 0;
void loop() 
{
  // обрабатываем входящие команды
  Core.handleCommands();

  // обновляем показания с датчиков
  Core.update();

  // тут пробуем поймать изменение состояния порта
  CoreDataList catchList = CoreDataStore.getBySensor(DigitalPortState);
  for(size_t i=0;i<catchList.size();i++)
  {
    CoreStoredData dataStored = catchList[i];
    DigitalPortData dt = dataStored;
    if(dt.Pin == LED_BUILTIN)
    {
      // наш тестовый пин
      static int state = -1;
      if(state == -1)
        state = dt.Value;

      if(state != dt.Value)
      {
        state = dt.Value;
        
        Serial.println(F("-----------------------------------------------------------------"));
        Serial.print(F("BUILTIN LED TRIGGERED, VALUE: "));
        if(dt.Value)
          Serial.println(F("HIGH"));
        else          
          Serial.println(F("LOW"));
      }
    }
  } // for
  unsigned long curMillis = millis();


  // обновляем нашу тестовую мигалку
  led13Blinker.update();


  if(curMillis - lastMillis > 3000) // каждые 3 секунды и выводим данные с датчиков в Serial
  {
    // данные с датчиков обновляются ядром автоматически, тут нам надо только их прочесть
    
    lastMillis =  curMillis;
    
    
    CoreTextFormatProvider textFormatter;

    Serial.println(F("\nGET ALL SENSORS"));
    
    for(size_t i=0;i<CoreDataStore.size();i++)
    {
      CoreStoredData dataStored = CoreDataStore.get(i);
      
      // получаем данные, отформатированные в текстовом виде
      String data = textFormatter.format(dataStored,i,true); // последний параметр - выводить ли единицы измерения

      // и печатаем их
      String sensorName = dataStored.sensor->getName();
      if(sensorName.length())
      {
           Serial.print(F("\t"));
           Serial.print(sensorName);
           Serial.print(F(": "));
      }
      else
      {
              Serial.print(F("\tSENSOR DATA: "));
      }
      Serial.println(data);            

    } // for

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ПРИМЕР ПОЛУЧЕНИЯ ДАННЫХ С DS3231 в виде структуры (датчику в конфиге прописано имя "RTC")
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     DateTimeData currentDateTime = CoreDataStore.get(F("RTC"));
     Serial.print(F("DATETIME: "));
     Serial.print(currentDateTime.day);
     Serial.print('.');
     Serial.print(currentDateTime.month);
     Serial.print('.');
     Serial.print(currentDateTime.year);
     Serial.print(' ');

     Serial.print(currentDateTime.hour);
     Serial.print(':');     
     Serial.print(currentDateTime.minute);
     Serial.print(':');     
     Serial.print(currentDateTime.second);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ЧИТАЕМ НАШИ АНАЛОГОВЫЕ ПИНЫ
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Serial.println(F("\nGET ANALOG INPUTS:"));
    CoreDataList allAnalog = CoreDataStore.getBySensor(AnalogPortState);

    // проходим по каждому, печатаем их в Serial (отправляем во внешний мир и т.п.)
    for(size_t i=0;i<allAnalog.size();i++)
    {
      CoreStoredData dataStored = allAnalog[i];
      String data = textFormatter.format(dataStored,i,true); // последний параметр - выводить ли единицы измерения

      // и печатаем их
      Serial.print(F("\tANALOG DATA: "));
      Serial.println(data);

      // теперь проверяем - это значение с пина power_3V3 ?
      AnalogPortData dt = dataStored;
      if(dt.Pin == power_3V3)
      {
        Serial.println(F("\tpower_3V3 FOUND!"));
      }
      
    } // for

    // теперь просто получаем данные с датчика по его строковому имени
    AnalogPortData current5V = CoreDataStore.get(F("5V"));
    Serial.print(F("\tCURRENT 5V VALUE: "));
    Serial.println(current5V.Value);


    // теперь получаем данные по типу датчика и пину
    AnalogPortData my5VData = CoreDataStore.get(AnalogPortState,power_200);
    Serial.print(F("\tCURRENT power_200 VALUE: "));
    Serial.println(my5VData.Value);
    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // АНАЛОГОВЫЕ ПИНЫ ПРОЧИТАНЫ
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // теперь получаем показания всех датчиков влажности
    Serial.println(F("\nGET ALL HUMIDITY"));
    CoreDataList allHumidity = CoreDataStore.getByType(Humidity);

    // и печатаем их
    for(size_t i=0;i<allHumidity.size();i++)
    {
      CoreStoredData dataStored = allHumidity[i];
      // получаем данные, отформатированные в текстовом виде
      String data = textFormatter.format(dataStored,i,true); // последний параметр - выводить ли единицы измерения

      // и печатаем их
      Serial.print(F("\tHUMIDITY: "));
      Serial.println(data);            
    }

    // теперь получаем показания всех датчиков BH1750
    Serial.println(F("\nGET ALL BH1750"));
    CoreDataList allBH1750 = CoreDataStore.getBySensor(BH1750);

    // и печатаем их
    for(size_t i=0;i<allBH1750.size();i++)
    {
      CoreStoredData dataStored = allBH1750[i];
      // получаем данные, отформатированные в текстовом виде
      String data = textFormatter.format(dataStored,i,true); // последний параметр - выводить ли единицы измерения

      // и печатаем их
      Serial.print(F("\tBH1750: "));
      Serial.println(data);            
    }    

   // теперь получаем данные о времени и дате
    Serial.println(F("\nGET DATETIME"));
    CoreDataList allDateTime = CoreDataStore.getByType(DateTime);
    
    // и печатаем их
    for(size_t i=0;i<allDateTime.size();i++)
    {
      CoreStoredData dataStored = allDateTime[i];
      // получаем данные, отформатированные в текстовом виде
      String data = textFormatter.format(dataStored,i,true); // последний параметр - выводить ли единицы измерения

      // и печатаем их
      Serial.print(F("\tDATETIME: "));
      Serial.println(data);            
    } 

    // теперь получаем температуру с часов реального времени DS3231
    Serial.println(F("\nGET RTC Temp"));
    CoreDataList allRTCTemp = CoreDataStore.getBySensor(DS3231Temperature);
    
    // и печатаем их
    for(size_t i=0;i<allRTCTemp.size();i++)
    {
      CoreStoredData dataStored = allRTCTemp[i];
      // получаем данные, отформатированные в текстовом виде
      String data = textFormatter.format(dataStored,i,true); // последний параметр - выводить ли единицы измерения

      // и печатаем их
      Serial.print(F("\tRTC temp: "));
      Serial.println(data);            
    }     

    // теперь получаем показания всех датчиков DS18B20
    Serial.println(F("\nGET ALL DS18B20"));
    CoreDataList allDS18B20 = CoreDataStore.getBySensor(DS18B20);

    // и печатаем их
    for(size_t i=0;i<allDS18B20.size();i++)
    {
      CoreStoredData dataStored = allDS18B20[i];
      // получаем данные, отформатированные в текстовом виде
      String data = textFormatter.format(dataStored,i,true); // последний параметр - выводить ли единицы измерения

      // и печатаем их
      Serial.print(F("\tDS18B20: "));
      Serial.println(data);            
    }

    // теперь получаем показания с датчика DS18B20 по его мнемоническому имени
    CoreStoredData ds2Temperature = CoreDataStore.get(F("DS2"));
    String ds2StringValue = textFormatter.format(ds2Temperature,0,true); // последний параметр - выводить ли единицы измерения
    Serial.print(F("\tGET BY MNEMONIC 'DS2' NAME: "));
    Serial.println(ds2StringValue);         
       
    // теперь получаем показания всех датчиков слежения за цифровыми портами
    Serial.println(F("\nGET ALL DIGITAL PINS STATE"));
    CoreDataList allDPins = CoreDataStore.getBySensor(DigitalPortState);

    // и печатаем их
    for(size_t i=0;i<allDPins.size();i++)
    {
      CoreStoredData dataStored = allDPins[i];
      // получаем данные, отформатированные в текстовом виде
      String data = textFormatter.format(dataStored,i,true); // последний параметр - выводить ли единицы измерения

      // и печатаем их
      Serial.print(F("\tPIN: "));
      Serial.println(data);            
    }    
    
    ramFree();

    // печатаем пару пустых строк, чтобы было видно блок
    Serial.println();
    Serial.println();
    
  } // if(curMillis - lastMillis > 3000)


}
//--------------------------------------------------------------------------------------------------------------------------------------

